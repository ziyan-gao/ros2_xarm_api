import time
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped

from xarm.wrapper import XArmAPI


class SafeServo(Node):
    def __init__(self):
        super().__init__('safe_servo')

        # =========================================================
        # Parameters
        # =========================================================
        self.declare_parameter('robot_ip', '192.168.1.232')

        self.declare_parameter('command_topic', '/servo_command')
        self.declare_parameter('force_topic', '/ufactory/uf_ftsensor_ext_states')

        # PD
        self.declare_parameter('kp', 0.25)
        self.declare_parameter('kd', 0.02)

        # control loop
        self.declare_parameter('control_period', 0.02)     # 50 Hz
        self.declare_parameter('max_step_xy', 3.0)         # mm per cycle

        # boundary (first version, plane only)
        self.declare_parameter('x_min', 160.0)
        self.declare_parameter('x_max', 390.0)
        self.declare_parameter('y_min', -360.0)
        self.declare_parameter('y_max', 360.0)

        # z range / fixed orientation
        self.declare_parameter('z_min', 50.0)
        self.declare_parameter('z_max', 80.0)
        self.declare_parameter('roll_fixed', 180.0)
        self.declare_parameter('pitch_fixed', 0.0)
        self.declare_parameter('yaw_fixed', 0.0)

        # collision-force-limit
        self.declare_parameter('force_limit', 15.0)
        self.declare_parameter('resume_force_ratio', 0.6)

        # if no force message for too long -> stop for safety
        self.declare_parameter('force_timeout_sec', 0.5)

        # =========================================================
        # Read parameters
        # =========================================================
        self.robot_ip = self.get_parameter('robot_ip').value
        self.command_topic = self.get_parameter('command_topic').value
        self.force_topic = self.get_parameter('force_topic').value

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)

        self.control_period = float(self.get_parameter('control_period').value)
        self.max_step_xy = float(self.get_parameter('max_step_xy').value)

        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)

        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.roll_fixed = float(self.get_parameter('roll_fixed').value)
        self.pitch_fixed = float(self.get_parameter('pitch_fixed').value)
        self.yaw_fixed = float(self.get_parameter('yaw_fixed').value)

        self.force_limit = float(self.get_parameter('force_limit').value)
        self.resume_force_ratio = float(self.get_parameter('resume_force_ratio').value)
        self.force_timeout_sec = float(self.get_parameter('force_timeout_sec').value)

        # =========================================================
        # Internal state
        # =========================================================
        self.latest_pose: Optional[List[float]] = None   # [x, y, z, roll, pitch, yaw]
        self.latest_force_norm: Optional[float] = None

        self.last_force_time = None

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.paused_by_force = False
        self.warned_force_timeout = False

        # =========================================================
        # xArm SDK init
        # =========================================================
        self.arm = XArmAPI(self.robot_ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)   # servo mode
        self.arm.set_state(0)
        time.sleep(0.1)

        # =========================================================
        # Subscribers
        # =========================================================
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            self.command_topic,
            self.command_callback,
            10
        )

        self.force_sub = self.create_subscription(
            WrenchStamped,
            self.force_topic,
            self.force_callback,
            10
        )

        # =========================================================
        # Timer
        # =========================================================
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info('safe_servo started')
        self.get_logger().info(f'robot_ip      = {self.robot_ip}')
        self.get_logger().info(f'command_topic = {self.command_topic}')
        self.get_logger().info(f'force_topic   = {self.force_topic}')

    # =========================================================
    # command callback
    # =========================================================
    def command_callback(self, msg: Float64MultiArray):
        """
        Expect:
            [x, y, z, roll, pitch, yaw]

        First version:
            x, y, z are used
            z is clamped to safe range [z_min, z_max]
            rpy are forced to fixed values
        """
        if len(msg.data) < 6:
            self.get_logger().warn(
                f'Invalid servo command length={len(msg.data)}, expected 6'
            )
            return

        raw_x = float(msg.data[0])
        raw_y = float(msg.data[1])
        raw_z = float(msg.data[2])

        x = self.clamp(raw_x, self.x_min, self.x_max)
        y = self.clamp(raw_y, self.y_min, self.y_max)
        z = self.clamp(raw_z, self.z_min, self.z_max)

        self.latest_pose = [
            x,
            y,
            z,
            self.roll_fixed,
            self.pitch_fixed,
            self.yaw_fixed
        ]

        self.get_logger().info(
            f'latest_pose updated: raw=({raw_x:.2f}, {raw_y:.2f}, {raw_z:.2f}) '
            f'-> bounded=({x:.2f}, {y:.2f}, {z:.2f}, '
            f'{self.roll_fixed:.2f}, {self.pitch_fixed:.2f}, {self.yaw_fixed:.2f})'
        )

    # =========================================================
    # force callback
    # =========================================================
    def force_callback(self, msg: WrenchStamped):
        fx = float(msg.wrench.force.x)
        fy = float(msg.wrench.force.y)
        fz = float(msg.wrench.force.z)

        force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)

        self.latest_force_norm = force_norm
        self.last_force_time = time.time()
        self.warned_force_timeout = False

    # =========================================================
    # main control loop
    # =========================================================
    def control_loop(self):
        # no target -> do nothing
        if self.latest_pose is None:
            return

        # -----------------------------------------------------
        # 1. force timeout check
        # -----------------------------------------------------
        if self.last_force_time is None:
            if not self.warned_force_timeout:
                self.get_logger().warn('No force message received yet, motion paused for safety')
                self.warned_force_timeout = True
            return

        dt_force = time.time() - self.last_force_time
        if dt_force > self.force_timeout_sec:
            if not self.warned_force_timeout:
                self.get_logger().warn(
                    f'Force topic timeout ({dt_force:.3f}s > {self.force_timeout_sec:.3f}s), motion paused for safety'
                )
                self.warned_force_timeout = True
            return

        # -----------------------------------------------------
        # 2. force limit check
        # -----------------------------------------------------
        if self.latest_force_norm is not None:
            if self.latest_force_norm >= self.force_limit:
                if not self.paused_by_force:
                    self.paused_by_force = True
                    self.get_logger().warn(
                        f'Force limit reached: {self.latest_force_norm:.3f} >= {self.force_limit:.3f}, pause motion'
                    )
                return

            if self.paused_by_force:
                if self.latest_force_norm < self.force_limit * self.resume_force_ratio:
                    self.paused_by_force = False
                    self.get_logger().info(
                        f'Force back to safe range: {self.latest_force_norm:.3f}, resume motion'
                    )
                else:
                    return

        # -----------------------------------------------------
        # 3. read current robot pose
        # -----------------------------------------------------
        current_pose = self.read_current_pose()
        if current_pose is None:
            return

        # -----------------------------------------------------
        # 4. compute next safe pose by PD
        # -----------------------------------------------------
        next_pose = self.compute_next_pose(current_pose, self.latest_pose)

        # -----------------------------------------------------
        # 5. send servo command
        # -----------------------------------------------------
        ret = self.arm.set_servo_cartesian(
            next_pose,
            speed=100,
            mvacc=2000
        )

        if ret != 0:
            self.get_logger().warn(f'set_servo_cartesian ret={ret}')

    # =========================================================
    # PD on x/y only
    # =========================================================
    def compute_next_pose(self, current_pose: List[float], target_pose: List[float]) -> List[float]:
        current_x = current_pose[0]
        current_y = current_pose[1]

        target_x = target_pose[0]
        target_y = target_pose[1]

        dt = self.control_period

        error_x = target_x - current_x
        error_y = target_y - current_y

        d_error_x = (error_x - self.prev_error_x) / dt
        d_error_y = (error_y - self.prev_error_y) / dt

        ux = self.kp * error_x + self.kd * d_error_x
        uy = self.kp * error_y + self.kd * d_error_y

        # step limit
        ux = self.clamp(ux, -self.max_step_xy, self.max_step_xy)
        uy = self.clamp(uy, -self.max_step_xy, self.max_step_xy)

        next_x = current_x + ux
        next_y = current_y + uy

        # boundary again for safety
        next_x = self.clamp(next_x, self.x_min, self.x_max)
        next_y = self.clamp(next_y, self.y_min, self.y_max)

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        next_z = self.clamp(target_pose[2], self.z_min, self.z_max)

        return [
            next_x,
            next_y,
            next_z,
            self.roll_fixed,
            self.pitch_fixed,
            self.yaw_fixed
        ]

    # =========================================================
    # read current pose from SDK
    # =========================================================
    def read_current_pose(self) -> Optional[List[float]]:
        try:
            ret = self.arm.get_position(is_radian=False)

            # Common form: (code, [x, y, z, roll, pitch, yaw])
            if isinstance(ret, tuple) and len(ret) >= 2:
                code, pose = ret[0], ret[1]
                if code == 0 and pose is not None and len(pose) >= 6:
                    return list(pose[:6])

            self.get_logger().warn(f'Unexpected get_position return: {ret}')
            return None

        except Exception as e:
            self.get_logger().error(f'Exception in read_current_pose: {e}')
            return None

    # =========================================================
    # utils
    # =========================================================
    @staticmethod
    def clamp(value: float, vmin: float, vmax: float) -> float:
        return max(vmin, min(vmax, value))


def main(args=None):
    rclpy.init(args=args)
    node = SafeServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()