"""Graceful restart of one allowlisted application node, owned by this monitor.

No hardware/Servo/MoveIt process management, name-based kill, process-group
signals, forced termination, or automatic restart after a crash.
"""
import json
import math
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ros2run.api import get_executable_path

# Stable application executables only; inventory owners remain outside this list.
APPLICATIONS = {
    'pickup_supervisor': 'safe_servo_visualization',
    'motion_coordinator': 'safe_servo_visualization',
    'pickup_pipeline': 'safe_servo_visualization',
    'place_pipeline': 'safe_servo_visualization',
    'pick_place_pipeline': 'safe_servo_visualization',
    'random_stable_loading': 'safe_servo_visualization',
    'policy_loading': 'safe_servo_visualization',
    'item_localization': 'safe_servo_visualization',
    'waypoint_store': 'safe_servo_visualization',
    'visualization_node': 'safe_servo_visualization',
    'box_marker_detector': 'box_marker_detection',
    'depth_box_refinement': 'box_marker_detection',
}
STOPPED = {'IDLE', 'READY', 'SUCCEEDED', 'FAULT', 'PREPARED', 'OBJECT_INFO_READY', 'AWAITING_GRASP'}


class ApplicationRestart(Node):
    def __init__(self, name, arguments):
        if name not in APPLICATIONS:
            raise ValueError('Application is not allowlisted: '+name)
        super().__init__('restart_' + name, use_global_arguments=False)
        self.identity = name
        executable = get_executable_path(package_name=APPLICATIONS[name], executable_name=name)
        if not executable:
            raise ValueError('Application executable is unavailable: '+name)
        self.command = [executable, *arguments]
        self.process = None
        self.stopping = False
        self.message = ''
        self.deadline = 0.
        self.joints_time = 0.
        self.joints = None
        self.stationary_since = None
        self.session = {}
        self.session_time = 0.
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/pick_place_test/status', self.on_session, 10)
        self.publisher = self.create_publisher(String, '/test_nodes/status', 10)
        self.create_service(Trigger, '/test_nodes/' + name + '/restart', self.restart)
        self.create_timer(.25, self.tick)
        self.spawn()

    def spawn(self):
        try:
            self.process = subprocess.Popen(self.command)
            self.message = 'Started; waiting for application readiness'
        except OSError as exc:
            self.process = None
            self.message = str(exc)
        self.stopping = False

    def on_joints(self, msg):
        now = time.monotonic()
        values = dict(zip(msg.name, msg.position))
        valid = len(values) >= 6 and all(math.isfinite(v) for v in values.values())
        still = (valid and self.joints is not None and now-self.joints_time < .5 and
                 all(abs(v-self.joints.get(k, float('inf'))) < .0005 for k, v in values.items()) and
                 all(math.isfinite(v) and abs(v) < .005 for v in msg.velocity))
        self.stationary_since = (self.stationary_since or now) if still else None
        if not still:
            self.joints = values if valid else None
        self.joints_time = now

    def on_session(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.session = data
                self.session_time = time.monotonic()
        except (ValueError, TypeError):
            pass

    def blocked(self):
        now = time.monotonic()
        if self.stopping:
            return 'Restart in progress'
        if now-self.session_time > 3.:
            return 'Task session status unavailable'
        if (self.session.get('state') not in ('IDLE', 'READY', 'FAULT') or
                self.session.get('random_active') or self.session.get('reset_in_progress')):
            return 'Abort/stop the task before restarting a dependency'
        if (now-self.joints_time > .5 or self.stationary_since is None or
                now-self.stationary_since < 1.):
            return 'Waiting for fresh stationary joints (1 second)'
        if self.session.get('automatic_motion_active'):
            return 'Disable automatic loading before restarting a dependency'
        owners = dict(pickup_supervisor='supervisor', motion_coordinator='motion',
                      pickup_pipeline='pickup', place_pipeline='place',
                      pick_place_pipeline='cycle', random_stable_loading='random', policy_loading='policy')
        for name, state in self.session.get('downstream', {}).items():
            if name not in ('supervisor', 'motion', 'pickup', 'place', 'cycle', 'staging', 'random', 'policy'):
                continue
            if name == owners.get(self.identity) and (self.process is None or self.process.poll() is not None):
                continue  # A crashed owner cannot publish a newer stopped status.
            if state not in STOPPED:
                return name + ' must be stopped first'
        return ''

    def restart(self, request, response):
        reason = self.blocked()
        if reason:
            response.message = reason
            return response
        self.stopping = True
        self.deadline = time.monotonic() + 5.
        self.message = 'Waiting for old application to exit normally'
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        response.success = True
        response.message = 'Restart requested: ' + self.identity + '; no motion replay'
        return response

    def tick(self):
        if self.stopping:
            if self.process is None or self.process.poll() is not None:
                self.spawn()
            elif time.monotonic() >= self.deadline:
                self.stopping = False
                self.message = 'Exit timed out; old process retained, no replacement started'
        code = self.process.poll() if self.process else None
        self.publisher.publish(String(data=json.dumps(dict(
            id=self.identity, pid=self.process.pid if self.process else None,
            alive=self.process is not None and code is None, exit_code=code,
            restarting=self.stopping, message=self.message, blocked=self.blocked()))))

    def close(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().error('Application has not exited; no forced termination performed')


def main():
    name, *arguments = sys.argv[1:]
    rclpy.init(args=[])
    node = ApplicationRestart(name, arguments)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
