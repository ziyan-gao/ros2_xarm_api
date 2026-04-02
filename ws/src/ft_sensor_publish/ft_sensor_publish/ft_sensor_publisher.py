import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped

from xarm.wrapper import XArmAPI
from geometry_msgs.msg import WrenchStamped
import pdb

class FTSensorPublisher(Node):
    def __init__(self):
        super().__init__('ft_sensor_publisher')

        self.declare_parameter('robot_ip', '192.168.1.232')
        # =========================================================
        # Publisher
        # =========================================================
        self.get_logger().info('ft_sensor_publisher started')
        self.robot_ip = self.get_parameter('robot_ip').value
        self.get_logger().info(f'robot_ip      = {self.robot_ip}')
        self.ft_ext_publisher = self.create_publisher(WrenchStamped, '/ufactory/uf_ftsensor_ext_states', 10)
        self.ft_raw_publisher = self.create_publisher(WrenchStamped, '/ufactory/uf_ftsensor_raw_states', 10)
        # =========================================================
        # API
        # ========================================================
        self.armApi = XArmAPI(self.robot_ip, enable_report=True)
        self.armApi.motion_enable(enable=True)
        self.armApi.set_ft_sensor_enable(0)

        self.armApi.clean_error()
        self.armApi.clean_warn()
        self.armApi.set_ft_sensor_enable(1)
        time.sleep(0.5)
        self.armApi.set_ft_sensor_zero()

        # =========================================================
        # Timer
        # =========================================================
        self.timer = self.create_timer(0.1, self.publish_ft_sensor_data)
    
    def publish_ft_sensor_data(self):
        raw_ft = self.armApi.ft_raw_force
        ext_ft = self.armApi.ft_ext_force
        raw_msg = WrenchStamped()
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        raw_msg.wrench.force.x = raw_ft[0]
        raw_msg.wrench.force.y = raw_ft[1]
        raw_msg.wrench.force.z = raw_ft[2]
        raw_msg.wrench.torque.x = raw_ft[3]
        raw_msg.wrench.torque.y = raw_ft[4]
        raw_msg.wrench.torque.z = raw_ft[5]
        self.ft_raw_publisher.publish(raw_msg)      
        ext_msg = WrenchStamped()
        ext_msg.header.stamp = self.get_clock().now().to_msg()
        ext_msg.wrench.force.x = ext_ft[0]          
        ext_msg.wrench.force.y = ext_ft[1]
        ext_msg.wrench.force.z = ext_ft[2]
        ext_msg.wrench.torque.x = ext_ft[3]
        ext_msg.wrench.torque.y = ext_ft[4]
        ext_msg.wrench.torque.z = ext_ft[5]
        self.ft_ext_publisher.publish(ext_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FTSensorPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()