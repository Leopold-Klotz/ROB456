#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # Timer callback at 10 Hz
        self.speed = 0.2
        self.turn = 0.0

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.turn

        self.publisher.publish(msg)
        self.get_logger().info(f'Published twist ({msg.linear.x}, {msg.angular.z})')

def main(args=None):
    rclpy.init(args=args)
    driver_node = DriverNode()
    rclpy.spin(driver_node)

    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()