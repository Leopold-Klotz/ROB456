#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import sin, cos
from geometry_msgs.msg import PointStamped

class CirclerNode(Node):
    def __init__(self):
        super().__init__('circler')
        self.publisher = self.create_publisher(PointStamped, 'dot', 10)
        self.timer = self.create_timer(0.1, self.publish_circle)  # Timer callback at 10 Hz
        self.theta = 0.0
        self.theta_inc = 0.05

    def publish_circle(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = cos(self.theta)
        msg.point.y = sin(self.theta)
        msg.point.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Published point at ({msg.point.x}, {msg.point.y})')
        self.theta += self.theta_inc

def main(args=None):
    rclpy.init(args=args)
    circler_node = CirclerNode()
    rclpy.spin(circler_node)

    circler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()