#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class StopperNode(Node):
    def __init__(self):
        super().__init__('stopper')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # Timer callback at 10 Hz
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.shortest = 0.0
        self.speed = 0.0
        self.turn = 0.0

    def scan_callback(self, msg):
        # Existing code to get the angle_min, angle_max, and num_readings from the msg
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        num_readings = len(msg.ranges)

        # Get the theta values for each distance reading
        thetas = np.linspace(angle_min, angle_max, num_readings)

        # YOUR CODE HERE
        # step 1: what is in front of the robot (19 cm )
        ranges = np.array(msg.ranges)
        y_values = ranges * np.sin(thetas)
        front_indices = np.where(np.abs(y_values) < 0.19)

        # Initially I thought that this would correspond to the front of the robot becuase it would be the fov in the front quarter
        # will ask about this in office hours or in class
        # front_indices = np.where(np.abs(thetas) < np.pi / 4)

        # step 2
        front_ranges = np.array(msg.ranges)[front_indices]

        # step 3
        shortest_distance = np.min(front_ranges)

        # step 4
        if shortest_distance < 1: # stop if 1 m
            speed = 0.0
        else: # slow down if getting close
            distance_scale = np.tanh(shortest_distance - 1)
            speed = 1 * distance_scale # 1 m/s scaled by distance

        # Create a twist and fill in all the fields (you will only set t.linear.x).
        t = Twist()
        t.linear.x = speed
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0

        # Send the command to the robot.
        self.publisher.publish(t)

        # Print out a log message to the INFO channel to let us know it's working.
        self.get_logger().info(f'Shortest: {shortest_distance} => {t.linear.x}')

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
    stopper_node = StopperNode()
    rclpy.spin(stopper_node)

    stopper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


