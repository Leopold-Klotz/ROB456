#!/usr/bin/env python3

# Publish a point that moves in a circle, so that we can visualize it with rviz.
#
# circle.py
#
# Bill Smart, smartw@oregonstate.edu
#
# Edited 11/7/23, Leopold Klotz, Klotzl@oregonstate.edu. Transition to ROS2


# Import ROS Python basic API and sys

##ROS1 Code
# import rospy
import sys

#ros2
import rclpy

# We're going to need to do some math
from math import sin, cos

# This time we're going to be using PointStamped, which we get from geometry_msgs.
from geometry_msgs.msg import PointStamped


if __name__ == '__main__':
	# Initialize the node, and call it "circler".

	##ROS1 Code
	# rospy.init_node('circler', argv=sys.argv)
	#ROS2 Code
	rclpy.init(args=sys.argv)
	node = rclpy.create_node('circler')

	# Set up a publisher.  This will publish on a topic called "dot", with a
	# message type of Point.

	##ROS1 Code
	# publisher = rospy.Publisher('dot', PointStamped, queue_size=10)
	#ROS2 Code
	publisher = node.create_publisher(PointStamped, 'dot', 10)

	# We're going to move the point around a circle.  To do this, we're going
	# to keep track of how far around the circle it is with an angle.  We're also
	# going to define how far it moves in each step.
	theta = 0.0
	theta_inc = 0.05

	# Rate allows us to control the (approximate) rate at which we publish things.
	# For this example, we want to publish at 10Hz.

	##ROS1 Code
	# rate = rospy.Rate(10)
	#ROS2 Code
	rate = node.create_rate(10)

	# This will loop until ROS shuts down the node.  This can be done on the
	# command line with a ctrl-C, or automatically from roslaunch.

	##ROS1 Code
	# while not rospy.is_shutdown():
	#ROS2 Code
	while rclpy.ok():
		# Make a point instance, and fill in the information.
		p = PointStamped()

		##ROS1 Code
		# p.header.stamp = rospy.Time.now()
		#ROS2 Code
		p.header.stamp = node.get_clock().now().to_msg()

		p.header.frame_id = 'map'
		p.point.x = cos(theta)
		p.point.y = sin(theta)
		p.point.z = 0.0

		# Publish the value of the counter.
		publisher.publish(p)

		# Print out a log message to the INFO channel to let us know it's working.

		##ros1
		# rospy.loginfo(f'Published point at ({p.point.x}, {p.point.y})')
		#ros2
		node.get_logger().info(f'Published point at ({p.point.x}, {p.point.y})')

		# Increment theta.  This will grow without bound, which is bad if we run
		# the node for long enough, but we're not going to worry about it for this
		# toy example.
		theta += theta_inc

		# Do an idle wait to control the publish rate.  If we don't control the
		# rate, the node will publish as fast as it can, and consume all of the
		# available CPU resources.  This will also add a lot of network traffic,
		# possibly slowing down other things.
		rate.sleep()