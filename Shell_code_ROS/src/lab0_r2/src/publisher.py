#!/usr/bin/env python3

# Basic topic publisher example.
#
# publisher.py
#
# Bill Smart, smartw@oregonstate.edu
#
# This example shows the basic code for publishing on topics.
#
# Edited 11/7/23, Leopold Klotz, Klotzl@oregonstate.edu. Transition to ROS2


# Import ROS Python basic API and sys
##ROS1 Code
# import rospy
import sys

#ros2
import rclpy


# We're going to publish 64-bit integers, so we need to import this from the
# std_msgs pacakge.  Note that we need to import this from std_msgs.msg.  This
# idiom is common in ROS when importing messages.  You can see all of the basic,
# built-in types with "rosmsg package std_msgs".  We've also added std_msgs to
# to the package.xml file in an <exec_depend> tag.
from std_msgs.msg import Int64


if __name__ == '__main__':
	# Initialize the node, and call it "publisher".

	##ROS1 Code
	# rospy.init_node('publisher', argv=sys.argv)
	#ROS2 Code
	rclpy.init(args=sys.argv)
	node = rclpy.create_node('publisher')


	# Set up a publisher.  This will publish on a topic called "counter", with a
	# message type of Int64.

	##ROS1 Code
	# publisher = rospy.Publisher('counter', Int64, queue_size=10)
	#ROS2 Code
	publisher = node.create_publisher(Int64, 'counter', 10)



	# Rate allows us to control the (approximate) rate at which we publish things.
	# For this example, we want to publish at 1Hz.
	##ROS1 Code
	# rate = rospy.Rate(1)
	#ROS2 Code
	rate = node.create_rate(1)

	# Set up a counter that we're going to increment to give us some data to
	# publish.
	counter = 0

	# This will loop until ROS shuts down the node.  This can be done on the
	# command line with a ctrl-C, or automatically from roslaunch.

	##ROS1 Code
	# while not rospy.is_shutdown():
	#ROS2 Code
	while rclpy.ok():
		# Publish the value of the counter.
		publisher.publish(counter)

		# Print out a log message to the INFO channel to let us know it's working.
		
		##ROS1 Code
		# rospy.loginfo(f'Published {counter}')
		#ROS2 Code
		node.get_logger().info(f'Published {counter}')

		# Increment the counter.
		counter += 1

		# Do an idle wait to control the publish rate.  If we don't control the
		# rate, the node will publish as fast as it can, and consume all of the
		# available CPU resources.  This will also add a lot of network traffic,
		# possibly slowing down other things.
		rate.sleep()