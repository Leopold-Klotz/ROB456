#!/usr/bin/env python3


# Basic topic subscriber example.
#
# subscriber.py
#
# Bill Smart, smartw@oregonstate.edu
#
# This example shows the basic code for subscribing to a topic.
#
# Edited 11/7/23, Leopold Klotz, Klotzl@oregonstate.edu. Transition to ROS2


# Import ROS Python basic API and sys

##ROS1 Code
# import rospy
import sys

#ros2
import rclpy

# We're going to subscribe to 64-bit integers, so we need to import the defintion
# for them.
from std_msgs.msg import Int64


# This is a function that is called whenever a new message is received.  The
# message is passed to the function as a parameter.
def callback(msg):
	"""
	Callback function to deal with incoming messages.
	:param msg: The message.
	"""

	# The value of the integer is stored in the data attribute of the message.
	
	##ROS1 Code
	# rospy.loginfo(f'Got {msg.data}')
	#ROS2 Code
	print(f'Got {msg.data}')


if __name__ == '__main__':
	# Initialize the node.

	##ROS1 Code
	# rospy.init_node('subscriber', argv=sys.argv)
	#ROS2 Code
	rclpy.init(args=sys.argv)
	node = rclpy.create_node('subscriber')

	# Set up a subscriber.  We're going to subscribe to the topic "counter",
	# looking for Int64 messages.  When a message comes in, ROS is going to pass
	# it to the function "callback" automatically.

	##ROS1 Code
	# subscriber = rospy.Subscriber('counter', Int64, callback)
	#ROS2 Code
	subscriber = node.create_subscription(Int64, 'counter', callback, 10)

	# Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages.

	##ROS1 Code
	# rospy.spin()
	#ROS2 Code
	rclpy.spin(node)
	
