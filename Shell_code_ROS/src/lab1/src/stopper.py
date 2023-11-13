#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to do some math
import numpy as np

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


# A callback to deal with the LaserScan messages.
def callback(scan):
	# Every time we get a laser scan, calculate the shortest scan distance in front
	# of the robot, and set the speed accordingly.  We assume that the robot is 38cm
	# wide.  This means that y-values with absolute values greater than 19cm are not
	# in front of the robot.  It also assumes that the LiDAR is at the front of the
	# robot (which it actually isn't) and that it's centered and pointing forwards.
	# We can work around these assumptions, but it's cleaner if we don't

	# Pulling out some useful values from scan
	angle_min = scan.angle_min
	angle_max = scan.angle_max
	num_readings = len(scan.ranges)

	# Doing this for you - get out theta values for each range/distance reading
	thetas = np.linspace(angle_min, angle_max, num_readings)

	# TODO: Determine what the closest obstacle/reading is for scans in front of the robot
	#  Step 1: Determine which of the range readings correspond to being "in front of" the robot (see comment at top)
	#    Remember that robot scans are in the robot's coordinate system - theta = 0 means straight ahead
	#  Step 2: Get the minimum distance to the closest object
	#  Step 3: Use the closest distance from above to decide when to stop
	#  Step 4: Scale how fast you move by the distance to the closet object (tanh is handy here...)
	#  Step 5: Make sure to actually stop if close to 1 m
	# Finally, set t.linear.x to be your desired speed (0 if stop)
	# Suggestion: Do this with a for loop before being fancy with numpy (which is substantially faster)
	# DO NOT hard-wire in the number of readings, or the min/max angle. You CAN hardwire in the size of the robot

	# Create a twist and fill in all the fields (you will only set t.linear.x).
	t = Twist()
	t.linear.x = 0.0
	t.linear.y = 0.0
	t.linear.z = 0.0
	t.angular.x = 0.0
	t.angular.y = 0.0
	t.angular.z = 0.0

	# YOUR CODE HERE
	# step 1: what is in front of the robot (19 cm )
	ranges = np.array(scan.ranges)
	y_values = ranges * np.sin(thetas)
	front_indices = np.where(np.abs(y_values) < 0.19)

	# Initially I thought that this would correspond to the front of the robot becuase it would be the fov in the front quarter
	# will ask about this in office hours or in class
	# front_indices = np.where(np.abs(thetas) < np.pi / 4)

	# step 2
	front_ranges = np.array(scan.ranges)[front_indices]

	# step 3
	shortest = np.min(front_ranges)

	# step 4
	if shortest < 1: # stop if 1 m
		speed = 0.0
	else: # slow down if getting close
		distance_scale = np.tanh(shortest - 1)
		speed = 1 * distance_scale # 1 m/s scaled by distance

	t.linear.x = speed

	# Send the command to the robot.
	publisher.publish(t)

	# Print out a log message to the INFO channel to let us know it's working.
	rospy.loginfo(f'Shortest: {shortest} => {t.linear.x}')


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('stopper', argv=sys.argv)

	# Set up a publisher.  The default topic for Twist messages is cmd_vel.
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
	subscriber = rospy.Subscriber('base_scan', LaserScan, callback, queue_size=10)

	# Now that everything is wired up, we just spin.
	rospy.spin()
