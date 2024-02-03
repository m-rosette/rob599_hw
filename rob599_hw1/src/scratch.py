#!/usr/bin/env python3


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to subscribe to 64-bit integers, so we need to import the defintion
# for them.
from std_msgs.msg import Int64, Float64
from sensor_msgs.msg import LaserScan


# This is a function that is called whenever a new message is received.  The
# message is passed to the function as a parameter.
def callback(msg):
	"""
	Callback function to deal with incoming messages.
	:param msg: The message.
	"""

	# The value of the integer is stored in the data attribute of the message.
	# rospy.loginfo('Got {0}'.format(min(msg.ranges)))
	rospy.loginfo(f"Got {msg.data}")


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('subscriber', argv=sys.argv)
	
    # Limit the publication rate.
	rospy.Rate(10)

	# Set up a subscriber.  We're going to subscribe to the topic "counter",
	# looking for Int64 messages.  When a message comes in, ROS is going to pass
	# it to the function "callback" automatically.
	subscriber = rospy.Subscriber('/wall_angle', Float64, callback)

	# Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages.
	rospy.spin()

