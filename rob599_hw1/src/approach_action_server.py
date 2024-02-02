#!/usr/bin/env python3


# approach_action_server.py
#
# Marcus Rosette
#
# This shows how to set up a simple action server. 


import rospy
import actionlib
import sys

from time import sleep


# Since actions are not first-class citizens, they're implemented using a set of automatically
# defined messages.  You'll need to import the Action, Goal, Feedback, and Result messages.
from rob599_hw1.msg import ApproachAction, ApproachGoal, ApproachFeedback, ApproachResult


class RobotActionServer:
# This is called when an action request is received.  The goal is passed as the function argument.
    def callback(self, goal):
        # Publish feedback, so that the caller knows about progress, if it cares.
        server.publish_feedback(ApproachFeedback(progress=min(ranges)))

        # Did we receive a new goal?  If so, preempt the current one.  We set the status of the goal to
        # preempted, and return from the callback.  If you don't want actions to be preemptable, then
        # don't put this code in the callback.  Polling for preemption here allows you to control when
        # and how actions are preempted.
        if server.is_new_goal_available():
            server.set_preempted(ApproachResult(success=result))
            return

        # Artificially wait for a bit.
        sleep(1)

        # Once we have the result assembled, we signal to the action server back end that it's available
        # with the set_succeeded() function.  There are also equivalent set_aborted() amd set_preempted()
        # calls.
        server.set_succeeded(ApproachResult(success=result))


if __name__ == '__main__':
	# Initilaize the node
	rospy.init_node('fibber', argv=sys.argv)

	# Set up the action server.  The action is called "fibonacci", and has type FibonacciAction.  The
	# function "callback" is called on new requests.  The final argument tells the action client back
	# end not to start autmatically.  You should always set this to False, since if you do otherwise,
	# then you risk race conditions (since the action server back end spins up new threads).
	server = actionlib.SimpleActionServer('fibonacci', FibonacciAction, callback, False)

	# Start the server.  By explicitly calling this function here, you can guarantee that the server
	# back end is properly started and ready to use.
	server.start()
	rospy.loginfo('Fibonacci action server started')

	# Give control over to ROS.
	rospy.spin()

