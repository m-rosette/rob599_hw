#!/usr/bin/env python3

import rospy
import actionlib
from rob599_hw1.msg import ApproachAction, ApproachGoal


class RobotActionClient:
    def __init__(self) -> None:
        self.action_node = rospy.init_node("action_node")
        self.client = actionlib.SimpleActionClient('approach_action', ApproachAction)
        
    # This callback will be called when the action is complete.
    def done_callback(self, status, result):
        # The status argument tells you if the action succeeded.  Sometimes actions that did not succeed can
        # return partial results.
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Suceeded with result {0}'.format(result.success))
        else:
            rospy.loginfo('Failed with result {0}'.format(result.success))


    # This callback will be called when the action becomes active on the server.  If the server is
    # set up to only handle one action at a time, this will let you know when it's actively working
    # on your action request.
    def active_callback(self):
        rospy.loginfo('Action is active')	


    # This callback is called every time the server issues a feedback message.
    def feedback_callback(self, feedback):
        rospy.loginfo('Feedback: {0}'.format(feedback.progress))

    def main(self, stopping_distance):
        self.client.wait_for_server()
        goal = ApproachGoal(number=stopping_distance)
        self.client.send_goal(goal, done_cb=self.done_callback, active_cb=self.active_callback, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()
        

if __name__ == '__main__':
    # Specify the stopping distance for the goal
    stopping_distance_goal = 0.25

    rob_action = RobotActionClient()

    try:
        rob_action.main(stopping_distance_goal)
    except rospy.ROSInterruptException:
        pass
