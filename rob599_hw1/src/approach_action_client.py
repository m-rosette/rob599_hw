#!/usr/bin/env python3

import rospy
import actionlib
from rob599_hw1.msg import ApproachAction, ApproachGoal

def send_approach_goal(stopping_distance):
    # Initialize the ROS node
    rospy.init_node('send_approach_goal')

    # Create an action client
    client = actionlib.SimpleActionClient('approach_action', ApproachAction)

    # Wait for the action server to start
    client.wait_for_server()

    # Create a goal
    goal = ApproachGoal(stopping_distance=stopping_distance)

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Print the result
    print(f"Result: {client.get_result()}")

if __name__ == '__main__':
    try:
        # Specify the stopping distance for the goal
        stopping_distance_goal = 0.5

        # Send the approach goal
        send_approach_goal(stopping_distance_goal)
    except rospy.ROSInterruptException:
        pass
