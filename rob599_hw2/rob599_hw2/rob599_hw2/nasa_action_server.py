#!/usr/bin/env python3

# Action server for controlling a NASA rocket launch
#
# nasa_action_server.py
#
# Marcus Rosette   
#
# Server that stages the control over a NASA launch

# Code was adapted from the ros2 example action server (https://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rob599_hw2_msgs.action import NasaRocket

import sys
import time


class NasaActionServer(Node):
    def __init__(self):
        super().__init__('nasa_action')
        self.get_logger().info("Started nasa_action node")
        
        # Set up an action server
        self.server = ActionServer(self, NasaRocket, 'nasa', 
                                   execute_callback=self.execute_callback, 
                                   goal_callback=self.goal_callback, 
                                   cancel_callback=self.cancel_callback)
        
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()
    
    def goal_callback(self, goal_request):
        """ Accept or reject a client request to begin an action """
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """ Accept or reject a client request to cancel an action """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """ Execute a goal """
        self.get_logger().info(f"Launch Countdown Goal: {goal_handle.request.time_till_launch}sec")

        # Start the feedback message
        feedback_msg = NasaRocket.Feedback()

        for i in range(0, goal_handle.request.time_till_launch):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal Canceled...')
                return NasaRocket.Result()
            
            feedback_msg.countdown = goal_handle.request.time_till_launch - i
            self.get_logger().info(f'Countdown Feedback: {feedback_msg.countdown}')
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        result = NasaRocket.Result()
        result.result = "Launched Rocket"

        # Let the action server know that we've succeeded in the action.
        goal_handle.succeed()
        self.get_logger().info(f"Result: {result.result}")

        return result


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Make a node class
    nasa_action = NasaActionServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    # give control over to ros using the new executor
    rclpy.spin(nasa_action, executor=executor)

    # Shutdown everything cleanly
    nasa_action.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
