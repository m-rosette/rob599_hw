#!/usr/bin/env python3

# Action client for controlling a NASA rocket launch
#
# nasa_action_client.py
#
# Marcus Rosette   
#
# Client that informs control of a NASA launch: countdown duration, cancelation parameter

# Code was adapted from the ros2 example action server (https://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py)


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rob599_hw2_msgs.action import NasaRocket


class NasaActionClient(Node):

    # def __init__(self, abort_launch):
    def __init__(self):
        # Node initialization
        super().__init__('nasa_action_client')

        # create action client under same name as server
        self.action_client = ActionClient(self, NasaRocket, 'nasa')

        # Setup a parameter for canceling the launch
        self.declare_parameter('abort_launch', False)

        # Create a timer for polling parameter changes
        self.abort_timer = self.create_timer(1, self.abort_timer_callback)

    def abort_timer_callback(self):
        """ Polls launch abort cancelation parameter """
        abort_launch = self.get_parameter('abort_launch').get_parameter_value().bool_value
        if abort_launch:
            self.send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_done(self, future):
        """ Cancels the goal and shutsdown the client """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Launch successfully canceled')
        else:
            self.get_logger().info('Launch failed to cancel')

        rclpy.shutdown()

    def goal_response_callback(self, future):
        """ Callback for when an action is accepted or rejected """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.goal_handle = goal_handle

        # proceed to callback to handle goal acceptance
        self.result_handle = goal_handle.get_result_async()
        self.result_handle.add_done_callback(self.result_callback)
        
        # create a timer to cancel the goal after a given number of seconds
        abort_launch = self.get_parameter('abort_launch').get_parameter_value().bool_value
        if abort_launch:
            self.get_logger().info('Cancel launch requested')
            self.cancel_timer = self.create_timer(1, self.timer_callback)

    def feedback_callback(self, feedback):
        """ Callback for providing countdown feedback """
        self.get_logger().info(f'Countdown: {feedback.feedback.countdown}s to launch')

    def timer_callback(self):
        """ Timer for canceling the action """
        self.get_logger().info('Canceling launch')
        # Cancel the goal
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self.cancel_timer.cancel()

    def send_goal(self, set_goal):
        """ Sends goals to the action server """
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # Set goal in action format
        goal_msg = NasaRocket.Goal()
        goal_msg.time_till_launch = set_goal

        # Send goal
        self.get_logger().info(f'Sending countdown goal request: {goal_msg.time_till_launch}')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def result_callback(self, future):
        """ Reports results after the action is completed """
        result = future.result().result
        self.get_logger().info(f"Launch Results: {result.result}")

        # shutdown after result processing
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = NasaActionClient()

    # Send goal
    action_client.send_goal(10)

    rclpy.spin(action_client)

    # secondary shutdown to catch anything that didnt get shutdown previously
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()