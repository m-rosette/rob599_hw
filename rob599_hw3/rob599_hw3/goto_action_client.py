import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rob599_hw3_msgs.action import GoTo
import sys


class GoToClient(Node):

    def __init__(self):
        # initialize the node
        super().__init__('go_to_client')
        self.action_client = ActionClient(self, GoTo, 'go_to')
        self.get_logger().info('GoToClient node has been started.')

        self.current_goal_handle = None
        self.current_goal_name = None

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

    def send_goal(self, goal_name):
        # Save the current goal name
        self.current_goal_name = goal_name

        # Save in action message
        goal_msg = GoTo.Goal()
        goal_msg.position_name = goal_name

        # send goal
        self.get_logger().info(f'Sending goal request: {goal_name}')
        self.current_goal_handle = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.current_goal_handle.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        distance_to_goal = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(f'Distance to goal "{self.current_goal_name}": {distance_to_goal}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        # if accepted, process result
        self.goal_handle = goal_handle.get_result_async()
        self.goal_handle.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal completed with outcome: {result.outcome}')

        # shutdown after result processing
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # get goal position, if available
    try:
        goal_pose = str(sys.argv[1])
    except:
        goal_pose = "None"

    # initialize client
    client = GoToClient()

    # send goal
    client.send_goal(goal_pose)

    # give control to ros
    rclpy.spin(client)

    # shutdown cleanly
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
