import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
import tf2_ros
import yaml
import os
import time
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rob599_hw3_msgs.srv import MemorizePosition, ClearPositions, Save, Load, KnockKnock
from rob599_hw3_msgs.action import GoTo, Patrol


class Places(Node):
    def __init__(self):
        super().__init__('places')
        self.get_logger().info("Started places node")

        # Initialize position dict
        self.positions = {}

        # Publishers and markers initialization
        self.marker_pub = self.create_publisher(Marker, 'saved_positions', 10)
        self.marker_id = 0

        # Services
        self.memorize_position_srv = self.create_service(
            MemorizePosition, 'memorize_position', self.memorize_position_callback)
        
        self.clear_positions_srv = self.create_service(
            ClearPositions, 'clear_positions', self.clear_positions_callback)
        
        self.save_places_srv = self.create_service(
            Save, 'save_places', self.save_places_callback)

        self.load_places_srv = self.create_service(
            Load, 'load_places', self.load_places_callback)
        
        self.knock_knock_srv = self.create_service(
            KnockKnock, 'knock_knock', self.knock_knock_callback)
        
        # Actions server
        self.go_to_action_server = ActionServer(
            self,
            GoTo,
            'go_to',
            execute_callback=self.go_to_callback)
        
        self.patrol_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.patrol_callback)
        
        # Setup storage directory
        self.storage_dir = os.path.join(
            get_package_share_directory('rob599_hw3'),
            'resource')

        # Initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()

    def memorize_position_callback(self, request, response):
        position_name = request.position_name
        current_pose = self.get_current_robot_pose()  # Get current robot pose
        if current_pose is not None:
            self.positions[position_name] = current_pose
            self.publish_marker(position_name, current_pose)
            response.success = True
            response.message = f"Position '{position_name}' memorized."
        else:
            response.success = False
            response.message = "Failed to memorize position. Unable to get current robot pose."
        return response
    
    def publish_marker(self, position_name, pose_stamped):
        # Publish arrow marker
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "map"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.id = self.marker_id
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose = pose_stamped.pose
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 1.0
        arrow_marker.color.a = 1.0
        self.marker_pub.publish(arrow_marker)

        # Publish text marker
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.id = self.marker_id + 1  # Make sure the text marker ID is different from the arrow marker ID
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = pose_stamped.pose.position
        text_marker.pose.position.z += 0.5  # Adjust the height of the text above the arrow
        text_marker.scale.z = 0.5  # Increase the size of the text
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = position_name
        self.marker_pub.publish(text_marker)

        self.marker_id += 2  # Increment marker ID by 2 to account for both arrow and text markers

        time.sleep(0.05)  # Allow rviz to process the current publish before introducing another

    def clear_positions_callback(self, request, response):
        self.positions = {}
        self.clear_markers()
        self.marker_id = 0  # Reset marker ID to zero
        response.success = True
        response.message = "All positions cleared."
        return response

    def clear_markers(self):
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

    def get_current_robot_pose(self):
        try:
            # Lookup the transform from the base_link to the map frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            
            # Create a PoseStamped message with the transform
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = transform.header.stamp
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation

            return pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to get robot pose from TF2")
            return None
        
    def save_places_callback(self, request, response):
        try:
            with open(os.path.join(self.storage_dir, request.filename + '.yaml'), 'w') as file:
                yaml_data = {}
                for position_name, pose_stamped in self.positions.items():
                    yaml_data[position_name] = {
                        'header': {
                            'stamp': {
                                'sec': pose_stamped.header.stamp.sec,
                                'nanosec': pose_stamped.header.stamp.nanosec
                            },
                            'frame_id': pose_stamped.header.frame_id
                        },
                        'pose': {
                            'position': {
                                'x': pose_stamped.pose.position.x,
                                'y': pose_stamped.pose.position.y,
                                'z': pose_stamped.pose.position.z
                            },
                            'orientation': {
                                'x': pose_stamped.pose.orientation.x,
                                'y': pose_stamped.pose.orientation.y,
                                'z': pose_stamped.pose.orientation.z,
                                'w': pose_stamped.pose.orientation.w
                            }
                        }
                    }
                yaml.dump(yaml_data, file)
            response.success = True
            response.message = f"Places saved to '{request.filename}.yaml'"
        except Exception as e:
            response.success = False
            response.message = f"Failed to save places: {str(e)}"
        return response

    def load_places_callback(self, request, response):
        try:
            with open(os.path.join(self.storage_dir, request.filename + '.yaml'), 'r') as file:
                yaml_data = yaml.safe_load(file)
                for position_name, data in yaml_data.items():
                    # Extract pose data from yaml
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp.sec = data['header']['stamp']['sec']
                    pose_stamped.header.stamp.nanosec = data['header']['stamp']['nanosec']
                    pose_stamped.header.frame_id = data['header']['frame_id']
                    pose_stamped.pose.position.x = data['pose']['position']['x']
                    pose_stamped.pose.position.y = data['pose']['position']['y']
                    pose_stamped.pose.position.z = data['pose']['position']['z']
                    pose_stamped.pose.orientation.x = data['pose']['orientation']['x']
                    pose_stamped.pose.orientation.y = data['pose']['orientation']['y']
                    pose_stamped.pose.orientation.z = data['pose']['orientation']['z']
                    pose_stamped.pose.orientation.w = data['pose']['orientation']['w']

                    # Save loaded positions
                    self.positions[position_name] = pose_stamped
                    
                    # Publish marker for loaded position
                    self.publish_marker(position_name, pose_stamped)

            response.success = True
            response.message = f"Places loaded from '{request.filename}.yaml'"
        except FileNotFoundError:
            response.success = False
            response.message = f"File '{request.filename}.yaml' not found"
        except yaml.YAMLError as e:
            response.success = False
            response.message = f"Failed to load places: {str(e)}"
        except Exception as e:
            response.success = False
            response.message = f"An error occurred: {str(e)}"
        return response
    
    def go_to_callback(self, goal_handle):
        self.get_logger().info(f"Goal recieved with name: {goal_handle.request.position_name}")

        goal_name = goal_handle.request.position_name

        if goal_name not in self.positions:
            self.get_logger().warn(f"Goal position '{goal_name}' not found")
            goal_handle.abort()
            return GoTo.Result(outcome=f"Position '{goal_name}' not found.")

        goal_pose = self.positions[goal_name]
        current_pose = self.get_current_robot_pose()
        if current_pose is None:
            self.get_logger().error("Failed to get current robot pose")
            goal_handle.abort()
            return GoTo.Result(outcome="Failed to get current robot pose.")

        # control robot to goal using BasicNavigator
        self.get_logger().info(f"Executing goal to '{goal_name}'")
        self.navigator.goToPose(goal_pose)

        # publish feedback until we're done
        while rclpy.ok() and not self.navigator.isTaskComplete():
            current_feedback = self.navigator.getFeedback()
            feedback_msg = GoTo.Feedback(distance_to_goal=float(current_feedback.distance_remaining))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  # Adjust sleep duration as needed

        # process
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            result.outcome = 'Goal succeeded!'
        elif result == TaskResult.CANCELED:
            result.outcome = 'Goal was canceled!'
        elif result == TaskResult.FAILED:
            result.outcome = 'Goal failed!'

        # Set goal outcome
        goal_handle.succeed()
        self.get_logger().info("Goal completed successfully")

        # Return result
        return GoTo.Result(outcome="Reached destination.")
    
    def patrol_callback(self, goal_handle):
        self.get_logger().info("Received patrol request")
        feedback_msg = Patrol.Feedback()
        result_msg = Patrol.Result()

        for position_name, pose_stamped in self.positions.items():
            if goal_handle.is_cancel_requested:
                result_msg.outcome = 'Patrol cancelled'
                goal_handle.canceled()
                return result_msg

            self.get_logger().info(f"Patrolling to {position_name}")
            feedback_msg.current_position = position_name

            # Execute the goal (replace this with your actual execution logic)
            self.navigator.goToPose(pose_stamped)

            # Loop until goal is reached or cancelled
            while not self.navigator.isTaskComplete():
                if goal_handle.is_cancel_requested:
                    result_msg.outcome = 'Patrol cancelled'
                    goal_handle.canceled()
                    return result_msg
                
                # Get current position from robot for feedback
                current_feedback = self.navigator.getFeedback()
                feedback_msg.distance_to_goal = float(current_feedback.distance_remaining)

                # Publish feedback
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)  # Sleep for 1 second before checking again

            # Publish next position
            next_position_name = "Next position"
            feedback_msg.current_position = next_position_name
            goal_handle.publish_feedback(feedback_msg)

        result_msg.outcome = 'Patrol completed'
        goal_handle.succeed()
        return result_msg
    
    def knock_knock_callback(self, request, response):
        # Preempt any ongoing action
        self.navigator.cancelTask()

        # Navigate to the front door
        front_door_pose = self.positions.get('front_door')
        if front_door_pose:
            self.navigator.goToPose(front_door_pose)
            while not self.navigator.isTaskComplete():
                response.message = "Checking front door"

            # Check if navigation succeeded
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                response.success = True
                response.message = "Arrived at the front door."
            else:
                response.success = False
                response.message = "Failed to reach the front door."
        else:
            response.success = False
            response.message = "Front door position not found."

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Places()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()