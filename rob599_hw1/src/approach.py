#!/usr/bin/env python3

# A fetch control node
#
# control.py
#
# Marcus Rosette
#
# A way to control the fetch robot


import rospy
import sys
import actionlib
from time import sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

from rob599_hw1.srv import StoppingDistance, StoppingDistanceResponse
from rob599_hw1.msg import ApproachAction, ApproachGoal, ApproachResult, ApproachFeedback

# Import the Marker message type from the visualization_msgs package.
from visualization_msgs.msg import Marker, MarkerArray


class RobotApproach:
    def __init__(self):
        # Start the node
        rospy.init_node('obstacle_avoidance_node')

        rospy.loginfo("Obstacle avoidance node started")

        # Stopping distance away from the wall
        self.dist_thresh = 1 # units = meter 

        # Default robot linear velocity
        self.lin_vel = 0.2

        # Action progress bool
        self.action_started = False

        # Initialize the shorted laser scan distance
        self.shortest_scan = None

        # Start the subscriber
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        
        # Start the publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Service server to set stopping distance
        self.stopping_distance_service = rospy.Service(
            'stopping_distance', StoppingDistance, self.handle_stopping_distance
        )

        # Action server to set stopping distance
        self.action_server = actionlib.SimpleActionServer(
            'approach_action', ApproachAction, self.action_callback, False
        )
        self.action_server.start()

        # Marker publisher for RViz
        self.marker_pub = rospy.Publisher('/rviz_markers', MarkerArray, queue_size=1)

        # Limit the publication rate.
        self.rate = rospy.Rate(10)

    def publish_cmd_vel(self, linear_velocity):
        # Create a Twist message to control the robot's velocity
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity

        # Publish the Twist message
        self.cmd_vel_pub.publish(twist_msg)

    def laser_callback(self, scan_msg):
        self.scan_msg = scan_msg

        # Extract the minimum distance from the laser scan
        self.shortest_scan = min(scan_msg.ranges)

        if self.shortest_scan < self.dist_thresh:
            rospy.logwarn("Laserscan reading less than threshold")

        # Check if the minimum distance is less than the threshold
        if self.shortest_scan < self.dist_thresh * 1.1:
            # Stop the robot if an obstacle is detected
            self.publish_cmd_vel(0.0)
        else:
            # Drive the robot forward if no obstacle is detected
            self.publish_cmd_vel(self.lin_vel)

        # Publish RViz markers
        self.publish_rviz_markers()
    
    def handle_stopping_distance(self, request):
        if not self.action_started:
            new_distance = request.stopping_distance

            # Check for bad values
            if new_distance <= 0:
                rospy.logerr("Stopping distance must be greater than zero.")
                return StoppingDistanceResponse(success=False)

            # Update the stopping distance
            self.dist_thresh = new_distance

            rospy.loginfo(f"Stopping distance set to {new_distance} meters.")
            return StoppingDistanceResponse(success=True)
        else:
            rospy.loginfo("Action in progress")
            return        
    
    def action_callback(self, goal):
        rospy.loginfo("Robot approach action server started")

        # Change action bool to started
        self.action_started = True

        # Check for bad values
        if goal.number <= 0:
            rospy.logerr("Stopping distance must be greater than zero.")
            return None
        else:
            self.dist_thresh = goal.number

        while self.dist_thresh < self.shortest_scan:
            self.action_server.publish_feedback(ApproachFeedback(progress=self.shortest_scan))

            if self.action_server.is_new_goal_available():
                result = False
                self.action_server.set_preempted(ApproachResult(success=result))
                return
            
            # Artificially wait for a bit.
            sleep(1)

        result = True
        self.action_server.set_succeeded(ApproachResult(success=result))
        self.action_in_progress = False
        rospy.loginfo("Robot approach action completed")

    def publish_rviz_markers(self):
        # Create a MarkerArray message
        marker_array = MarkerArray()

        # Marker for the line between the robot and the shortest contact point
        line_marker = Marker()
        line_marker.header.frame_id = "base_link"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.id = 0
        line_marker.color.r = 1.0
        line_marker.color.a = 1.0
        line_marker.scale.x = 0.05  # Line width

        # Robot position
        robot_position = Point()
        robot_position.x = 0.0
        robot_position.y = 0.0
        robot_position.z = 0.0

        # Shortest contact point
        contact_point = Point()
        contact_point.x = self.shortest_scan
        contact_point.y = 0.0
        contact_point.z = 0.0

        # Set line marker points
        line_marker.points.append(robot_position)
        line_marker.points.append(contact_point)

        marker_array.markers.append(line_marker)

        # Marker for text displaying the shortest distance
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.id = 1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.scale.z = 0.25  # Text size

        # Set text marker position and content
        # Calculate the midpoint between robot and contact point
        midpoint = Point()
        midpoint.x = contact_point.x
        midpoint.y = contact_point.y
        midpoint.z = contact_point.z
        text_marker.text = f"Shortest distance: {self.shortest_scan:.2f} meters"

        marker_array.markers.append(text_marker)

        # Marker for the contact point
        contact_marker = Marker()
        contact_marker.header.frame_id = "base_link"
        contact_marker.type = Marker.SPHERE
        contact_marker.action = Marker.ADD
        contact_marker.id = 2
        contact_marker.color.r = 0.0
        contact_marker.color.g = 1.0
        contact_marker.color.b = 0.0
        contact_marker.color.a = 1.0
        contact_marker.scale.x = 0.1  # Sphere diameter
        contact_marker.scale.y = 0.1
        contact_marker.scale.z = 0.1

        # Set contact marker position
        contact_marker.pose.position = contact_point

        marker_array.markers.append(contact_marker)

        # Publish the MarkerArray message
        self.marker_pub.publish(marker_array)

        
if __name__ == '__main__':
    try:
        obst_avoid = RobotApproach()      
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



