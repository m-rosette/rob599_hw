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
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rob599_hw1.srv import StoppingDistance, StoppingDistanceResponse
from rob599_hw1.msg import ApproachAction, ApproachGoal, ApproachResult, ApproachFeedback


class RobotApproach:
    def __init__(self):
        # Start the node
        rospy.init_node('obstacle_avoidance_node')

        rospy.loginfo("Obstacle avoidance node started")

        # Stopping distance away from the wall
        self.dist_thresh = 1 # units = meter 

        # Default robot linear velocity
        self.lin_vel = 0.2

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
            'approach_action', ApproachAction, self.execute_approach, False
        )
        self.action_server.start()

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
        min_distance = min(scan_msg.ranges)

        if min_distance < self.dist_thresh:
            rospy.logwarn("Laserscan reading less than threshold")

        # Check if the minimum distance is less than the threshold
        if min_distance < self.dist_thresh * 1.1:
            # Stop the robot if an obstacle is detected
            self.publish_cmd_vel(0.0)
        else:
            # Drive the robot forward if no obstacle is detected
            self.publish_cmd_vel(self.lin_vel)
    
    def handle_stopping_distance(self, request):
        new_distance = request.stopping_distance

        # Check for bad values
        if new_distance <= 0:
            rospy.logerr("Stopping distance must be greater than zero.")
            return StoppingDistanceResponse(success=False)

        # Update the stopping distance
        self.dist_thresh = new_distance

        rospy.loginfo(f"Stopping distance set to {new_distance} meters.")
        return StoppingDistanceResponse(success=True)
    
    # def action_callback(self, goal):
        
    
    def execute_approach(self, goal):
        # Define feedback and result objects
        feedback = ApproachFeedback()
        result = ApproachResult()

        # Loop until the robot reaches the goal or preempted
        while not rospy.is_shutdown():
            # Extract the minimum distance from the laser scan
            min_distance = min(self.scan_msg.ranges)

            # Provide feedback to the action client
            feedback.feedback_distance = min_distance
            self.action_server.publish_feedback(feedback)

            # Check if the minimum distance is less than the goal
            if min_distance < goal.stopping_distance:
                # Stop the robot if the goal is reached
                self.publish_cmd_vel(0.0)
                result.success = True
                self.action_server.set_succeeded(result, "Goal reached.")
                return

            # Drive the robot forward if the goal is not reached
            self.publish_cmd_vel(self.lin_vel)
            self.rate.sleep()
            
        
if __name__ == '__main__':
    try:
        obst_avoid = RobotApproach()      
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



