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
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RobotControl:
    def __init__(self):
        # Start the node
        rospy.init_node('obstacle_avoidance_node')

        # Stopping distance away from the wall
        self.dist_thresh = 1 # units = meter 

        # Default robot linear velocity
        self.lin_vel = 0.2

        # Start the subscriber
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        
        # Start the publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def publish_cmd_vel(self, linear_velocity):
        # Create a Twist message to control the robot's velocity
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity

        # Publish the Twist message
        self.cmd_vel_pub.publish(twist_msg)

    def laser_callback(self, scan_msg):
        # Extract the minimum distance from the laser scan
        min_distance = min(scan_msg.ranges)

        # Check if the minimum distance is less than the threshold
        if min_distance < self.dist_thresh:
            # Stop the robot if an obstacle is detected
            self.publish_cmd_vel(0.0)
        else:
            # Drive the robot forward if no obstacle is detected
            self.publish_cmd_vel(self.lin_vel)
            
        
if __name__ == '__main__':
    try:
        obstacle_avoidance_node = RobotControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



