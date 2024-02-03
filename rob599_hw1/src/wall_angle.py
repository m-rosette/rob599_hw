#!/usr/bin/env python3

# A wall angle estimator from the frame of the robot
#
# wall_angle.py
#
# Marcus Rosette
#
# A way to estimate the angle of the wall in front of the robot from its frame of reference
# This node subscribes to the filtered_scan messages from filter_scan.py

import rospy
import sys
import numpy as np
from numpy.polynomial.polynomial import polyfit 
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64


class WallAngleCalculator:
    def __init__(self) -> None:
        rospy.init_node("wall_angle_node")

        rospy.loginfo("Wall angle node started")

        rospy.Subscriber("filtered_scan", LaserScan, self.callback, queue_size=None)

        self.marker_pub = rospy.Publisher("wall_angle_marker", Marker, queue_size=10)
        self.wall_angle_pub = rospy.Publisher("angle_to_wall", Float64, queue_size=10)

    # Creates text marker that says current angle to wall
    def set_text_marker(self, angle):
        angle_text = Marker() 
        angle_text.header.frame_id = "base_link"
        angle_text.type = angle_text.TEXT_VIEW_FACING
        angle_text.id = 2 
        angle_text.action = angle_text.ADD
        angle_text.color.r = 1.0
        angle_text.color.g = 1.0
        angle_text.color.b = 1.0
        angle_text.color.a = 1.0
        angle_text.scale.z = 0.25  # Text size
        angle_text.pose.position.x = -0.8 
        angle_text.pose.position.y = 0.0
        angle_text.pose.position.z = 0.0
        angle_text.text = str(f"Wall angle: {round(angle, 4)} rads")
        return angle_text

    def callback(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        non_inf_indices = np.where(np.array(msg.ranges) != float("inf"))[0]

        x = np.array(msg.ranges)[non_inf_indices] * np.cos(angles[non_inf_indices])
        y = np.array(msg.ranges)[non_inf_indices] * np.sin(angles[non_inf_indices])

        slope, intercept = polyfit(x, y, 1)

        wall_angle = np.arctan(slope)

        # Adjust angle and publish the text marker and angle
        adjusted_angle = wall_angle - 1.5708 if wall_angle > 0 else wall_angle + 1.5708
        self.marker_pub.publish(self.set_text_marker(adjusted_angle))
        self.wall_angle_pub.publish(adjusted_angle)


if __name__ == '__main__':
    try:
        wall_angle_calculator = WallAngleCalculator()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass