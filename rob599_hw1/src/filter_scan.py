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

import numpy as np
import math
from sensor_msgs.msg import LaserScan


class FilteredScan:
    def __init__(self):
        # Start the node
        rospy.init_node('filtered_laser_scan')

        rospy.loginfo("Filter scan node started")

        # Robot width
        self.robot_width = 1 # units = meter 

        # Scan angle
        self.front_scan_angle = 30.0

        # Start the subscriber
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Start the publisher
        self.filtered_scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)

        # Limit the publication rate.
        self.rate = rospy.Rate(10)

    def laser_callback(self, scan_msg):
        # Extracting relevant information
        ranges = scan_msg.ranges
        angle_increment = scan_msg.angle_increment
        num_scans = len(ranges)

        # Calculating the index range for the front scans
        mid_scan = math.ceil(num_scans / 2)
        boundary_scan = 10
        front_scans = ranges[mid_scan-boundary_scan:mid_scan+boundary_scan]
        # instead of truncating the list, make sure its the same size as the original -> making the stuff outside the bounds -inf or +inf
        
        # half_width_scans = int(self.robot_width / (2 * scan_msg.angle_increment))
        # start_index = int((num_scans / 2) - half_width_scans)
        # end_index = int((num_scans / 2) + half_width_scans)

        # # Extracting front scans
        # # front_scans = ranges[start_index:end_index]
        # front_scans = ranges[0:10]

        # Creating a new LaserScan message with only front scans
        filtered_scan_msg = LaserScan()
        filtered_scan_msg.header = scan_msg.header
        filtered_scan_msg.angle_min = scan_msg.angle_min
        filtered_scan_msg.angle_max = scan_msg.angle_max
        filtered_scan_msg.angle_increment = angle_increment
        filtered_scan_msg.range_min = scan_msg.range_min
        filtered_scan_msg.range_max = scan_msg.range_max
        filtered_scan_msg.ranges = front_scans

        # Publish the filtered scan message
        self.filtered_scan_pub.publish(filtered_scan_msg)
            
        
if __name__ == '__main__':
    try:
        filtered_scan = FilteredScan()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



