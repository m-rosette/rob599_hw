#!/usr/bin/env python3

# A laserscan filter node
#
# filter_scan.py
#
# Marcus Rosette
#
# A way to filter the laser scan to a set threshold in front of the robot


import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


class FilteredScan:
    def __init__(self):
        # Start the node
        rospy.init_node('filtered_laser_scan')

        rospy.loginfo("Filter scan node started")

        # Robot width
        self.robot_width = 1 # units = meter 

        # Horizonal distance of desired scan (halved)
        self.dist_thresh = 0.5 # units = meter

        # Start the subscriber
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Start the publisher
        self.filtered_scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)

        # Limit the publication rate.
        self.rate = rospy.Rate(10)

    def laser_callback(self, scan_msg):
        # Convert laser ranges to a NumPy array
        ranges = np.array(scan_msg.ranges)

        # Arrange the angles in a NumPy array
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

        # Calculate the x-distance of each laser reading using trigonometry
        x_dist = ranges * np.sin(angles)

        # Create a boolean mask for distances exceeding a threshold and set them to Inf
        ranges[np.abs(x_dist) > self.dist_thresh] = np.inf

        # Update the scan message with the trimmed version
        scan_msg.ranges = ranges

        # Publish the filtered scan message to a ROS topic
        self.filtered_scan_pub.publish(scan_msg)
            
        
if __name__ == '__main__':
    try:
        filtered_scan = FilteredScan()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



