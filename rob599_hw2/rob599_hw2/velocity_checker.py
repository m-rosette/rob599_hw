#!/usr/bin/env python3

# velocity_check.py
#
# Marcus Rosette   
#
# This is a node that subscribes to Twist messages and checks how many values are within a limit

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np


class VelocityChecker(Node):
	def __init__(self):
		super().__init__('velocity_checker')
		self.get_logger().info("Started velocity_checker node")

		# Create subscriber
		self.vel_sub = self.create_subscription(Twist, 'speed_in', self.velocity_callback, 10)

		# Create a timer
		self.timer = self.create_timer(30, self.timer_callback)
		
        # Set parameters
		self.declare_parameter('linear_max_check', 4.0)
		self.declare_parameter('angular_max_check', 2.0)
		
		self.exceeded_count = 0
		self.msg_count = 0

	def velocity_callback(self, msg):
		linear_max_check = self.get_parameter('linear_max_check').get_parameter_value().double_value
		angular_max_check = self.get_parameter('angular_max_check').get_parameter_value().double_value
        
		self.msg_count += 1
		
		linear = msg.linear
		angular = msg.angular
		
		# Extract individual components from Vector3 for linear velocity
		linear_x = linear.x
		linear_y = linear.y
		linear_z = linear.z
		
		# Extract individual components from Vector3 for angular velocity
		angular_x = angular.x
		angular_y = angular.y
		angular_z = angular.z
		
		if np.abs(linear_x) > linear_max_check or np.abs(linear_y) > linear_max_check or np.abs(linear_z) > linear_max_check \
			or np.abs(angular_x) > angular_max_check or np.abs(angular_y) > angular_max_check or np.abs(angular_z) > angular_max_check:
			self.exceeded_count += 1
        	

	def timer_callback(self):
		# Log the counts
		self.get_logger().info(f"Recieved {self.msg_count} total messages and {self.exceeded_count} were over the limit")
		
        # Reset the counts
		self.exceeded_count = 0
		self.msg_count = 0


def main(args=None):
	# Initialize rclpy
	rclpy.init(args=args)

	# Make a node class
	velocity_checker = VelocityChecker()

	# give control over to ROS2
	rclpy.spin(velocity_checker)

	# shutdown everything cleanly
	rclpy.shutdown()
	

if __name__ == '__main__':
	main()
