#!/usr/bin/env python3

# Node that limits the linear and rotational velocity of a robot
#
# velocity_limiter.py
#
# Marcus Rosette   
#
# This is a node that subscribes to, limits, and publishes velocities to a Twist message

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rob599_hw2_msgs.srv import ApplyBrakes

import numpy as np


class VelocityLimiter(Node):
	def __init__(self):
		super().__init__('velocity_limiter')
		self.get_logger().info("Started velocity_limiter node")

		self.declare_parameter("linear_max", 4.0)
		self.declare_parameter("angular_max", 2.0)
		self.declare_parameter("with_watchdog", True)
		self.declare_parameter("watchdog_period", 1.0)

		# Create publisher and subscriber
		self.vel_pub = self.create_publisher(Twist, 'speed_out', 10)
		self.vel_sub = self.create_subscription(Twist, 'speed_in', self.velocity_callback, 10)

		# Create a watchdog timer
		watchdog_period = self.get_parameter('watchdog_period').get_parameter_value().double_value
		self.watchdog_timer = self.create_timer(watchdog_period, self.timer_callback)
		self.last_twist_time = self.get_clock().now().nanoseconds

		# Create a service and its timer
		self.braking_srv = self.create_service(ApplyBrakes, 'apply_brakes', self.braking_service_callback)
		self.braking_timer = self.create_timer(0.1, self.braking_timer_callback)
		self.brake_check = False

	def velocity_callback(self, msg):
		""" Caps the Twist messages if braking is disabled """
		# Get the currect time of recieving a Twist message
		self.last_twist_time = self.get_clock().now().nanoseconds

		# If not braking, cap the Twist values to their set max parameters
		if not self.brake_check:
			msg = self.limit_velocity(msg)
			self.vel_pub.publish(msg)

	def timer_callback(self):
		""" Watchdog timer to catch any missed Twist messages during set period of time """
		with_watchdog = self.get_parameter('with_watchdog').get_parameter_value().bool_value
		watchdog_period = self.get_parameter('watchdog_period').get_parameter_value().double_value

		if with_watchdog:
			if self.get_clock().now().nanoseconds - self.last_twist_time > watchdog_period * 1e9:
				self.get_logger().info(f"Watchdog Triggered: No Twist recieved within {watchdog_period}s. Setting a zero Twist")
				zero_twist = Twist()
				self.vel_pub.publish(zero_twist)

	def limit_velocity(self, msg):
		""" Limit the Twist values to the set parameters for linear and angluar velocity """
		linear_max = self.get_parameter('linear_max').get_parameter_value().double_value
		angular_max = self.get_parameter('angular_max').get_parameter_value().double_value

		# Clip linear velocities
		msg.linear.x = np.clip(msg.linear.x, -np.abs(linear_max), np.abs(linear_max))
		msg.linear.y = np.clip(msg.linear.y, -np.abs(linear_max), np.abs(linear_max))
		msg.linear.z = np.clip(msg.linear.z, -np.abs(linear_max), np.abs(linear_max))

		# Clip angular velocities
		msg.angular.x = np.clip(msg.angular.x, -np.abs(angular_max), np.abs(angular_max))
		msg.angular.y = np.clip(msg.angular.y, -np.abs(angular_max), np.abs(angular_max))
		msg.angular.z = np.clip(msg.angular.z, -np.abs(angular_max), np.abs(angular_max))
		return msg
	
	def braking_service_callback(self, request, response):
		""" Controls the service by a stored bool for whether braking is happening or not """
		if request.status:
			self.get_logger().info("Applying Brakes")
			self.brake_check = True
		if not request.status:
			self.get_logger().info("Not Applying Brakes")
			self.brake_check = False
		
		response.result = True
		return response
			
	def braking_timer_callback(self):
		""" Sends a zero Twist message if braking is set to True """
		if self.brake_check:
			self.get_logger().info("Braking - Publishing a Zero Twist")
			zero_twist = Twist()
			self.vel_pub.publish(zero_twist)


def main(args=None):
	# Initialize rclpy
	rclpy.init(args=args)

	# Make a node class
	velocity_limiter = VelocityLimiter()

	# give control over to ROS2
	rclpy.spin(velocity_limiter)

	# shutdown everything cleanly
	rclpy.shutdown()
	

if __name__ == '__main__':
	main()
