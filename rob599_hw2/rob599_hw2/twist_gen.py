#!/usr/bin/env python3

# twist_gen.py
#
# Marcus Rosette
#
# This is a node that publishes Twist messages

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np


class TwistPublisher(Node):
	def __init__(self):
		super().__init__('twist_publisher')
		self.get_logger().info("Started twist_publisher")

		# Create publisher
		self.twist_pub = self.create_publisher(Twist, 'speed_in', 10)

		# Create timer
		self.timer = self.create_timer(1, self.timer_callback)
	
	def generate_rand_twist(self):
		""" Generates random Twist messages """
		msg = Twist()
		
		# Random linear velocities
		msg.linear.x, msg.linear.y, msg.linear.z = np.round(np.random.uniform(-5.0, 5.0, size=3), 1)
		
		# Random angular velocities
		msg.angular.x, msg.angular.y, msg.angular.z = np.round(np.random.uniform(-2.5, 2.5, size=3), 1)
		return msg

	# This callback will be called every time the timer fires.
	def timer_callback(self):
		""" Publishes the random twists on a timer """
		msg = self.generate_rand_twist()

		self.twist_pub.publish(msg)


def main(args=None):
	# Initialize rclpy
	rclpy.init(args=args)

	# Make a node class
	twist_publisher = TwistPublisher()

	# give control over to ROS2
	rclpy.spin(twist_publisher)

	# shutdown everything cleanly
	rclpy.shutdown()


if __name__ == '__main__':
	main()