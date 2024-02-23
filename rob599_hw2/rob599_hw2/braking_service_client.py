#!/usr/bin/env python3

# Node that limits the linear and rotational velocity of a robot
#
# braking_service_client.py
#
# Marcus Rosette   
#
# This is a client that calls the service to apply braking

import rclpy
from rclpy.node import Node
from rob599_hw2_msgs.srv import ApplyBrakes

import sys


class BrakingServiceClient(Node):
    def __init__(self):
        super().__init__('braking_client')
        
        # Set up a service client
        self.client = self.create_client(ApplyBrakes, 'apply_brakes')

        # Wait until we have a connection to the server
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to start')

    def send_request(self, braking_status):
        """ Sends the request for braking """
        request = ApplyBrakes.Request()
        request.status = braking_status

        # Call the service asynchronously
        self.response = self.client.call_async(request)


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Make a node class
    braking_client = BrakingServiceClient()
    
    # Convert command-line argument to a boolean value
    braking_status = bool(int(sys.argv[1]))

    # Log the status of the user input
    braking_client.get_logger().info(f"Requested braking status: {braking_status}")

    # Send the service request
    braking_client.send_request(braking_status)

    # Shutdown everything cleanly
    braking_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
