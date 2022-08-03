"""
Implement Client for Addition MicroService.

Implement MicroserviceClientAddition class and the ROS 2 command:
    microservices_py client_addition
"""
import random

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MicroServiceClientAddition(Node):
    """
    Client of addition microservice.

    This Node creates a clients that connects to an AdditionMicroService.
    It supports a method to send a request to the server and received result.
    """

    def __init__(
            self):
        """
        Construct Client.

        This create the service 'addition_service' and
        register the callback function 'addition_service_callback'
        to process with every request.
        """
        # Call Node constructor
        super().__init__('AdditionMicroClient')

        # Create both clients
        self.addition_client = self.create_client(
            AddTwoInts, 'addition_service')

    def wait_to_find_server(
            self,
            timeout_seconds: int = 5):
        """
        Wait until server is available or timeout.

        Parameters
        ----------
        timeout_seconds : int
            Number of seconds to wait before timeout.

        Returns
        -------
        bool
            True if server available, False if timeout.
        """
        while not self.addition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

            # Check if timeout has expired
            timeout_seconds -= 1
            if timeout_seconds <= 0:
                return False

        return True

    def send_request(
            self,
            a: int,
            b: int) -> int:
        """
        Send request to server.

        Parameters
        ----------
        a : int
            First number to operate with
        b : int
            Second number to operate with

        Returns
        -------
        int
            Solution of the operation
        """
        # Set request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Send request
        self.future = self.addition_client.call_async(request)

        self.get_logger().info(
            f'Request sent, waiting for server result.')

        # Wait for response
        rclpy.spin_until_future_complete(self, self.future)

        # Return result
        return self.future.result().sum


def main():
    """
    Create and execute AdditionService.

    This is the main function that would be called
    by using command: ros2 run
        microservices_py.server_addition

    No arguments are needed.
    """
    # Initialize ROS2 RCL py
    rclpy.init()

    # Create Client Node
    minimal_client = MicroServiceClientAddition()

    # Wait until server is available
    if minimal_client.wait_to_find_server():
        # Server already available
        # Get random integers
        a = random.randint(0, 100)
        b = random.randint(0, 100)

        # Send request
        result = minimal_client.send_request(a, b)
        minimal_client.get_logger().info(
            f'Result of operation: for {a} + {b} = {result}')

    else:
        minimal_client.get_logger().warning(
            'Service not available, aborting.')

    # Destroy Node and stop ROS2 RCL
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
