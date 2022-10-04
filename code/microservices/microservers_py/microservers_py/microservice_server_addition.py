"""
Implement MicroService Addition.

Implement AdditionMicroService class and the ROS 2 command:
    microservices_py server_addition
"""
import random
import time

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class AdditionMicroService(Node):
    """
    MicroServer that adds two integers.

    This server will run forever and will response to every request
    by adding the two integers in the request and senging the result
    in the response.
    """

    def __init__(
            self):
        """Construct default Addition Server."""
        super().__init__('AdditionMicroServer')
        self.srv = self.create_service(
            AddTwoInts,
            'addition_service',
            self.addition_service_callback)

    def addition_service_callback(
            self,
            request: AddTwoInts.Request,
            response: AddTwoInts.Response):
        """Response to the request received."""
        # ...
        # here can be executed any code, as complex as needed
        # to execute it in Cloud and not in Edge.
        # ...

        # Calculate this microservice result
        response.sum = request.a + request.b

        # Wait for a random time to simulate a long computation
        time.sleep(random.randint(1, 3))

        # Log server result
        self.get_logger().info(
            f'Incoming request {{ {request.a} + {request.b} = '
            f'{response.sum} }}')

        # return response
        return response


def main():
    """Create and execute AdditionMicroService. No arguments are needed."""
    # Initialize ROS2 RCL py
    rclpy.init()

    # Create Server Node
    addition_server_node = AdditionMicroService()

    # Run server
    rclpy.spin(addition_server_node)

    # Destroy Node and stop ROS2 RCL
    addition_server_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
