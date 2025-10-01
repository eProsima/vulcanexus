"""
This file contains a ROS 2 SharedNode and the main entrypoint to test the turtlesim tools.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from vulcanai.console import VulcanConsole


class SharedNode(Node):
    def __init__(self, name: str = "vulcanai_shared_node"):
        super().__init__(name)
        # Dictionary to store created clients
        self._vulcan_clients = {}
        # Dictionary to store created publishers
        self._vulcan_publishers = {}

        # Ensure entities creation is thread-safe.
        self.node_lock = threading.Lock()

    def get_client(self, srv_type, srv_name):
        """
        Get a cached client for the specified service type and name or
        create a new one if it doesn't exist.
        """
        key = (srv_type, srv_name)
        with self.node_lock:
            if key not in self._vulcan_clients:
                client = self.create_client(srv_type, srv_name)
                self._vulcan_clients[key] = client
                self.get_logger().info(f"Created new client for {srv_name}")
            return self._vulcan_clients[key]

    def get_publisher(self, msg_type, topic_name):
        """
        Get a cached publisher for the specified message type and topic name or
        create a new one if it doesn't exist.
        """
        key = (msg_type, topic_name)
        with self.node_lock:
            if key not in self._vulcan_publishers:
                publisher = self.create_publisher(msg_type, topic_name, 10)
                self._vulcan_publishers[key] = publisher
                self.get_logger().info(f"Created new publisher for {topic_name}")
            return self._vulcan_publishers[key]

    def wait_for_message(self, msg_type, topic: str, timeout: float = None):
        """
        Block until a message is received or timeout expires.
        Subscriptions are created on demand and destroyed after use to avoid
        handling spins and callbacks in a separate thread.
        """
        future = Future()

        def callback(msg):
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(msg_type, topic, callback, 10)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        self.destroy_subscription(sub)

        if future.done():
            return future.result()
        return None


def main(args=None):
    # Create a ROS 2 node that will be used by the tools to avoid
    # recurrent creation and destruction of DDS Participants
    rclpy.init(args=args)
    node = SharedNode(name="vulcanai_shared_node")

    user_context = """\
You are controlling the turtlesim simulation from ROS 2.
The simulation has one or more turtles that can move around, drawing on the screen as they go."""

    console = VulcanConsole()
    console.manager.register_tools_from_entry_points("turtle_tools")
    console.manager.add_user_context(user_context)
    # Add the shared node to the console manager blackboard to be used by tools
    console.manager.bb["main_node"] = node
    console.run()


if __name__ == '__main__':
    main()

