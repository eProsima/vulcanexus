"""
This file contains example tools to interact with ROS 2 turtlesim and
demonstrate how to create custom tools compatible with VulcanAI.
"""

from math import cos, sin, pi
import time

import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim_msgs.srv import Kill, Spawn, TeleportAbsolute, TeleportRelative
from turtlesim_msgs.msg import Pose

from vulcanai.tool_registry import vulcanai_tool
from vulcanai.tools import AtomicTool, CompositeTool


@vulcanai_tool
class SpawnTurtleTool(AtomicTool):
    name = "spawn_turtle"
    description = "Spawn a new turtle in turtlesim. 'spawned' indicates success."
    tags = ["turtlesim", "spawn", "add", "create", "service"]
    input_schema = [
        ("x", "float"),
        ("y", "float"),
        ("theta", "float"),
        ("name", "string"),
    ]
    output_schema = {"name": "string", "spawned": "bool"}

    def run(self, **kwargs):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        client = node.get_client(Spawn, "spawn")
        if not client.wait_for_service(timeout_sec=5.0):
            raise Exception("Service not available, aborting...")

        req = Spawn.Request()
        req.x = float(kwargs.get("x", 5.54))
        req.y = float(kwargs.get("y", 5.54))
        req.theta = float(kwargs.get("theta", 0.0))
        req.name = kwargs.get("name", "")

        with node.node_lock:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            if result is None:
                raise Exception("Service call failed timeout, aborting...")

        return {"name": result.name, "spawned": True}


@vulcanai_tool
class KillTurtleTool(AtomicTool):
    name = "kill_turtle"
    description = "Kill a turtle 'name' in turtlesim. 'killed' indicates success."
    tags = ["turtlesim", "kill", "remove", "delete", "service"]
    input_schema = [
        ("name", "string"),
    ]
    output_schema = {"name": "string", "killed": "bool"}

    def run(self, name: str = ""):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        if not name:
            print("No turtle name provided, aborting...")
            return {"name": "", "killed": False}

        client = node.get_client(Kill, "kill")
        if not client.wait_for_service(timeout_sec=5.0):
            raise Exception("Service not available, aborting...")

        req = Kill.Request()
        req.name = name
        with node.node_lock:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            if result is None:
                raise Exception("Service call failed timeout, aborting...")

        return {"name": req.name, "killed": True}


@vulcanai_tool
class ResetTurtleSimTool(AtomicTool):
    name = "reset_turtlesim"
    description = "Reset the turtlesim environment. 'reset' indicates success."
    tags = ["turtlesim", "reset", "environment", "service"]
    input_schema = []
    output_schema = {"reset": "bool"}

    def run(self):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        client = node.get_client(Empty, "reset")
        if not client.wait_for_service(timeout_sec=5.0):
            raise Exception("Service not available, aborting...")

        req = Empty.Request()
        with node.node_lock:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            if result is None:
                raise Exception("Service call failed timeout, aborting...")

        return {"reset": True}


@vulcanai_tool
class ClearTurtleSimTool(AtomicTool):
    name = "clear_turtlesim"
    description = "Clear the turtlesim environment, meaning it erases every turtle trail. 'cleared' indicates success."
    tags = ["turtlesim", "clear", "erase", "service"]
    input_schema = []
    output_schema = {"cleared": "bool"}

    def run(self):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")

        client = node.get_client(Empty, "clear")
        if not client.wait_for_service(timeout_sec=5.0):
            raise Exception("Service not available, aborting...")

        req = Empty.Request()
        with node.node_lock:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            if result is None:
                raise Exception("Service call failed timeout, aborting...")

        return {"cleared": True}


@vulcanai_tool
class AbsoluteTeleportTurtleTool(AtomicTool):
    name = "absolute_teleport_turtle"
    description = "Teleport a turtle 'name' to an absolute position in turtlesim. 'teleported' indicates success."
    tags = ["turtlesim", "teleport", "absolute", "service"]
    input_schema = [
        ("name", "string"),
        ("x", "float"),
        ("y", "float"),
        ("theta", "float"),
    ]
    output_schema = {"name": "string", "teleported": "bool"}

    def run(self, **kwargs):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        if not kwargs.get("name", ""):
            print("No turtle name provided, aborting...")
            return {"name": "", "teleported": False}

        name = kwargs.get("name", "")
        client_name = f"/{name}/teleport_absolute"

        client = node.get_client(TeleportAbsolute, client_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise Exception("Service not available, aborting...")

        req = TeleportAbsolute.Request()
        req.x = kwargs.get("x", 0.0)
        req.y = kwargs.get("y", 0.0)
        req.theta = kwargs.get("theta", 0.0)

        with node.node_lock:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            if result is None:
                raise Exception("Service call failed timeout, aborting...")

        return {"name": name, "teleported": True}


@vulcanai_tool
class RelativeTeleportTurtleTool(AtomicTool):
    name = "relative_teleport_turtle"
    description = "Teleport a turtle 'name' to a relative position in turtlesim. 'teleported' indicates success."
    tags = ["turtlesim", "teleport", "relative", "service"]
    input_schema = [
        ("name", "string"),
        ("linear", "float"),
        ("angular", "float"),
    ]
    output_schema = {"name": "string", "teleported": "bool"}

    def run(self, **kwargs):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        if not kwargs.get("name", ""):
            print("No turtle name provided, aborting...")
            return {"name": "", "teleported": False}

        name = kwargs.get("name", "")
        client_name = f"/{name}/teleport_relative"

        client = node.get_client(TeleportRelative, client_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise Exception("Service not available, aborting...")

        req = TeleportRelative.Request()
        req.linear = kwargs.get("linear", 0.0)
        req.angular = kwargs.get("angular", 0.0)

        with node.node_lock:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            if result is None:
                raise Exception("Service call failed timeout, aborting...")

        return {"name": name, "teleported": True}


@vulcanai_tool
class GetTurtlePose(AtomicTool):
    name = "get_turtle_pose"
    description = "Get the current pose of a turtle 'name' in turtlesim. Fails if something goes wrong."
    tags = ["turtlesim", "pose", "position", "location"]
    input_schema = [
        ("name", "string"),
    ]
    output_schema = {
        "name": "string",
        "x": "float",
        "y": "float",
        "theta": "float",
    }

    def run(self, name: str = ""):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        name = name or "turtle1"
        topic = f"/{name}/pose"

        msg = node.wait_for_message(Pose, topic, timeout_sec=5.0)
        if msg is None:
            print(f"Could not get pose for turtle '{name}', aborting...")
            raise Exception("No pose message received")

        return {"name": name, "x": msg.x, "y": msg.y, "theta": msg.theta}

@vulcanai_tool
class MoveTurtleTool(AtomicTool):
    name = "move_turtle"
    description = "Move the turtle 'name' with 'linear' and 'angular' velocity by publishing the message 'duration' times (seconds). Use zero velocity to stop. 'success' indicates if the command was sent correctly."
    tags = ["turtlesim", "move", "velocity", "cmd_vel", "stop", "draw"]
    input_schema = [
        ("name", "string"),
        ("linear", "float"),
        ("angular", "float"),
        ("duration", "int"),
    ]
    output_schema = [{"success": "bool"}]

    def run(self, **kwargs):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        if not kwargs.get("name", ""):
            print("No turtle name provided, aborting...")
            return {"success": False}

        name = kwargs.get('name', "")
        pub = node.get_publisher(Twist, f"/{name}/cmd_vel")
        msg = Twist()
        msg.linear.x = float(kwargs.get("linear", 0.0))
        msg.angular.z = float(kwargs.get("angular", 0.0))
        for idx in range(int(kwargs.get("duration", 1.0))):
            node.get_logger().info(f"Publishing message {idx + 1} to topic /{name}/cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
            pub.publish(msg)
            time.sleep(1)
        return {"success": True}


@vulcanai_tool
class DrawRectangleTool(CompositeTool):
    name = "draw_rectangle"
    description = "Move the turtle 'name' in a rectangular shape. 'success' indicates if rectangle was drawn successfully."
    tags = ["turtlesim", "draw", "rectangle", "move", "cmd_vel"]
    input_schema = [
        ("name", "string"),
        ("size", "float"),
    ]
    output_schema = {"success": "bool"}
    dependencies = ["move_turtle", "relative_teleport_turtle"]

    def run(self, name: str = "", size: float = 2.0):
        node = self.bb.get("main_node", None)
        if node is None:
            raise Exception("Could not find shared node, aborting...")
        if not name:
            print("No turtle name provided, aborting...")
            return {"success": False}

        # Access the instances of the dependent tools and set their blackboards
        move_tool = self.resolved_deps.get("move_turtle", None)
        tp_relative_tool = self.resolved_deps.get("relative_teleport_turtle", None)
        move_tool.bb = self.bb
        tp_relative_tool.bb = self.bb

        name = name or "turtle1"
        size = size
        linear_speed = 1.0
        angular_turn = pi / 2

        # Arguments are passed as dictionaries when calling directly other tools
        side_1_args = {
            "name": name,
            "linear": linear_speed,
            "angular": 0.0,
            "duration": size / linear_speed,
        }
        side_2_args = {
            "name": name,
            "linear": linear_speed,
            "angular": 0.0,
            "duration": (size - 1) / linear_speed,
        }
        turn_args = {
            "name": name,
            "linear": 0.0,
            "angular": angular_turn,
        }

        # Pass arguments as kwargs
        move_tool.run(**side_1_args)
        tp_relative_tool.run(**turn_args)
        move_tool.run(**side_2_args)
        tp_relative_tool.run(**turn_args)
        move_tool.run(**side_1_args)
        tp_relative_tool.run(**turn_args)
        move_tool.run(**side_2_args)
        tp_relative_tool.run(**turn_args)

        return {"success": True}

