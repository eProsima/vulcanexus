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
