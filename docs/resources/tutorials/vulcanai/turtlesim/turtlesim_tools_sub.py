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
