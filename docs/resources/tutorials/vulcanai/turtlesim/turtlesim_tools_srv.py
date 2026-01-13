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
