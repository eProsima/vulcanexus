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
