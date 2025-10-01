from vulcanai.manager import ToolManager
manager = ToolManager()

# Register tools from the math_tools.py file
manager.register_tools_from_file("path/to/math_tools.py")

# Register tools from an installed entry point named 'math_tools'
manager.register_tools_from_entry_points("math_tools")

# Register tools manually
manager.register_tool(AddTool())
manager.register_tool(MultiplyTool())
manager.register_tool(AddAndMultiplyTool())
