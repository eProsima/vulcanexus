from vulcanai import AtomicTool, CompositeTool, vulcanai_tool

@vulcanai_tool
class AddTool(AtomicTool):
    name = "add"
    description = "Adds two numbers together."
    input_schema = [("a", "float"), ("b", "float")]
    output_schema = {"result": "float"}

    def run(self, a: float, b: float):
        return {"result": a + b}

@vulcanai_tool
class MultiplyTool(AtomicTool):
    name = "multiply"
    description = "Multiplies two numbers together."
    input_schema = [("a", "float"), ("b", "float")]
    output_schema = {"result": "float"}

    def run(self, a: float, b: float):
        return {"result": a * b}

@vulcanai_tool
class AddAndMultiplyTool(CompositeTool):
    name = "add_and_multiply"
    description = "Adds two numbers and then multiplies the result by a third number."
    input_schema = [("a", "float"), ("b", "float"), ("c", "float")]
    output_schema = {"result": "float"}
    dependencies = ["add", "multiply"]

    def run(self, a: float = 1, b: float = 1, c: float = 1):
        add_result = self.resolved_deps["add"].run(a=a, b=b)["result"]
        multiply_result = self.resolved_deps["multiply"].run(a=add_result, b=c)["result"]
        return {"result": multiply_result}
