from vulcanai import AtomicTool, vulcanai_tool

@vulcanai_tool
class AddTool(AtomicTool):
    name = "add"
    description = "Adds two numbers together."
    input_schema = [("a", "float"), ("b", "float")]
    output_schema = {"result": "float"}

    def run(self, a: float, b: float):
        return {"result": a + b}

@vulcanai_tool
class AddAndMultiplyTool(AtomicTool):
    name = "add_and_multiply"
    description = "Multiply the result of an addition by a third number."
    input_schema = [("c", "float")]
    output_schema = {"result": "float"}

    def run(self, c: float = 1):
        result = 0
        output_add = self.bb.get("add", None)
        if output_add:
            result = output_add.get("result", 0)
        if not result:
            print("Warning: 'result' not found in blackboard or is zero, defaulting to 1.")
            result = 1

        multiply_result = result * c
        print(f"Multiplying result from blackboard ({result}) by c ({c}) to get {multiply_result}")
        return {"result": multiply_result}
