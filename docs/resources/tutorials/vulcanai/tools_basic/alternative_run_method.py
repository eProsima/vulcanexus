def run(self, **kwargs):
    a = kwargs.get("a", 1)
    b = kwargs.get("b", 1)
    c = kwargs.get("c", 1)
    # Same code as before from here
    add_result = self.resolved_deps["add"].run(a=a, b=b)["result"]
    multiply_result = self.resolved_deps["multiply"].run(a=add_result, b=c)["result"]
    return {"result": multiply_result}
