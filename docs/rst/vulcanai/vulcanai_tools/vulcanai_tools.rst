.. _vulcanai_tools:

VulcanAI Tools
==============

.. note::
   VulcanAI is currently in active development, and new features and improvements are being added regularly.
   Current version is in Beta stage.

Background
----------

VulcanAI is a library that provides a framework to easily and flexibly create powerful AI applications.
To do so, it implements the concept of *tools*, which are Python classes that define specific functionalities that can be used by VulcanAI agents to accomplish complex tasks.
Tools can be as simple as a calculator or a web search engine, or as complex as a robot controller or a data analysis pipeline.

Tools are designed to be modular and reusable, but most importantly, to be easily integrated with Large Language Models (LLMs) output and reasoning.
By combining different tools, VulcanAI agents can perform a wide range of tasks in various environments.
This tutorial provides an overview of the different types of tools available in VulcanAI, as well as how to create custom tools to extend its capabilities.

Definition of Tools
-------------------

In the context of VulcanAI, a *tool* is a Python class that encapsulates a specific functionality or a set of related functionalities.
Each tool can be thought of as a building block that can be combined with other tools to create more complex behaviors and capabilities.
Every tool must inherit from either the `AtomicTool` or the `CompositeTool` class of the `VulcanAI` library, depending on whether it represents a single action or a composition of multiple actions.
Both types inherit from the base `ITool` class, which provides common functionality and interfaces for all tools.

- **Atomic tools** are the simplest type of tools, representing a single action.
  They do not have dependencies on other tools and can be executed independently.
  They are designed to be used as building blocks for more complex behaviors and can be easily combined with other tools to create more sophisticated workflows.
  An *AtomicTool* could be a tool listing ROS 2 topics, or a tool that performs a computation and then calls a ROS 2 service with the result.
- **Composite tools**, on the other hand, are more complex and can encapsulate multiple actions or behaviors.
  They have dependencies on other tools, which are specified during their initialization.
  Composite tools can be thought of as higher-level abstractions that combine the functionalities of multiple atomic tools to achieve a specific goal.
  An example of a *CompositeTool* could be a tool that gets all available ROS 2 topics through an atomic tool, and then uses that information to publish a message to a specific topic using another atomic tool.

  The idea of *CompositeTools* is to provide an interface to create more complex behaviors by orchestrating the execution of multiple atomic tools which must act always in the same coordinated way.
  Note that most LLMs are able to obtain the same behavior of a *CompositeTool* by calling the atomic tools it depends on in the correct order, but this requires the LLM to reason about the correct sequence of actions to achieve the desired outcome.

  Composite tools allow to reduce indetermination and increase reliability by providing a predefined sequence of actions that are known to work together effectively.
  Also, the same behavior achieved through a *CompositeTool* could also be accomplished by a single *AtomicTool* that implements all the logic internally, but this would reduce modularity and reusability, as the complex behavior would be tightly coupled to a single tool.
  The decision to use a *CompositeTool* or a single *AtomicTool* depends on the developer's preference and the specific use case.

Tool Interfaces
---------------

.. figure:: /rst/figures/vulcanai/tools_class_uml.png
   :align: center

The class *ITool* defines the interface that all tools must implement.
It provides the basic structure and functionality that all tools share, ensuring consistency and interoperability between different tools.
The main methods and attributes defined in the *ITool* class are:

- `name`: A string attribute that defines the name of the tool. It must uniquely identify the tool so the agent can call it by name.
- `description`: A string attribute that provides a brief description of the tool's functionality. This description is used by the LLM to understand what the tool does and when to use it.
- `tags`: A list of strings that can be used to categorize and organize tools. When working with big number of tools, the library performs a `top-k` algorithm to rank the tools based on their relevance to the current task. Tags can help agents to find and select the appropriate tools for a given task.
- `input_schema`: A list of Tuples composed of two strings, where the first string is the name of an input parameter and the second string is the type of the parameter (e.g., "str", "int", "float" or "bool"). This schema defines the expected inputs for the tool, which helps the LLM to understand how to call the tool correctly.
- `output_schema`: A dictionary that defines the expected output of the tool. The keys are the names of the output parameters, and the values are their corresponding types (e.g., "str", "int", "float" or "bool"). This schema helps the LLM to understand what kind of output to expect from the tool.
- `run(self, **kwargs)`: An abstract method that **must** be implemented by all subclasses. This method contains the logic for executing the tool's functionality. It must take arguments corresponding to the input parameters defined in the `input_schema` and return a dictionary containing the output parameters defined in the `output_schema`. This is the method that will be called by the VulcanAI agent when it decides to use the tool.

Both *AtomicTool* and *CompositeTool* inherit from the *ITool* class, so they share the same interface and must implement the same methods and attributes.

Composite tools have two additional attributes:

- `dependencies`: A list of tool names (strings) that the composite tool depends on. These dependencies are other tools that must be available for the composite tool to function correctly. When a Composite tool is registered, this list is used to ensure that all its dependencies are available.
- `resolved_deps`: A dictionary where keys represent tool names and the values are instances of the tools. This parameter is automatically configured by the library when resolving dependencies and it must be used when defining the run method to call other tools.

Registering tools
-----------------

VulcanAI provides multiple mechanisms to register tools so they can be used by VulcanAI agents.
First of all, each tool must be decorated with the `@vulcanai_tool` decorator, which indicates the tool registry that a tool should be registered in.

.. literalinclude:: /resources/tutorials/vulcanai/tools_basic/math_tools.py
    :language: python
    :lines: 3-4

Once the tools are decorated, they can be registered in the following ways:

- By passing the tools file path to the tool registry

  .. literalinclude:: /resources/tutorials/vulcanai/tools_basic/register_tools.py
      :language: python
      :lines: 1-3, 4-5

- By adding the tools file as an entrypoint of an installed python module.

  .. literalinclude:: /resources/tutorials/vulcanai/tools_basic/register_tools.py
      :language: python
      :lines: 1-3, 7-8

- By manually registering the tool class in the tool registry:

  .. literalinclude:: /resources/tutorials/vulcanai/tools_basic/register_tools.py
      :language: python
      :lines: 1-3, 10-13

.. note::

  To install an entrypoint, you can add the following to your *"setup.py"* file:

  .. code-block:: python

      entry_points={
            "custom_tools": [
                "custom_tools = my_custom_tools.my_custom_tools",
            ],
        },

  or the following to your *"pyproject.toml"* file:

  .. code-block:: toml

      [project.entry-points."math_tools"]
      math_tools = "my_module.math_tools"

  In case of using a *"setup.py"* file of a ROS 2 package, remember to add a new complete module, not another entry to the *"console_scripts"* entry point.

Tool Examples
-------------

Here are some examples of both atomic and composite tools to illustrate how they can be implemented in VulcanAI.

.. literalinclude:: /resources/tutorials/vulcanai/tools_basic/math_tools.py
    :language: python

Note that the ``run()`` method can also be defined with `**kwargs` to make it more flexible or easier for tools that require multiple inputs.
In this case the input parameters must be extracted from the `kwargs` dictionary.
The following ``run()`` method is equivalent to the previous one of the `AddAndMultiplyTool` tool:

.. literalinclude:: /resources/tutorials/vulcanai/tools_basic/alternative_run_method.py
    :language: python

Also, note that we have not defined any constructor (`__init__` method) for the tools.
This is because the base classes already provide a default constructor that handles the initialization of the tool's attributes.

Lastly, take into account that no `tags` attribute has been defined for these tools.
This is an optional attribute that can be used to categorize and organize tools, which can be especially useful when working with a large number of tools, but it is not strictly necessary for the tool to function correctly.

To test these examples, we can use the **VulcanAI console**, which provides an interactive environment to experiment with VulcanAI agents and tools.
First, we instantiate the console registering the tools in the console:

.. code-block:: bash

    wget -O math_tools.py https://raw.githubusercontent.com/eProsima/vulcanexus/kilted/docs/resources/tutorials/vulcanai/tools_basic/math_tools.py && \
    vulcanai_console --register-from-file math_tools.py

Now, use the console terminal to interact with the VulcanAI agent and test the tools:

.. code-block:: bash

    [USER] >>> add 4 and 2
    [USER] >>> multiply 6 and 7
    [USER] >>> add 4 and 2, then multiply the result by 7

You can check the console logs to see the plan created by the agent to accomplish each task, as well as the tool calls made during the execution.

.. _vulcanai_tools_sharing_data:

Sharing data between tools
--------------------------

Tools are designed to accept input parameters and return output parameters, which allows them to share data and interact with each other.
When a tool is called by a VulcanAI agent, the agent provides the necessary input parameters based on the tool's `input_schema`.
The tool then must process the input data and return the output parameters as defined in its `output_schema`, in dictionary format.

However, there might be scenarios where the output of one tool needs to be used as the input for another tool.
For example, if there is a tool that retrieves data from a database and another tool that processes that data, the output of the first tool must be passed as input to the second tool.
VulcanAI agents are capable of handling such scenarios by managing the flow of data between tools, and automatically assigning a dynamic input which will be resolved at runtime.

A quick example using the tools defined in the previous section can illustrate this concept.
First, delete the `AddAndMultiplyTool` composite tool from the tools files and then call the VulcanAI console again:

.. code-block:: bash

    sed -i '24,35d' "math_tools.py" && \
    vulcanai_console --register-from-file math_tools.py

Now try the same command as before:

.. code-block:: bash

    [USER] >>> add 4 and 2, then multiply the result by 7

You should see that the agent is capable of reasoning about the data flow between the two tools, and it automatically passes a reference of the `AddTool` as an input to the `MultiplyTool`, which will be resolved at runtime.

This substitution, nonetheless, requires the agent to reason about the correct sequence of tool calls and how to pass data between them.
Although most LLMs are able to perform this reasoning, it can introduce some indetermination and reduce reliability, as the LLM might not always choose the correct sequence of tool calls or might not pass the data correctly.

To mitigate this issue, VulcanAI comes with an extra feature that allows to manually access the *blackboard* of the agent.
The *blackboard* is a shared memory space where tools write data, every time a tool is executed, its output is stored in the *blackboard*, following the output schema of the tool.
This mechanism allows to freely share any type of data between tools or even between other entities in the Python environment, without the need for the LLM to reason about how to pass data between tools.

To use the *blackboard*, tools can access it through the ``self.bb`` attribute, which is a dictionary-like object that allows to reading data.

For example, we could rewrite the `AddAndMultiplyTool` tool to use the *blackboard* like this:

.. literalinclude:: /resources/tutorials/vulcanai/tools_basic/math_tools_blackboard.py
    :language: python
    :lines: 13-31

In this way, we access the output of the `AddTool` directly from the *blackboard* instead of relying on the agent to pass it as an input.

This example can be tested in the same way as before, but now using the modified `math_tools_bb.py` file:

.. code-block:: bash

    wget -O math_tools_bb.py https://raw.githubusercontent.com/eProsima/vulcanexus/kilted/docs/resources/tutorials/vulcanai/tools_basic/math_tools_blackboard.py && \
    vulcanai_console --register-from-file math_tools_bb.py

A good rule of thumb is to use the *blackboard* for sharing data between tools when the data is complex or the input could be hard to interpret by the LLM.
For simple cases where the output of one tool is directly used as the input for another tool, it is usually better to let the agent handle the data flow, as it keeps the tools more decoupled and easier to maintain.

Next steps
----------

To learn more about VulcanAI tools and the rest of its capabilities, you can explore a real case use example in the :ref:`VulcanAI with TurtleSim <tutorials_vulcanai_turtlesim_tutorials>` tutorial, which guides you through the process of creating custom tools to control the TurtleSim simulator using VulcanAI.
