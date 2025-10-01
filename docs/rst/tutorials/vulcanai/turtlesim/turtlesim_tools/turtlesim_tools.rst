.. _tutorials_vulcanai_turtlesim_tools:

Tools for Turtlesim
===================

The first step when creating new tools for VulcanAI agents is to decide:

- What does the tool need to accomplish.
- What are the inputs that the tool is going to receive.
- What should be the output that the tool will generate.

By answering these questions, we can define the ``name``, ``description``, ``input_schema`` and ``output_schema`` of the tool, which are essential for the agent to understand how to use it.

For this tutorial, we will create a tool for most basic functionalities of TurtleSim, including services for reseting the environment or spawning new turtles and classic pub-sub topology to move the turtles.
First, create a ROS 2 package inside a your workspace (make sure to replace ``<your_workspace>`` with the path to your ROS 2 workspace):

.. code-block:: bash

    cd ~/<your_workspace>/src && \
    ros2 pkg create --build-type ament_python vulcanai_turtlesim_demo && \
    cd vulcanai_turtlesim_demo

Sharing a single ROS 2 Node between tools
-----------------------------------------

In order to be able to interact with the ROS 2 environment, the tools need to have access to a ROS 2 node, as they will need to create publishers, subscribers or service clients.
To avoid creating a new node for each tool, which would be highly inefficient as it would require the discovery of a new entity in the ROS 2 network for every single tool call, we will create a single ROS 2 node and share it between all the tools.

In this way, all the tools will be able to reuse the same entities created by other calls, and it will also exemplify how to share data between tools by means of the *blackboard*.
The blackboard is a shared memory space where tools can write and read data, allowing them to share information without the need for the LLM to reason about how to pass data between tools.
To get a better understanding of this concept, please refer to the :ref:`Sharing data between tools <vulcanai_tools_sharing_data>` section in the :ref:`VulcanAI Tools <vulcanai_tools>` tutorial.

To create the shared ROS 2 node, create a new file called ``ros2_node.py`` inside the ``vulcanai_turtlesim_demo/vulcanai_turtlesim_demo`` folder with the following content:

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 1-5, 10-66

Examine the code
^^^^^^^^^^^^^^^^

This code creates a ROS 2 node that can be shared between multiple tools, exposing methods to provide and reuse service clients and publishers.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 1-5

We import the basic ``threading`` module to create a lock that will ensure that the creation of new entities is thread-safe.
If you don't how locks work, just consider them as a protection mechanism needed to avoid errors when **multiple threads** access the same shared resource simultaneously.
It will avoid the creation of duplicate entities when multiple tools try to create them at the same time.

The other imports are related to ROS 2, including the ``rclpy`` library, the ``Node`` class and a ``Future`` object that will be used to wait for messages.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 10-19

The ``SharedNode`` class inherits from the ``Node`` class, and it initializes the node with the name ``vulcanai_shared_node``.
Then, two dictionaries are created to store a reference to created service clients and publishers, so they can be reused later.
Finally, a lock from the ``threading`` module is created to ensure thread-safety.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 21-32

The method ``get_client`` creates a new service client if it doesn't exist yet, or returns the existing one if it has already been created.
Everything under the ``with self.lock:`` statement is protected by the lock, ensuring that no duplicated service clients are created.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 34-45

The method ``get_publisher`` works in a similar way, creating or returning an existing publisher.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 47-66

Finally, the method ``wait_for_message`` is a utility function that allows to wait for a single message from a topic, which is needed for tools that will use subscriptions.
Subscriptions, unlike publishers or service clients, will not be reused.

To control TurtleSim we do not require to keep a subscription alive and handling its callbacks between tool calls, as we will only need to get the current state of the turtles when requested by the user.
Therefore, we will create a new subscription every time we need to get a message from a topic, spin the node until we get a message, and then destroy it after receiving the first message.
The ``Future`` object is used in the subscriber callback to wait for the message to be received.

Define the tools
----------------

TurtleSim exposes multiple services and topics that can be used to control the turtles and interact with the environment.
For this tutorial, tools can be obtained directly from Vulcanexus repository.
We will only explain the most relevant parts of the code, illustrating each type of tool:

- An *AtomicTool* that calls a service to spawn a turtle.
- An *AtomicTool* that creates a publisher to move a turtle.
- An *AtomicTool* that creates a subscription to get the current state of a turtle.
- A *CompositeTool* that combines multiple *AtomicTool*s to draw a rectangle.

Run the following commands to download the tools (make sure to replace ``<your_workspace>`` with the path to your ROS 2 workspace):

.. code-block:: bash

    cd ~/<your_workspace>/src/vulcanai_turtlesim_demo/vulcanai_turtlesim_demo && \
    wget https://https://raw.githubusercontent.com/eProsima/vulcanexus/refs/heads/main/docs/resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py

Examining a tool with a service client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To explain how to implement a tool that calls a service, we will use the ``SpawnTurtleTool`` as example.
This tool will call the ``/spawn`` service to create a new turtle in the environment, which uses the ``turtlesim_msgs/srv/Spawn`` service type.

The tool is marked with the ``@vulcanai_tool`` decorator, which is needed to register the tool in VulcanAI.
It inherits from the ``AtomicTool`` class, as it only performs a single action and does not have any dependencies on other tools.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 14-25

The ``name`` and ``description`` attributes are used by the agent to understand the purpose of the tool.
The ``tags`` attribute is optional, but here it is used to categorize the tool and also add keywords that can help the agent to find it when needed.
The ``input_schema`` and ``output_schema`` attributes define the inputs and outputs of the tool.
In this case, the tool requires the name of the turtle to be spawned, as well as its initial position and orientation.
The output is the name of the spawned turtle and a boolean indicating if the opearation was successful.

The ``run()`` method contains the logic of the tool.
It first retrieves the shared ROS 2 node from the blackboard, and then calls the already explained ``get_client()`` method of the shared node for the ``/spawn`` service.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 27-34

Note that if the *shared_node* is not found or the service is not available, the tool will raise an exception and abort the operation.
Exceptions are handled by VulcanAI, automatically assingning a null output to the tool and informing the agent about the error.

Then, a request object is created and filled with the input data, which is retrieved from the `kwargs` dictionary.
Finally, the service is called and the future is awaited until the response is received before returning the output.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 36-49

The output is returned as a dictionary, matching the defined output schema.

The same pattern is applied to other tools that call services, such as the ``KillTurtleTool``, ``ResetTurtlesimTool``, ``ClearTurtlesimTool``, ``AbsoluteTeleportTurtleTool`` and ``RelativeTeleportTurtleTool``.

Examining a tool with a publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``MoveTurtleTool`` uses a publisher to send velocity commands to a turtle.
The tool is defined in the same way as the previous one, with the necessary basic attributes to inform the agent about its purpose and how to use it.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 255-266

The run method differs from the previous one, as it needs to create a publisher instead of a service client.
First, the input parameters are retrieved from the ``kwargs`` dictionary.
The name is used to create the topic name, which follows the pattern ``/<turtle_name>/cmd_vel``.
Linear and angular velocities are used to create a ``Twist`` message that will be published to the topic.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 268-280

Then, the generated message is published to the topic.
A for loop is used to publish the message multiple times, based on the ``duration`` input parameter.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 280-285

Examining a tool with a subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``GetTurtlePoseTool`` uses a subscription to get the current pose of a turtle.
The tool is defined in the same way as the previous ones, with the necessary basic attributes to inform the agent about its purpose and how to use it.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 226-239

In this case, the ``run()`` method needs to call the ``wait_for_message()``, which creates a subscription to the topic and waits for a single message to be received.
Note that this method is the one responsible of the spinning of the node until a message is received, so we don't need to call ``rclpy.spin()`` or similar methods.
The output of the method is the received message, which is then used to fill the output dictionary.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 241-253

Take a look at the definition of the method ``run(self, name: str = "")``.
The input parameter is defined as a regular method parameter instead of using the ``kwargs`` dictionary, which is also valid and more convenient when there are few input parameters.

Examining a tool with a *CompositeTool*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The last tool we will examine is the ``DrawRectangleTool``, which is a composite tool that combines multiple atomic tools to draw a rectangle with a turtle.
The definition of the tool remains almost the same as the previous ones, adding only one new attribute: ``dependencies``.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 288-298

The ``dependencies`` attribute is a list of tool names that will be used during the execution of the *CompositeTool*.
In this case, the tool depends on the ``MoveTurtleTool`` and ``RelativeTeleportTurtleTool`` to move the turtle and teleport it to the starting position.
Dependencies are indicated by using the exact name of the tool, as defined in the ``name`` attribute of the tool.
In this example, dependencies are ``move_turtle`` and ``relative_teleport_turtle``.

The definition of the ``run()`` is similar to the previous tools, as well as the retrieval of the shared node from the blackboard.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 300-312

However, the ``run()`` method of a *CompositeTool* adds new behavior, as it needs to reuse the functionality of the dependent tools.
To do so, *CompositeTools* owns an attribute called ``self.resolved_deps``, which is a dictionary that contains a reference to the dependent tools, indexed by their name.
This dictionary is automatically filled by VulcanAI when the tool is instantiated, so we can directly use it in the ``run()`` method.

Note lines:

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 311-312

It is important to remark how we are manually assigning the current *blackboard* to the dependent tools.
This step is usually done by the VulcanAI manager when tools are called, but in this case we need to do it manually, as the *CompositeTool* is the one being called by the agent, and not the dependent tools.

The Rectangle will be drawn by moving the turtle in a straight line and then turning 90 degrees, repeating this process four times but with different distances for the sides.
Therefore, we create 3 sets of arguments that will be passed to the ``move_tool`` to move the turtle the desired distance and to ``tp_relative_tool`` to teleport it a rotation of 90 degrees.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 314-336

Finally, we call the ``run()`` method of the dependent tools with the corresponding arguments, and return a success output.
We could check the output of each tool call to ensure that the operation was successful, but for simplicity we will assume that everything works as expected.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/turtlesim_tools.py
    :language: python
    :lines: 338-346

Next steps
----------

In this tutorial we have learned how to create custom tools for VulcanAI to control TurtleSim, using a shared ROS 2 node to create and reuse service clients and publishers.
We have examined different types of tools, including *AtomicTools* that call services, create publishers and subscriptions, and a *CompositeTool* that combines multiple *AtomicTools* to accomplish a more complex task.

Now, we are missing the main script that will be used to run the VulcanAI console and manage the interaction between the user, the agent, and the tools.
Continue with the :ref:`VulcanAI Console and manager <tutorials_vulcanai_turtlesim_main>` tutorial to learn how to create the main script and run the console.
