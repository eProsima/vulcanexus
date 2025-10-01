.. _tutorials_vulcanai_turtlesim_main:

VulcanAI Console and manager
============================

In the previous tutorial (:ref:`tutorials_vulcanai_turtlesim_tools`), we created a package that contains a shared ROS 2 node and the tools needed to control TurtleSim using VulcanAI.
Now, we will create the main script that will be used to run the VulcanAI console and manage the interaction between the user, the agent, and the tools.

VulcanAI console is a command-line interface that allows users to interact with the VulcanAI agent.
It provides a simple way to input natural language commands and check the agent's responses.
VulcanAI console already includes the manager that handles the creation and management of agents, tools, and user contexts.
Therefore, we only need to create a script that initializes the console, registers the tools we created, and starts the interaction loop.

Write the main method
---------------------

In the same file where we define the ``SharedNode`` class, we can write the main method that will be used as entry point for the package.

Just add the following content at the end of the ``ros2_node.py`` file:

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 69-88

And the following include at the beginning of the file:

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 7

Examine the code
^^^^^^^^^^^^^^^^

First, we initialize the ``rclpy`` library and create an instance of the ``SharedNode`` class.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 72-73
    :dedent: 4

Then, we write a brief context that will inform the agent about the purpose of the node, which is to control TurtleSim.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 75-77

Next, we will use the ``VulcanConsole`` as main interface for the user to interact with the agent.
This object already includes all classes and logic necessary to create and manage agents, tools and user contexts.
We only need to specify the name of the module where the tools will be installed after building the package, and add the user context to the console manager.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 79-81
    :dedent: 4

Finally, we manually add the shared node to the console manager blackboard, so it can be accessed by the tools and call the ``run`` method to start the console.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 82-84
    :dedent: 4

Last two lines calls the ``main`` method when the script is executed directly.

.. literalinclude:: /resources/tutorials/vulcanai/turtlesim/ros2_node.py
    :language: python
    :lines: 87-88

As you can see, the main method is quite simple, as most of the complexity is encapsulated in the ``SharedNode`` class.
Instantiating the console and registering the tools is very straightforward.

Build the package
-----------------

Now, we only need to build the package and run the ``ros2_node.py`` script to start the console and interact with the agent.

However, we will first add the recently created ``ros2_node.py`` file to the ``setup.py`` file, so it can be installed and run using the ``ros2 run`` command.
Additionally, we will create a new entry-point for the package, where we will install the tools.
Replace the ``entry_points`` section of the ``setup.py`` file with the following content:

.. code-block:: python

    entry_points={
        "console_scripts": [
            "vulcanai_turtlesim_demo = vulcanai_turtlesim_demo.ros2_node:main",
        ],
        "turtle_tools": [
            "turtle_tools = vulcanai_turtlesim_demo.turtlesim_tools",
        ],
    },

Then, go to the root of your workspace and build the package:

.. code-block:: bash

    cd ~/<your_workspace> && \
    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && \
    colcon build --packages-select vulcanai_turtlesim_demo && \
    source install/setup.bash

Remember that if VulcanAI is installed in a virtual environment, you need to activate it before sourcing the workspace.
Check the :ref:`Running VulcanAI from virtual environment with Vulcanexus <vulcanai_from_venv>` section for more details.

After successfully building the package, the node is ready to be launched and tested.
Continue with the :ref:`Using VulcanAI to control TurtleSim <tutorials_vulcanai_turtlesim_usage>` tutorial to learn how to use the created tools to control TurtleSim through natural language instructions.

Get all the code
----------------

Instead of following all the steps in the :ref:`VulcanAI Tools <tutorials_vulcanai_turtlesim_tools>` and :ref:`VulcanAI Console and manager <tutorials_vulcanai_turtlesim_main>` tutorials, you can also get all the code with the following command:

.. code-block:: bash

    cd ~/<your_workspace> && \
    mkdir temp && \
    mkdir src && \
    git clone https://github.com/eProsima/vulcanexus.git temp/ && \
    cp -r temp/code/vulcanai_turtlesim/vulcanai_turtlesim_demo src/. && \
    rm -rf temp/ && \
    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && \
    colcon build --packages-select vulcanai_turtlesim_demo && \
    source install/setup.bash

Remember to replace ``<your_workspace>`` with the path to your ROS 2 workspace.
The code will be downloaded to the ``src/vulcanai_turtlesim_demo`` folder of your workspace and can be directly built and run.
Getting the code this way is equivalent to following all the steps in the previous tutorials.
