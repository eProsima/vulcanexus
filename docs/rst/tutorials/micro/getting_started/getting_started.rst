
.. micro_ros_getting_started:

Getting started micro-ROS
=========================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none

Overview
--------

This tutorial provides step-by-step instructions to use Vulcanexus micro tools to build and execute a micro-ROS publisher/subscriber demo.

Prerequisites
-------------

Ensure that the Vulcanexus installation includes Vulcanexus micro (either ``vulcanexus-humble-desktop``, ``vulcanexus-humble-micro``, or ``vulcanexus-humble-base``).
Also, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash


Create a micro-ROS workspace
----------------------------

A workspace is a directory containing ROS 2 packages. Before using ROS 2, it's necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2's packages available for you to use in that terminal.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

    # Best practice is to create a new directory for every new workspace.
    mkdir -p ~/microros_ws/src
    # Another best practice is to put any packages in your workspace into the src directory.
    cd ~/microros_ws/src


Build micro-ROS library
^^^^^^^^^^^^^^^^^^^^^^^

The first step is to download and build micro-ROS sources into the created workspace. Vulcanexus micro includes the ``micro_ros_setup`` tool, which will handle all the needed steps.

Let's begin using the ``create_firmware_ws.sh`` command with ``host`` as target:

.. code-block:: bash

  # Create firmware step.
  ros2 run micro_ros_setup create_firmware_ws.sh host

This command will download all needed sources and tools to build micro-ROS as a library. A folder named firmware must be present in your workspace, which will old platform specific tools and source code:

.. code-block:: text

  ~/microros_ws/src/
  ├── firmware
  │   └── dev_ws
  └── src
      ├── eProsima
      │   ├── Micro-CDR
      │   └── Micro-XRCE-DDS-Client
      ├── ros2
      │   ├── common_interfaces
      │   ├── example_interfaces
      │   ├── rcl_interfaces
      │   └── unique_identifier_msgs
      └── uros
          ├── micro-ROS-demos
          ├── micro_ros_msgs
          ├── micro_ros_utilities
          ├── rclc
          ├── rmw_microxrcedds
          └── rosidl_typesupport_microxrcedds

As there is no need for extra configuration steps usually needed for cross-compilation or RTOS configuration, let's build:

.. code-block:: bash

  # Build step.
  ros2 run micro_ros_setup build_firmware.sh

This completes the micro-ROS library build. Remember sourcing this workspace before building or running a micro-ROS app.

.. code-block:: bash

  # Source micro-ROS installation.
  source install/local_setup.bash

.. note::

  A set of examples apps for Linux is also present and build under the ``src/uros/micro-ROS-demos/rclc`` directory.

Create a package
^^^^^^^^^^^^^^^^

Recall that packages should be created in the ``src`` directory, not the root of the workspace. So, navigate into microros_ws/src, and run the package creation command:

.. code-block:: bash

    ros2 pkg create --build-type ament_cmake micro_pubsub

.. code-block:: text

    micro_pubsub/
    ├── CMakeLists.txt
    ├── package.xml
    ├── include
    │   └── micro_pubsub
    └── src


Add micro-ROS apps to the workspace
-----------------------------------

After the workspace is ready and sourced, micro-ROS applications can be added, build and run.

.. note::

  Certain topics as message memory handling or micro-ROS API details are not covered on this tutorial. Check the `micro-ROS User API <_tutorials_micro_user_api>`_ tutorial for more details.

Write the micro-ROS publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On this example app, a publisher will send a periodic string message using a configurable timer.

Add the publisher source code on ``micro_pubsub/src/micro_publisher.c``:

.. literalinclude:: /resources/tutorials/micro/getting_started/micro_publisher.c
    :language: c

Write the micro-ROS subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This subscriber example is very similar to the publisher, but here the message reception will be handle by the subscriber callback if there is enough memory available.

Add the subscriber source code on ``micro_pubsub/src/micro_subscriber.c``:

.. literalinclude:: /resources/tutorials/micro/getting_started/micro_subscriber.c
    :language: c

Add dependencies
^^^^^^^^^^^^^^^^

Before building this examples, micro-ROS used headers and tools need to be included on the created ROS2 package:

1. Open ``package.xml`` on ``microros_ws/src/micro_pubsub`` directory

2. make sure to fill in the ``<description>``, ``<maintainer>`` and ``<license>`` tags:

  .. code-block:: xml

      <description>Examples of minimal publisher/subscriber using rclcpp</description>
      <maintainer email="you@email.com">Your Name</maintainer>
      <license>Apache License 2.0</license>

3. Add a new line after the ``ament_cmake buildtool`` dependency and paste the following dependencies corresponding to your node's include statements:

  .. code-block:: xml

      <depend>rcl</depend>
      <depend>rclc</depend>
      <depend>std_msgs</depend>
      <depend>rmw_microxrcedds</depend>

4. Add the dependencies to the `CMakeLists.txt` file, the following example can be used:

  .. literalinclude:: /resources/tutorials/micro/getting_started/CMakeLists.txt

Build
-----

Once the source code is ready, build your new package:

.. code-block:: bash

  # ~/microros_ws
  colcon build --packages-select micro_pubsub

Run
------------

Start the Agent
^^^^^^^^^^^^^^^

The agent is included on Vulcanexus micro tool set. The default transport configuration for micro-ROS is UDPv4 with the Agent on ``127.0.0.1:8888``:

.. code-block:: bash

    # Start the agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v5

This command will start the Agent, allowing communication between our micro-ROS apps and ROS2 environment.

.. note::

  At the top of the apps ``main`` function, micro-ROS is initialized using the ``rclc_support_init`` method.
  It is important that the Agent is reachable on this step, as the method will fail if a connection cannot be established.

Run the apps
^^^^^^^^^^^^^^^

Before running the apps, set Micro XRCE-DDS as RMW implementation for each terminal:

.. code-block:: bash

  # Set Micro XRCE-DDS as RMW implementation
  export RMW_IMPLEMENTATION=rmw_microxrcedds

Run the publisher in one terminal:

.. code-block:: bash

  # ~/microros_ws
  ros2 run micro_pubsub publisher

Then run listener in another terminal:

.. code-block:: bash

  # ~/microros_ws
  ros2 run micro_pubsub subscriber

The listener will start printing messages to the console. Notice how the creation of entities and each publication appears on the Agent log.
