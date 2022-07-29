.. _tutorials_ros2_introduction:

.. figure:: /rst/figures/tutorials/ros/biglogo.png
    :width: 500px
    :align: center

ROS2 PubSub example
===================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none

Overview
--------

This tutorial will guide you through the process of create a new workspace, create a package, implement a simple example with a publisher and a subscriber nodes and build and run it.


Create a workspace
--------------------

A workspace is a directory containing ROS 2 packages.

To create a new workspace follow next steps:

    .. code-block:: bash

        # Before use ROS 2 functionalities it is necessary to source
        # ROS 2 installation in the terminal where you are going to work in.
        source /opt/ros/humble/setup.bash

        # Best practice is to create a new directory for every new workspace.
        mkdir -p ~/dev_ws/src
        # Another best practice is to put any packages in your workspace into the src directory.
        cd ~/dev_ws/src

Create a package
----------------

Packages are the most atomic unit of build and the unit of release.

For this tutorial we are going to create a single `cpp_pubsub` package.

Recall that packages should be created in the src directory, not the root of the workspace.

    .. code-block:: bash

        cd ~/dev_ws/src

        # Remember to source ROS 2 installation!.
        ros2 pkg create --build-type ament_cmake cpp_pubsub

        # Recall that this is the directory in any CMake package where the source files containing executables belong
        cd cpp_pubsub/src

cpp_pubsub whould look like this:

    .. code-block:: text

        cpp_pubsub/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include
        │   └── cpp_pubsub
        └── src

.. figure:: /rst/figures/tutorials/core/ros2_pub_sub/gif1.gif
    :width: 1000px
    :align: center

.. image:: ../../../resources/terminalizer/pubsub_workspace_terminalizer.gif

Create publisher_member_function.cpp
------------------------------------

Lets start with publisher node, this node is going to publish messages that hopefully are going to be received by subscriber node.

First create cpp file in our package src folder:

    .. code-block:: bash

        # on ~/dev_ws/srccpp_pubsub/src
        touch publisher_member_function.cpp

cpp_pubsub whould look like this:

    .. code-block:: text

        cpp_pubsub/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include
        │   └── cpp_pubsub
        └── src
            └── publisher_member_function.cpp

Copy and paste next code on `publisher_member_function.cpp`. Feel free to spend some time reading comments to understand the code.

    .. literalinclude:: ../../../../resources/examples/core/ros2_pub_sub/src/publisher_member_function.cpp
        :language: C++
        :linenos:

Create subscriber_member_function.cpp
-------------------------------------

As we did before, first create the cpp file:

    .. code-block:: bash

        # on ~/dev_ws/srccpp_pubsub/src
        touch publisher_member_function.cpp

Both cpp files should be under src directory:

    .. code-block:: text

        cpp_pubsub/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include
        │   └── cpp_pubsub
        └── src
            ├── subscriber_member_function.cpp
            └── publisher_member_function.cpp

topic name and message type used by the publisher and subscriber must match to allow them to communicate.

    .. literalinclude:: ../../../../resources/examples/core/ros2_pub_sub/src/subscriber_member_function.cpp
        :language: C++
        :linenos:


Add dependencies
----------------

Open `package.xml` on `dev_ws/src/cpp_pubsub` directory

make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags:

    .. code-block:: xml

        <description>Examples of minimal publisher/subscriber using rclcpp</description>
        <maintainer email="you@email.com">Your Name</maintainer>
        <license>Apache License 2.0</license>

Add a new line after the `ament_cmake buildtool` dependency and paste the following dependencies corresponding to your nodes include statements:

    .. code-block:: xml

        <depend>rclcpp</depend>
        <depend>std_msgs</depend>

CMakeLists.txt
--------------

Replace `CMakeLists.txt` content with next example:

    .. literalinclude:: ../../../../resources/examples/core/ros2_pub_sub/CMakeLists.txt
        :language: text
        :linenos:

Build
-----

You likely already have the rclcpp and std_msgs packages installed as part of your ROS 2 system.

Run rosdep in the root of your workspace (dev_ws) to check for missing dependencies before building:

    .. code-block:: bash

        # ~/dev_ws
        rosdep install -i --from-path src --rosdistro humble -y

Still in the root of your workspace, dev_ws, build your new package:

    .. code-block:: bash

        # ~/dev_ws
        colcon build --packages-select cpp_pubsub

.. figure:: /rst/figures/tutorials/core/ros2_pub_sub/gif2.gif
    :width: 1000px
    :align: center

Run
-----

Source setup.bash on your workspace

    .. code-block:: bash

        # ~/dev_ws
        . install/setup.bash

Run the talker in one terminal:

    .. code-block:: bash

        # ~/dev_ws
        ros2 run cpp_pubsub talker


Then run listener in another terminal:

    .. code-block:: bash

        # ~/dev_ws
        ros2 run cpp_pubsub listener

The listener will start printing messages.
