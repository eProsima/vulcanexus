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

Writing a simple publisher and subscriber build it using Colcon.

Nodes are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.


Create a workspace
--------------------

A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

    .. code-block:: bash

        source /opt/ros/humble/setup.bash

        # Best practice is to create a new directory for every new workspace.
        mkdir -p ~/dev_ws/src
        # Another best practice is to put any packages in your workspace into the src directory.
        cd ~/dev_ws/src

Create a package
----------------


Recall that packages should be created in the src directory, not the root of the workspace. So, navigate into dev_ws/src, and run the package creation command:

    .. code-block:: bash

        ros2 pkg create --build-type ament_cmake cpp_pubsub

        # Recall that this is the directory in any CMake package where the source files containing executables belong
        cd cpp_pubsub/src
    
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

Create publisher_member_function.cpp
------------------------------------

    .. code-block:: bash

        # on ~/dev_ws/srccpp_pubsub/src
        touch publisher_member_function.cpp

    .. code-block:: text

        cpp_pubsub/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include
        │   └── cpp_pubsub
        └── src
            └── publisher_member_function.cpp

    .. literalinclude:: ../../../../resources/examples/core/ros2_pub_sub/src/publisher_member_function.cpp
        :language: C++
        :linenos:

Create subscriber_member_function.cpp
-------------------------------------

    .. code-block:: bash

        # on ~/dev_ws/srccpp_pubsub/src
        touch publisher_member_function.cpp

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

Add a new line after the `ament_cmake buildtool` dependency and paste the following dependencies corresponding to your node’s include statements:

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

You likely already have the rclcpp and std_msgs packages installed as part of your ROS 2 system. It’s good practice to run rosdep in the root of your workspace (dev_ws) to check for missing dependencies before building:

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

source setup.bash on your workspace
    
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

The listener will start printing messages to the console.