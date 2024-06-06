.. _uses_cases_keys:

Keyed Topics in ROS 2 Turtlesim Demo
====================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

This use case explains how to run three turtlesim nodes by leveraging topic keys and content filtering for efficient communication.

* **Topic keys** reduce the number of entities needed in the data flow, leading to more efficient resource and bandwidth usage.
* **Content Filter Topics** allow subscribers to receive only the messages that match specific criteria, ensuring each Turtle Node processes only relevant data.

By integrating topic keys and content filtering, each turtle node processes the data intended for it.
This setup illustrates efficient and effective communication in a complex system with multiple entities.

Users can control the turtles using the keyboard, while the Qt application visually represents their movements.
The Qt application terminal displays the turtles' positions, and the controller terminal shows the commands sent to each Turtle Node.
The controller receives the turtles position.
Each velocity and pose message is tagged with a key representing the turtle ID.

The diagram below shows the entities required using topic keys:

.. figure:: /rst/figures/use_cases/keys/keys.png
    :align: center

The diagram below shows the entities required **without** using topic keys:

.. figure:: /rst/figures/use_cases/keys/non-keys.png
    :align: center

.. note::

    Topic keys work exclusively with publishers and subscribers, not with services.
    Multiple service clients can use the same service, but there can only be one service server for a given service.
    Since each Turtle Node must have a service server to control goals, it does not make sense to have multiple clients (e.g., controllers) using the same service server with different keys.


Prerequisites
-------------

Familiarity with the following topics is recommended:

* :ref:`topic_keys`
* `Content Filter Topic <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/contentFilteredTopic/contentFilteredTopic.html>`__

It is required to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`


Keyed Messages
--------------

The keyed Topics use the following message types to represent the turtles' velocity and pose:

``docs_turtlesim::msg::KeyedPose`` Represents the pose of a turtle with a key.

- `key:` Identifier of the turtle.
- `x:` X-coordinate of the turtle's position.
- `y:` Y-coordinate of the turtle's position.
- `theta:` Orientation angle of the turtle.
- `linear_velocity:` Linear velocity of the turtle.
- `angular_velocity:` Angular velocity of the turtle.

.. code-block:: bash

    // KeyedPose.idl
    module docs_turtlesim {
        module msg {
            struct KeyedPose {
                @key long key;
                double x;
                double y;
                double theta;

                double linear_velocity;
                double angular_velocity;
            };
        };
    };

``docs_turtlesim::msg::KeyedVelocity`` Represents the velocity of a turtle with a key.

- `key:` Identifier of the turtle.
- `linear:` Linear velocity of the turtle.
- `angular:` Angular velocity of the turtle.

.. code-block:: bash

    // KeyedTwist.idl
    module docs_turtlesim {
        module msg {
            struct KeyedTwist {
                @key long key;
                docs_turtlesim::msg::Vector3  linear;
                docs_turtlesim::msg::Vector3  angular;
            };
        };
    };


Prepare the ROS 2 workspace
---------------------------

The ROS 2 package used in this use case is:

* `docs_turtlesim <https://github.com/eProsima/vulcanexus/tree/main/code/turtlesim>`__: a simple ROS 2 package for teaching ROS 2 concepts.

To create a new ROS 2 workspace and clone the `docs_turtlesim` package run:

.. code-block:: bash

    mkdir -p $HOME/ros2-ws/src
    cd $HOME/ros2-ws/src
    git clone --depth=1 https://github.com/eProsima/vulcanexus.git tmp_dir
    cd tmp_dir
    cp -r code/turtlesim ..
    cd ..
    rm -rf tmp_dir

Build the ROS 2 workspace with:

.. code-block:: bash

    cd $HOME/ros2-ws
    colcon build

The resulting directory structure should look like this:

.. code-block:: bash

    $HOME/ros2-ws/
    ├── build
    ├── install
    ├── log
    └── src
        └── docs_turtlesim

Execution
---------

First, it is necessary to setup the Vulcanexus environment and the ROS 2 workspace in all the terminals.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    source $HOME/ros2-ws/install/setup.bash

Run the controller application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To start the turtlesim controller, run:

.. code-block:: bash

    ros2 run docs_turtlesim turtlesim_multi_control

Run Turtlesim Nodes with Key
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To launch the turtlesim application with three turtle nodes, use this command:

.. code-block:: bash

    ros2 run docs_turtlesim turtlesim_node_keys

Now you can control multiple turtles with the controller and see the pose of each turtle in the terminals.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../_static/resources/use_cases/keys/keys.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>
