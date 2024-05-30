.. _uses_cases_keys:

Running Turtle Nodes with a Qt application using keys
=====================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

This use case explains how to run three Turtle Nodes with a Qt application by leveraging topic keys and topic content filtering for efficient communication.

* **Topic keys** reduce the number of entities needed in the data flow, leading to more efficient resource and bandwidth usage.
* **Content Filter Topics** allow subscribers to receive only the messages that match specific criteria, ensuring each Turtle Node processes only relevant data.

By integrating topic keys and content filtering, each turtle node processes the data intended for it.
This setup illustrates efficient and effective communication in a complex system with multiple entities.

Users can control the turtles using the keyboard, while the Qt application visually represents their movements.
The Qt application terminal displays the turtles' positions, and the controller terminal shows the commands sent to each Turtle Node.
The controller receives the turtles position but it has subscriber filtered to show only the positions of turtles 1 and 2.
Each velocity and pose message is tagged with a key representing the turtle ID.

The following diagram illustrates the system architecture:

.. figure:: /rst/figures/use_cases/keys/keys.png
    :align: center

.. note::

    Topic keys work exclusively with publishers and subscribers, not with services.
    Multiple service clients can use the same service,
    but there can only be one service server for a given service.
    Since each Turtle Node must have a service server to control goals,
    it does not make sense to have multiple clients (e.g., controllers) using the same service server with different keys.


Prerequisites
-------------

Familiarity with the following topics is recommended:

* `Content Filter Topic <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/contentFilteredTopic/contentFilteredTopic.html>`__
* :ref:`topic_keys`

It is required to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`


Keyed Messages
--------------

The keyed Topics use the following message types to represent the turtles' velocity and pose:

.. raw:: html

    <div style="display: flex;">
        <div style="flex: 1; padding: 10px;">
            <p><b>docs_turtlesim::msg::KeyedPose</b></p>
            <p>Represents the pose of a turtle with a key.</p>
            <ul>
                <li>key: Identifier of the turtle.</li>
                <li>x: X-coordinate of the turtle's position.</li>
                <li>y: Y-coordinate of the turtle's position.</li>
                <li>theta: Orientation angle of the turtle.</li>
            </ul>
        </div>
        <div style="flex: 1; padding: 10px;">
            <img src="../../../_static/resources/use_cases/keys/keyed_pose.png" alt="KeyedPose" style="width: 220px;">
        </div>
    </div>

.. raw:: html

    <div style="display: flex;">
        <div style="flex: 1; padding: 10px;">
            <p><b>docs_turtlesim::msg::KeyedVelocity</b></p>
            <p>Represents the velocity of a turtle with a key.</p>
            <ul>
                <li>key: Identifier of the turtle.</li>
                <li>linear: Linear velocity of the turtle.</li>
                <li>angular: Angular velocity of the turtle.</li>
            </ul>
        </div>
        <div style="flex: 1; padding: 10px;">
            <img src="../../../_static/resources/use_cases/keys/keyed_twist.png" alt="KeyedTwist" style="width: 320px;">
        </div>
    </div>


Prepare the ROS 2 workspace
---------------------------

The ROS 2 package used in this use case is:

* `docs_turtlesim <https://github.com/eProsima/vulcanexus/tree/main/code/turtlesim>`__: a simple ROS 2 package for teaching ROS 2 concepts.

To create a new ROS 2 workspace and clone the `docs_turtlesim` package run:

.. code-block:: bash

    mkdir -p $HOME/ROS2-ws/src
    cd $HOME/ROS2-ws/src
    git clone --depth=1 https://github.com/eProsima/vulcanexus.git tmp_dir
    cd tmp_dir
    git archive --format zip --output ../turtlesim.zip HEAD:code/turtlesim
    cd ..
    unzip turtlesim.zip -d docs_turtlesim
    rm -rf tmp_dir turtlesim.zip

Build the ROS 2 workspace with:

.. code-block:: bash

    cd $HOME/ROS2-ws
    colcon build

The resulting directory structure should look like this:

.. code-block:: bash

    $HOME/ROS2-ws/
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
    source $HOME/ROS2-ws/install/setup.bash

Run Controller
^^^^^^^^^^^^^^

To start the turtlesim controller, run:

.. code-block:: bash

    ros2 run docs_turtlesim turtlesim_multi_teleop

Run Qt application
^^^^^^^^^^^^^^^^^^

To launch the multi-turtlesim Qt application, use this command:

.. code-block:: bash

    ros2 run docs_turtlesim turtlesim_multi_qt

Now you can control multiple turtles with the controller and see the pose of each turtle in the terminals, while the Qt application provides a graphical interface.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../_static/resources/use_cases/keys/keys.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>
