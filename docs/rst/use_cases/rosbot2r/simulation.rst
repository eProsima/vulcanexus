.. _uses_cases_rosbot_2r_simulation:

Mapping Webots environment with ROSbot 2R teleoperation
=======================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

This document explain how to map a Webots environment with the teleoperation of a `ROSbot 2R` robot from Husarion.

In this demo we will use the simulation of `ROSbot 2R` - an autonomous mobile robot by `Husarion <https://husarion.com/manuals/rosbot/>`_, designed for learning ROS and for research and development purposes.
It is an affordable platform that can serve as a base for a variety of robotic applications, including inspection robots and custom service robots.
The robot features a solid aluminum frame and is equipped with a Raspberry Pi 4 with 4GB of RAM, distance sensors, an RPLIDAR A2 laser scanner, and an RGB-D Orbbec Astra camera.

.. figure:: /rst/figures/use_cases/rosbot2r/rosbot2r.png
    :align: center

`Webots` is an open-source three-dimensional mobile robot simulator.
It is the simulator selected for simulating the Husarion environment and the ROSbot.
It is based in Qt, a physics engine (ODE fork) and an OpenGL 3.3 rendering engine.
Please, refer to :ref:`Webots <webots>` for more information.

.. figure:: /rst/figures/use_cases/rosbot2r/webots.png
    :align: center

`RViz2 <https://github.com/ros2/rviz>`_ is a 3D visualization tool developed for ROS 2.
It allows the display of ROS 2 Topics and the extension of its capabilities through the development of plugins.

.. figure:: /rst/figures/use_cases/rosbot2r/rviz.png
    :align: center

`rosbot-mapping <https://github.com/husarion/rosbot-mapping>`_ is the GitHub repository from Husarion where you can find the Docker compose and configuration used in this demo.
All its contents are explained below.

.. figure:: /rst/figures/use_cases/rosbot2r/rosbot_mapping_repository.png
    :align: center

Prerequisites
-------------

This tutorial covers the first steps of setting up a ROSbot 2R simulation in Webots.
For this task, we will use the Docker Compose from the repository mentioned above.
Therefore, it is necessary to have Docker and Docker Compose installed.
If you do not have them installed, please follow `Install Docker on Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_ and `Getting Docker Compose <https://docs.docker.com/compose/install/>`_ installation guides.

High-performance computing is needed for the simulation.
It is necessary to process such a large amount of data and to perform complex calculations at high speed.
This requires to use NVIDIA Container Runtime.
Ensure that you have NVIDIA GPU and NVIDIA Container Toolkit installed, otherwise, follow the installation steps `here <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html>`__.

Add the `rosbot-mapping <https://github.com/husarion/rosbot-mapping>`_  GitHub repository into your workspace directory with the following command:

.. code-block:: bash

    git clone https://github.com/husarion/rosbot-mapping

ROS 2 Packages
--------------

The ROS 2 Packages involve in the demo are the following:

* `webots_ros2_husarion <https://github.com/husarion/webots_ros2/tree/master/webots_ros2_husarion/webots_ros2_husarion>`_
    It is a ROS 2 package prepared to start Webots with Husarion environment.

* `slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`_
    It is a ROS 2 package that provide a set of tools for 2D Simultaneous Localization and Mapping (SLAM).

* `nav2_map_server <https://github.com/ros-planning/navigation2/tree/main/nav2_map_server>`_
    It is a ROS 2 package from Navigation that provides maps build with the information of the sensors, in this case, the RPLIDAR A2.

* `teleop_twist_keyboard <https://github.com/ros2/teleop_twist_keyboard>`_
    It is a ROS 2 package that provides a node which takes keypresses from the keyboard and publishes them as ``Twist`` messages.

Environment review
------------------

The Docker Compose used for the simulation is ``compose.sim.webots.yaml``.
You can find it `here <https://github.com/husarion/rosbot-mapping/blob/master/compose.sim.webots.yaml>`__.

The Docker Compose launches the following containers:

* **rviz**: is the container that is responsible for starting up RViz2 with the appropriate configurations.

* **rosbot**: start the environment and the robot in Webots.

* **mapping**: will be responsible for locating the robot and creating a map using the odometry and LIDAR data received from the sensors.

* **map-saver**: is responsible for the storage of the map that was previously created.

The following lines apply to all containers and mean that everything is running in host.

.. code-block:: yaml

    x-net-config:
        &net-config
        network_mode: host
        ipc: host
        env_file: net.env

Below are the GPU configurations that apply to ``RViz2`` and ``Webots``, as they are graphical applications.

.. code-block:: yaml

    x-gpu-config:
        &gpu-config
        runtime: nvidia
        environment:
            - DISPLAY=${DISPLAY:?err}
            - NVIDIA_VISIBLE_DEVICES=all
            - NVIDIA_DRIVER_CAPABILITIES=all

Then start the container definition.

The first thing you can see is the ``rviz`` container, which is started with the configuration file found `here <https://github.com/husarion/rosbot-mapping/blob/master/config/rosbot.rviz>`__.
The plugins used are defined in the configuration file.
Among them, for example, we find the plugin from the navigation stack ``2D Goal Pose``, which allows the user to send a target by setting a desired pose for the robot to achieve.
For the purposes of this tutorial, however, we need only pay attention to the ``Displays`` and familiarise ourselves with the information they provide.
Between them you can see the display of the map created in ``static_map`` and the information from the LIDAR in ``scan``.

.. code-block:: yaml

    services:

        rviz:
            image: husarion/rviz2:vulcanexus-humble-11.2.5-20230308
            <<: [ *net-config, *gpu-config ]
            container_name: rviz
            volumes:
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./config/rosbot.rviz:/root/.rviz2/default.rviz

The following service defines the ``rosbot`` container.
It launches webots with the robot and environment of choice.
It also starts the nodes responsible for publishing the robot's state, as ``robot_localisation`` or ``robot_state_publisher``.



.. code-block:: yaml

        rosbot:
            image: husarion/webots:vulcanexus-humble-2023.0.1-20230301
            <<: [ *net-config, *gpu-config ]
            volumes:
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            command: ros2 launch webots_ros2_husarion robot_launch.py robot_name:=rosbot

Next, the ``mapping`` container will start up the slam_toolbox node with the configuration file, which can be found `here <https://github.com/husarion/rosbot-mapping/blob/master/config/slam_toolbox_webots.yaml>`__.
``use_sim_time`` set to ``True`` define that must use Webots clock.

.. code-block:: yaml

        mapping:
            image: husarion/slam-toolbox:vulcanexus-humble-2.6.4-20230228
            <<: *net-config
            volumes:
            - ./config/slam_toolbox_webots.yaml:/slam_params.yaml
            command: >
            ros2 launch slam_toolbox online_sync_launch.py
                slam_params_file:=/slam_params.yaml
                use_sim_time:=True

Finally, the ``map-server`` container is responsible for saving the map created by the previous container every 5 seconds.
It will store the map in your ``./maps`` directory.

.. code-block:: yaml

        map-saver:
            image: husarion/nav2-map-server:vulcanexus-humble-1.1.5-20230228
            <<: *net-config
            volumes:
            - ./maps:/maps
            command: bash -c "while true; do ros2 run nav2_map_server map_saver_cli --free 0.15 --fmt png -f /maps/map; sleep 5; done"

Execution
---------

First, it is necessary to launch the docker compose ``compose.sim.webots.yaml`` that will activate the containers ``rviz``, ``rosbot``, ``mapping`` and ``map-saver``.

Start the containers in a new terminal:

.. code-block:: bash

    xhost +local:docker
    docker compose -f compose.sim.webots.yaml up

.. figure:: /rst/figures/use_cases/rosbot2r/step_1.png
    :align: center

In order to teleoperate the ROSbot with the keyboard, launch the teleoperation node, ``teleop_twist_keyboard``, inside the ``rviz`` docker container previously created.
Use the commands below in a second terminal:

.. code-block:: bash

    docker exec -it rviz bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

.. figure:: /rst/figures/use_cases/rosbot2r/step_2.png
    :align: center

Now you can move around the environment with the keyboard and create a map!

.. figure:: /rst/figures/use_cases/rosbot2r/webots_rviz_map.png
    :align: center
