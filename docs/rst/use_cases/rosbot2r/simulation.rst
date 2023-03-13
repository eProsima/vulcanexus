.. _uses_cases_rosbot_2r_simulation:

Mapping Webots environment with ROSbot2R teleoperation
======================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

This document explain how to map a Webots environment with the teleoperation of a `ROSbot2R` robot from Husarion.

Prerequisites
-------------

It is necessary to have Docker and Docker Compose installed. If you do not have them installed, please follow `Install Docker on Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_ and `Getting Docker Compose <https://docs.docker.com/compose/install/>`_.

It uses NVIDIA container Runtime, ensure thar you have NVIDIA GPU and NVIDIA Container Toolkit installed, otherwise, follow the installation steps `here <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html>`_.

`rosbot-mapping <https://github.com/husarion/rosbot-mapping>`_ is the GitHub repository from Husarion where can be found the docker compose and configuration used in this demo.
Clone the repository in you workspace with the following command:

.. code-block:: bash

    git clone https://github.com/husarion/rosbot-mapping

ROSbot2R
--------

`ROSbot2R` simulation is the robot used in this demo. It is a robot platform from `Husarion <https://husarion.com/manuals/rosbot/>`_ powered by ROS with a Raspberry Pi 4.
It is an autonomous mobile robot with sensors as RPLIDAR A2 laser scan, RGB-D Orbbec Astra camera or IMU.
It is an affordable robot for development and can be used for service or inspection robots among others.

.. figure:: /rst/figures/use_cases/rosbot2r/rosbot2r.png
    :align: center

Webots
------

`Webots` is an open-source three-dimensional mobile robot simulator.

Please, refer to :ref:`Webots <webots>` for more information.

.. figure:: /rst/figures/use_cases/rosbot2r/webots.png
    :align: center

RViz2
-----

`RViz2 <https://github.com/ros2/rviz>`_ is a 3D visualization tool developed for ROS 2.

.. figure:: /rst/figures/use_cases/rosbot2r/rviz.png
    :align: center

ROS 2 Nodes
-----------

The ROS  2 Nodes involve in the demo are the following:

* `slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`_
    It is a ROS 2 package that provide a set of tools for 2D Simultaneous Localization and Mapping (SLAM).

* `nav2_map_server <https://github.com/ros-planning/navigation2/tree/main/nav2_map_server>`_
    It is a ROS 2 package from Navigation that provides maps build with the information of the sensors, in this case, the RPLIDAR A2.

* `teleop_twist_keyboard <https://github.com/ros2/teleop_twist_keyboard>`_
    It is a ROS 2 package that provides a node which takes keypresses from the keyboard and publishes them as ``Twist`` messages.

Execution
---------

First, it is necessary to launch a docker compose ``compose.sim.webots.yaml`` that will activate a container for ``RViz2``, ``Webots``, ``SLAM`` and ``nav2_map_server``.

Start the containers in a new terminal:

.. code-block:: bash

    xhost +local:docker
    docker compose -f compose.sim.webots.yaml up

.. figure:: /rst/figures/use_cases/rosbot2r/step_1.png
    :align: center

In order to teleoperate the ROSbot with the keyboard, launch the teleoperation node, ``teleop_twist_keyboard``, inside the ``RViz2`` docker container previously created. Use the commands below in a second terminal:

.. code-block:: bash

    docker exec -it rviz bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

.. figure:: /rst/figures/use_cases/rosbot2r/step_2.png
    :align: center

Now you can move around the environment with the keyboard and create a map!

.. figure:: /rst/figures/use_cases/rosbot2r/webots_rviz_map.png
    :align: center

.. note::

    This execution follow the steps explained in the `GitHub repository <https://github.com/husarion/rosbot-mapping>`_ from Husarion.
