.. _uses_cases_rosbot_xl_autonomy:

Large data communication with ROS 2 (a practical example)
=========================================================

This use case explains how to use `ROSbot XL` with `Orbbec Astra` camera to perform autonomous navigation and mapping while taking images of the environment.
This tutorial serves to demonstrate the **new auto-configuration of Fast DDS**, the default communications middleware in ROS 2, **for sending large data** between multiple systems in a distributed environment.

As we will see throughout this case study, enabling large data mode in Vulcanexus is as simple as setting ``FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`` environment variable.

This command changes the default configuration of the transport layer in Fast DDS to better support the large data packages that the Orbbec Astra camera sends from the robot to the remote machine, among others.

.. raw:: html

    <video width=100% height=auto autoplay loop muted>
        <source src="../../../_static/resources/use_cases/large_data/demo.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>

Background
----------

In this demo we will use an `ROSbot XL` - an autonomous mobile robot platform by `Husarion <https://husarion.com/manuals/rosbot/>`_, equipped with a `RPLIDAR A2` laser scanner, and a `RGB-D Orbbec Astra`, to demonstrate the new Fast DDS builtin configuration to transmit large data.

The activation of this Fast DDS mode of operation is triggered by executing the following command before running any ROS 2 node.

.. code-block:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

If you want to know more about the changes made to the middleware, please refer to the `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/transport.html>`__.

The `ROSbot XL` is an affordable robot platform for rapid development of autonomous robots.
It can be a base for industrial robots, custom service robots, inspection robots and robots working in swarms.

.. figure:: /rst/figures/use_cases/rosbotxl/rosbotxl.png
    :align: center

`RViz2 <https://github.com/ros2/rviz>`_ is a 3D visualization tool developed for ROS 2.
It allows the display of ROS 2 Topics and the extension of its capabilities through the development of plugins.

.. figure:: /rst/figures/use_cases/rosbotxl/rviz.png
    :align: center

`rosbot_xl_ros <https://github.com/husarion/rosbot_xl_ros>`__ is the GitHub repository from Husarion where you can find the ROS 2 Launch files and configurations for the ROSbot XL.
All its contents are explained below.

.. figure:: /rst/figures/use_cases/rosbotxl/rosbot_xl_ros_repository.png
    :align: center

`sllidar_ros2 <https://github.com/husarion/rosbot_xl_ros>`__ is the GitHub repository from Husarion where you can find the ROS 2 Launch files and configurations for the RPLIDAR A2.
All its contents are explained below.

.. figure:: /rst/figures/use_cases/rosbotxl/sllidar_ros2_repository.png
    :align: center

`astra-docker <https://github.com/husarion/astra-docker/tree/ros2>`_ is the GitHub repository from Husarion where you can find the `Dockerfile <https://github.com/husarion/astra-docker/blob/ros2/Dockerfile>`_ to install and clone all the necessary packages to launch the Orbbec Astra camera in ROS 2.

.. figure:: /rst/figures/use_cases/rosbotxl/astra-docker_repository.png
    :align: center

`Nav2 <https://navigation.ros.org/>`__ is the ROS 2 Navigation Stack that provides a modular architecture for autonomous navigation in robots.
It includes packages for mapping, localization, path planning, and obstacle avoidance.

.. figure:: /rst/figures/use_cases/rosbotxl/nav2.png
    :align: center

`MicroXRCEAgent <https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html>`_ acts as a server between the DDS Network and eProsima Micro XRCE-DDS Clients applications.
It is part of the `Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/index.html>`_ stack, which is the default middleware for ``micro-ROS``.
It allows the ROS 2 stack to communicate with the micro-ROS stack.

.. figure:: /rst/figures/use_cases/rosbotxl/microxrceagent.png
    :align: center

Prerequisites
-------------

This use case assumes that you have a ROSbot XL equipped and ready to use with an Orbbec Astra camera.
If this is not your case, please refer to the `ROSbot XL Quick Start guide <https://husarion.com/manuals/rosbot/>`_ to set up your robot.
It is also necessary to have a machine with Vulcanexus installed.
Please refer to :ref:`Vulcanexus installation guide <linux_binary_installation>` to install Vulcanexus in your device.

Below are the steps to install the necessary packages and dependencies to run the demo.

#.  Clone the `rosbot_xl_ros <https://github.com/husarion/rosbot_xl_ros/tree/master>`__ repository from GitHub into your workspace directory using the following command:

    .. code-block:: bash

        git clone https://github.com/husarion/rosbot_xl_ros.git

#.  Add the `sllidar_ros2 <https://github.com/husarion/sllidar_ros2>`__ GitHub repository into your workspace directory with the following command:

    .. code-block:: bash

        git clone https://github.com/husarion/sllidar_ros2.git

#.  Follow the installation instructions provided on the `NAV 2 <https://navigation.ros.org/getting_started/index.html#installation>`__ website to install the ``nav2_bringup`` package.

#.  Please refer to the provided Dockerfile at `this link <https://github.com/husarion/astra-docker/blob/ros2/Dockerfile>`_ to install the ``astra_camera`` package.
    This Dockerfile facilitates the installation and cloning of all necessary packages required to launch the Orbbec Astra camera in ROS 2.

Ensure that all dependencies and prerequisites are met before proceeding further with the use case.

ROS 2 Packages
--------------

The ROS 2 Packages involve in the demo are the following:

*   `rosbot_xl_bringup <https://github.com/husarion/rosbot_xl_ros/tree/master/rosbot_xl_bringup>`_:
    a ROS 2 package that provides the ROS 2 launch files and configurations for the ROSbot XL.

*   `sllidar_ros2 <https://github.com/husarion/sllidar_ros2>`__:
    a ROS 2 package that provides the ROS 2 node for the RPLIDAR A2.

*   `astra_camera <https://github.com/rafal-gorecki/ros2_astra_camera/tree/bb20d81e09255356730ec39ce88250635185e895/astra_camera>`_:
    a ROS 2 package that provides the ROS 2 driver for Orbbec 3D cameras.

*   `nav2_bringup <https://github.com/ros-planning/navigation2/tree/humble/nav2_bringup>`_:
    a ROS 2 package that provides the ROS 2 launch files and configurations for the NAV 2 stack.

Execution
---------

The execution of the demo is divided into two parts:

#.  ROSbot XL bring-up.
#.  Launch the controller on the user computer.

ROSbot XL
^^^^^^^^^

First, it is necessary to setup the Vulcanexus environment and the ROS 2 workspace in all the terminals.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    source $HOME/ROS2-ws/install/setup.bash

Then set the environment variable ``FASTDDS_BUILTIN_TRANSPORTS`` to ``LARGE_DATA`` on all the terminals.
As mentioned before, this command changes the default configuration of the transport layer in Fast DDS to better support the large data packages that are going to be sent in this demo.

.. code-block:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

.. warning::

    Do not forget to run the above commands in all terminals to correctly setup Vulcanexus and ROS 2 to use the large data mode.

1. **Micro XRCE Agent**

    Launch the Micro XRCE Agent running the following command in a terminal.

    .. code-block:: bash

        MicroXRCEAgent udp4 --port 8888

2. **ROSbot XL Bringup**

    Then, launch the ROSbot XL Bringup running the following command in another terminal.
    The `mecanum` argument is optional and it is used to set the robot as a mecanum robot.

    .. code-block:: bash

        ros2 launch rosbot_xl_bringup bringup.launch.py mecanum:=True

3. **RPLIDAR A2**

    Launch the RPLIDAR A2, specifying the baud rate and port:

    .. code-block:: bash

        ros2 launch sllidar_ros2 sllidar_launch.py serial_baudrate:=115200 serial_port:=/dev/ttyRPLIDAR

4. **SLAM**

    In order to launch SLAM, run the following command.
    The `map` argument is optional and it is used to set the path of the map file.
    The `use_sim_time` argument is optional and it is used to set the use time provided by simulation.
    The `params_file` argument is optional and it is used to set the path of the parameters file.

    .. code-block:: bash

        ros2 launch nav2_bringup slam_launch.py map:=/maps/map.yaml use_sim_time:=False params_file:=/home/husarion/rosbot-xl-autonomy/config/nav2_rpp_params.yaml

5. **NAV 2**

    In order to launch the NAV 2 stack, run the following command.
    The `use_sim_time` argument is optional and it is used to set the use time provided by simulation.
    The `params_file` argument is optional and it is used to set the path of the navigation configuration file.

    .. code-block:: bash

        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/husarion/ros2_ws/config/nav2_rpp_params.yaml

6. **Orbbec Astra**

    Finally, launch the Orbbec Astra camera running the following command.

    .. code-block:: bash

        ros2 launch astra_camera astra_mini.launch.py

.. note::

    The ROS 2 launch files explained earlier are executed using a single ROS 2 launch file for ease of use.
    You can locate this launch file `here <https://github.com/eProsima/vulcanexus/tree/kilted/resources/py/rosbotxl_astra_navigation_mapping_launch.py>`__.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../_static/resources/use_cases/large_data/rosbot_terminals.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>

Now the ROSbot XL is ready to move around the environment and create a map!

External computer
^^^^^^^^^^^^^^^^^

First, it is necessary to load the Vulcanexus environment and the ROS 2 workspace.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    source $HOME/ROS2-ws/install/setup.bash

It is then necessary to set the ``FASTDDS_BUILTIN_TRANSPORTS`` environment variable to ``LARGE_DATA`` to change the default transport layer configuration to support the large data packets sent by the Orbbec Astra camera.

.. code-block:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

Finally, launch RViz2 and set the configuration file to visualize the robot and the environment.

.. code-block:: bash

    ros2 run rviz2 rviz2


.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../_static/resources/use_cases/large_data/laptop_terminals.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>

Now, the laptop is prepared to visualize the environment and robot movement.
You can navigate the environment, set goals, and generate maps effectively.

Troubleshooting
---------------

If Large Data mode of Fast DDS is not sufficient to see the image and navigation costmap correctly in RViz, it is possible that the limiting factor for communications is the machine on which we run ROS 2.
To improve communications, it is possible to apply some settings to each device running ROS 2, such as increasing the size of the system's network buffers.
To do this, run the following commands:

.. code-block:: bash

    sudo sysctl -w net.core.rmem_max=100000000
    sudo sysctl -w net.core.wmem_max=100000000


Next steps
----------

This tutorial has explained how to configure ROS 2 for large data transmission.
Stay tuned as this auto-configuration will be extended in future versions of Vulcanexus, offering more middleware configuration options via the same environment variable.
