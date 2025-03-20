.. include:: ../../../exports/alias.include

.. _tutorials_cloud_wan_edge_cloud_wan_edge_cloud:


Edge-Cloud communication on WAN
===============================

Background
----------

This tutorial will cover the first steps to setup a distributed network of remotely controlled robots from the Cloud or an edge device.
More specifically, it will focus on a basic edge-cloud architecture in which there is an edge robot deployed on a :term:`LAN` with access to the Internet, and a server in the Cloud reachable through the Internet.

.. _warning_lan:

.. warning::

    This tutorial is intended for :term:`WAN` communication.
    However, if communication through a LAN is your only option, it is still possible to follow the tutorial by changing the ROS 2 Domain Ids so that each ROS 2 node uses a different Domain (``0`` and ``1``).
    This way the ROS 2 nodes are logically isolated and will not discover other nodes out of their ROS 2 Domain.

Following, all the elements involved in this architecture will be studied, starting with the edge robot, continuing with the controller hosted in the cloud also built as a ROS 2 node and concluding with the intermediate elements that enable communication over the Internet.

The image below describes the scenario presented in this tutorial.

.. figure:: /rst/figures/tutorials/cloud/edge_cloud_wan.png
   :align: center

Several key elements can be observed in it:

#.  **ROS 2 Application**.
    *Turtlesim* is the application used for this tutorial.
    *Turtlesim* is a ROS 2 application, first developed for ROS, aimed at teaching the basic concepts of ROS 2 such as publish/subscribe, services and actions.
    The edge robot will then be a ``turtlesim_node``, which is a simulator of a robot making use of these communication methods.

#.  **ROS 2 Device Controller**.
    This is a ROS 2 application that sends commands to the edge robot.
    A basic C++ application has been developed for this tutorial that sends publications under the topic on which the ``turtlesim_node`` listens.
    By means of these publications (commands) from the controller, and the feedback information that the controller receives from the ``turtlesim_node``, it is possible to control this node automatically without the need for user intervention which facilitates the deployment of the scenario at hand.
    The key feature of the *DDS Router* is that it is easy to configure, allowing to connect different networks with ROS 2 applications without requiring to apply any changes to the developer's software or applications.

#.  **ROS 2 Router / DDS Router**.
    *eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see DDS Router documentation `here <https://eprosima-dds-router.readthedocs.io/en/latest/>`_).
    That is, ROS 2 nodes such as publishers and subscribers, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic areas on their own dedicated local networks as if they were all on the same network through the use of *DDS Router*.

    This example presents two routers that enable Internet communication:

    * *DDS Router Edge*. This is the DDS Router that is deployed on the edge robot side. This way it is possible for the robot to communicate out-of-the-box with an external server.
    * *DDS Router Cloud*. It plays the server role in the communication. It will expose a public network address to which the nodes connect to establish communication.


Prerequisites
-------------

This tutorial will require two machines (*Robot 1* and *Cloud Server*) deployed on different networks (*LAN 1* and *Cloud*).
It is possible to simulate the scenario by deploying everything needed on the same machine and two virtual networks but let's focus on the case of a real deployment.

It is also necessary to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Deployment on LAN 1
-------------------

First, let's deploy the ``turtlesim_node`` and DDS Router Edge on a machine on *LAN 1*.

Running turtlesim_node on the edge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Setup the Vulcanexus environment to have the ``turtlesim_node`` available.
For this, there are two possible options:

#.  Running the Vulcanexus Docker image.

    Run the Vulcanexus Docker image with:

    .. code-block:: bash

        xhost local:root
        docker run \
            -it \
            --privileged \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            ubuntu-vulcanexus:jazzy-desktop

    Then, within the container, source the Vulcanexus installation with:

    .. code-block:: bash

            source /opt/vulcanexus/jazzy/setup.bash

#.  Setting up the development environment on the local host. For this second option, it is necessary to have installed the ``vucanexus-jazzy-desktop`` package, since this is the one that includes all the simulation tools, demos and tutorials.

    Source the following file to setup the Vulcanexus environment:

    .. code-block:: bash

            source /opt/vulcanexus/jazzy/setup.bash

Once the environment has been setup using one of the above options, simply run the ``turtlesim_node``.

.. code-block:: bash

    ros2 run turtlesim turtlesim_node

And a popup window like the following should appear:

.. figure:: /rst/figures/tutorials/cloud/turtlesim_node.png
   :scale: 75%
   :align: center

As can be seen, it is not necessary to perform any additional configuration in the ROS 2 application.


Running DDS Router Edge
^^^^^^^^^^^^^^^^^^^^^^^

Then, to run the DDS Router Edge configure the environment as in the previous step.

.. note::

    If deploying Vulcanexus from the Docker image, note that you will need to have a configuration file (``config.yaml``) for the DDS Router Edge accessible from your Docker container.

    This can be achieved by mounting a shared volume when launching the container, by copying the file from the local host to the container in case it is already running, or by editing a file from the Docker container itself.

Setup the Vulcanexus environment, either in a Docker container or on the local host, running the following command:

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash

Let's create a DDS Router configuration file as the one shown below.

.. literalinclude:: /resources/tutorials/cloud/wan_edge_cloud/dds_router_edge.yaml
    :language: yaml

Next, the most relevant aspects of this configuration file are explained.

The ``participants`` are the interfaces of the DDS Router to communicate with other networks. In this case, we have two kinds of participants:

    *   ``local``: this is a simple participant that communicates with all ROS 2 nodes it finds.
        For more information about this participant please refer to the `Simple Participant section <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/participants/simple.html#user-manual-participants-simple>`_ of the DDS Router documentation.

    *   ``wan``: it is a participant designed for the communication between two *DDS Routers*.
        It uses the |InitialPeersFastDdsDocs| to establish a point-to-point communication between two DDS entities, two *DDS Routers* in this case.

    For the DDS Router Edge, a connection address shall be defined which must be the same as the one exposed by the Cloud Server.

.. note::

    In this case, the DDS Router will forward all topics found in the network.
    However, it is important to mention that the ROS 2 topics relayed by the DDS Router can be filtered by configuring the ``allowlist`` and ``blocklist``.
    If this is the case please refer to the `DDS Router documentation <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/configuration.html#topic-filtering>`_ for information on how to do this.

The following figure summarizes the deployment on the edge.

.. figure:: /rst/figures/tutorials/cloud/edge_deployment.png
   :align: center

Now, run the DDS Router with the configuration file created as an argument.

.. code-block::

    ddsrouter -c <path/to/file>/ddsrouter_edge.yaml

Deployment on Cloud
-------------------

Running the turtlesim_square_move on the Cloud
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the ``turtlesim_square_move`` in the Cloud Server machine, which is the controller of the edge ``turtlesim_node``.
This will send commands to the ROS 2 application to the edge to move the turtle and receive information about the current state of the turtle at any time.

A ROS 2 application that moves the turtle by drawing a square has been developed for this purpose.
The application is based on the `ROS 2 tutorials <https://github.com/ros/ros_tutorials/tree/jazzy/turtlesim>`_, but has been slightly modified in order to make it easier to understand and adapt to the ROS 2 and modern C++ programming methods.

Then, start by creating the workspace of this application and downloading the source code:

.. code-block:: bash

    mkdir -p turtlesim_move_ws/src && cd turtlesim_move_ws/src
    git clone --branch jazzy https://github.com/eProsima/vulcanexus.git
    mv vulcanexus/code/turtlesim .
    rm -rf vulcanexus
    cd ..

Once created the workspace, source the Vulcanexus environment and build the ``turtlesim_square_move`` application.

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash
    colcon build

.. note::

    Since the purpose of this tutorial is not to explain how to create a ROS 2 node, but rather, the communication of these in distributed environments, the code will not be discussed in detail.
    Stay tuned for new tutorials in which we will discuss how to configure ROS 2 nodes, publishers/subscribers, services and actions.

Then source the ``turtlesim_move_ws`` workspace:

.. code-block:: bash

    source turtlesim_move_ws/install/setup.bash

And finally, run the application:

.. tabs::

    .. tab:: WAN

        .. code-block:: bash

            ros2 run turtlesim turtlesim_square_move

    .. tab:: LAN

        .. code-block:: bash

            ROS_DOMAIN_ID=1 ros2 run turtlesim turtlesim_square_move

        .. note::

            As stated :ref:`here <warning_lan>`, change the ROS 2 Domain Id if running the edge and cloud applications on the same LAN.


The important points to note in this application are the following:

*   The control application sends the movement commands to the ``turtlesim_node`` through a publisher in the `geometry_msgs/msg/Twist <https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html>`_ topic.
    This topic expresses the velocity at which the turtle has to move divided into linear velocity (``linear``) and angular velocity (``angular``).

*   The application knows the position of the turtle on the map at any moment and whether it is stopped or moving by subscribing to the `turtlesim/msg/pose <https://docs.ros2.org/foxy/api/turtlesim/msg/Pose.html>`_ topic.
    This topic provides information about the turtle's coordinates (``x`` and ``y``) and the turtle's rotation (``theta``). We can also know its linear and angular velocity (``linear_velocity`` and ``angular_velocity``).


Running the DDS Router Cloud
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _tutorials_cloud_wan_edge_cloud_wan_edge_cloud_configure_transversal_nat:

Configure transversal NAT on the network router
'''''''''''''''''''''''''''''''''''''''''''''''

The first thing to do before starting to configure DDS Router is to configure the network router to allow a remote communication from the Internet to reach a specific device on the LAN, more specifically to expose an IP address and a port to the network that will be used by our DDS Router application.

This configuration will depend on your network router, but it should be similar to the one shown in the following image.

.. figure:: /rst/figures/tutorials/cloud/router_settings.png
   :align: center

.. warning::

    Due to a current limitation of DDS Router, the external port and internal port must match.
    Stay tuned for new versions of DDS Router that are intended to address this limitation.

Configure the DDS Router Cloud
''''''''''''''''''''''''''''''

The DDS Router Cloud configuration file is quite similar to the DDS Router Edge configuration file, as can be seen below:

.. tabs::

    .. tab:: WAN

        .. literalinclude:: /resources/tutorials/cloud/wan_edge_cloud/dds_router_cloud_wan.yaml
            :language: yaml

    .. tab:: LAN

        .. literalinclude:: /resources/tutorials/cloud/wan_edge_cloud/dds_router_cloud_lan.yaml
            :language: yaml

        .. note::

            As stated :ref:`here <warning_lan>`, set the ROS 2 Domain Id on the ``local`` participant in order to discover the ``turtlesim_square_move`` ROS 2 node.


In this case there are also two participants, two communication interfaces for the DDS Router.
The first one communicates the DDS Router with any ROS 2 node, while the second one enables to establish a communication channel with another DDS Router.

Although quite similar to the WAN participant in the DDS Router Edge instance, notice that this participant sets a listening address (``listening-addresses``), rather than a connection address.
This is because it is the participant that waits for incoming communications since it has this network address exposed and accessible from the Internet.

To finish, as done in the previous steps, setup the Vulcanexus environment sourcing the `setup.bash` file and run the DDS Router Cloud with the above configuration.

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash
    ddsrouter -c <path/to/file>/ddsrouter_cloud.yaml

The following figure summarizes the deployment on the Cloud.

.. figure:: /rst/figures/tutorials/cloud/cloud_deployment.png
   :align: center


Results
-------

If all the steps in this tutorial have been followed, the turtle in the ``turtlesim_node`` on the edge should move around creating a square,

.. figure:: /rst/figures/tutorials/cloud/turtlesim_node_square.png
   :scale: 75%
   :align: center

and the ``turtlesim_square_move`` should prompt the following traces

.. code-block:: bash

    root@dbf79a437eb3:/turtlesim_move_ws# ros2 run turtlesim turtlesim_square_move
    [INFO] [1657870899.585667136] [turtlesim_square_move]: New goal [7.544445 5.544445, 0.000000]
    [INFO] [1657870901.585656311] [turtlesim_square_move]: Reached goal
    [INFO] [1657870901.585767260] [turtlesim_square_move]: New goal [7.448444 5.544445, 1.570796]
    [INFO] [1657870905.685637930] [turtlesim_square_move]: Reached goal
    [INFO] [1657870905.685753714] [turtlesim_square_move]: New goal [7.466837 7.544360, 1.561600]
    [INFO] [1657870907.885655744] [turtlesim_square_move]: Reached goal
    [INFO] [1657870907.885742857] [turtlesim_square_move]: New goal [7.466837 7.544360, 3.132396]
    [INFO] [1657870911.985655175] [turtlesim_square_move]: Reached goal
    [INFO] [1657870911.985738726] [turtlesim_square_move]: New goal [5.467175 7.581143, 3.123200]
    [INFO] [1657870914.085652821] [turtlesim_square_move]: Reached goal

Next steps
----------

Feel free to read the following tutorials extending this one to similar scenarios:

* :ref:`Edge-Cloud TLS communication on WAN <tutorials_cloud_wan_edge_cloud_tls_wan_edge_cloud_tls>`: secure Edge-Cloud communication channel by using TLS protocol.

.. todo:

    * :ref:`Edge-Edge communication via repeater <tutorials_cloud_edge_edge_repeater_cloud_edge_edge_repeater_cloud>`: communicate two edge devices distantly located be means of a third bridging component hosted in the cloud.
