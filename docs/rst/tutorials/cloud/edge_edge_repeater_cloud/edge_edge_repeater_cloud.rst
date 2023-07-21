.. include:: ../../../exports/alias.include

.. _tutorials_cloud_edge_edge_repeater_cloud_edge_edge_repeater_cloud:


Edge-Edge communication via Repeater
====================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

This tutorial will move one step further from the previous tutorial :ref:`tutorials_cloud_wan_edge_cloud_wan_edge_cloud`.
It is recommended to follow these tutorials in order, as some concepts or installations may be already covered.
It will focus on an edge-edge architecture in which both edge robots are deployed on different networks (WAN) with access to the Internet,
and communicate through a server hosted in the Cloud.

The scenario we are considering in this tutorial is the one where the edges are not directly connected
(are under different LANs) and each robot does not have access to the other's location and network
(as they may be behind intermediate NATs or be part of dynamic networks).
In this scenario, the Cloud server will work as a **TURN** (Traversal Using Relays around NAT), a.k.a **Repeater**.
This kind of servers are meant to be accessed from every point in the WAN network, and forward the messages received from
any edge to the rest of devices connected to the server.
Thus, it creates a bridge between 2 networks that do not have direct access to each other,
facilitating network configuration aspects such as NATs Traversals and dynamic addresses.

.. _warning_lan_edge_edge:

.. warning::

    This tutorial is intended for WAN communication.
    However, if communication through a LAN is your only option, it is still possible to follow the tutorial by changing the ROS 2 Domain Ids so that each edge uses a different Domain (``0`` and ``1``).
    This way the ROS 2 nodes are logically isolated and will not discover other nodes out of their ROS 2 Domain.

Following, all the elements involved in this architecture will be studied, starting with the edge robots and continuing with the intermediate elements that enable communication over the Internet between each edge and the Cloud.
One edge will work as a controller and the other as a robot (as already explained in :ref:`previous tutorial <tutorials_cloud_wan_edge_cloud_wan_edge_cloud>`).

The image below describes the scenario presented in this tutorial.

.. figure:: /rst/figures/tutorials/cloud/repeater_wan.png
   :align: center

Several key elements can be observed in it:

#.  **ROS 2 Application**.
    *Turtlesim* is the application used for this tutorial.
    *Turtlesim* is a ROS 2 application, first developed for ROS, aimed at teaching the basic concepts of ROS 2 such as publish/subscribe, services and actions.
    The edge robot will then be a ``turtlesim_node``, which is a simulator of a robot making use of these communication methods.

#.  **ROS 2 Controller**.
    A basic C++ application has been developed for this tutorial that sends publications under the topic on which the ``turtlesim_node`` listens.
    It has been developed a basic C++ application for this tutorial that sends publications on the topic that the ``turtlesim_node`` listens.
    By means of these publications (commands) from the controller, and the feedback information that the controller receives from the ``turtlesim_node``,
    it is possible to control this node automatically without the need for user intervention which facilitates the deployment of the scenario at hand.
    The key feature of the *DDS Router* is that it is easy to configure, allowing to connect different networks with ROS 2 applications without requiring to apply any changes to the developer's software or applications.

#.  **ROS 2 Router / DDS Router**.
    *eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see DDS Router documentation `here <https://eprosima-dds-router.readthedocs.io/en/latest/>`_).
    That is, ROS 2 nodes such as publishers and subscribers, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic areas on their own dedicated local networks as if they were all on the same network through the use of *DDS Router*.

    This example presents two routers that enable Internet communication:

    * *DDS Router Edge*. This is the DDS Router that is deployed on the edge robot side. This way it is possible for the robot to communicate out-of-the-box with an external server.
    * *DDS Router Repeater*. It plays the TURN server role in the communication. It will expose some public network addresses to which the edge Routers could establish communication, and will forward the messages from one LAN to the other.


Prerequisites
-------------

This tutorial will require three machines (*Robot 1*, *Controller 1* and *Cloud Server*) deployed on different networks (*LAN 1*, *LAN 2* and *Cloud*).
It is possible to simulate the scenario by deploying everything needed on the same machine and three virtual networks but let's focus on the case of a real deployment.

It is also necessary to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Deployment on LAN 1
-------------------

First, let's deploy the ``turtlesim_node`` and DDS Router Edge on a machine on *LAN 1*.

Running turtlesim_node on Edge 1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
            ubuntu-vulcanexus:iron-desktop

    Then, within the container, source the Vulcanexus installation with:

    .. code-block:: bash

            source /opt/vulcanexus/iron/setup.bash

#.  Setting up the development environment on the local host. For this second option, it is necessary to have installed the ``vucanexus-iron-desktop`` package, since this is the one that includes all the simulation tools, demos and tutorials.

    Source the following file to setup the Vulcanexus environment:

    .. code-block:: bash

            source /opt/vulcanexus/iron/setup.bash

Once the environment has been setup using one of the above options, simply run the ``turtlesim_node``.

.. code-block:: bash

    ros2 run turtlesim turtlesim_node

And a popup window like the following should appear:

.. figure:: /rst/figures/tutorials/cloud/turtlesim_node.png
   :scale: 75%
   :align: center

As can be seen, it is not necessary to perform any additional configuration in the ROS 2 application.


.. _tutorials_cloud_edge_edge_repeater_cloud_edge_edge_repeater_cloud_running_router_1:

Running DDS Router Edge 1
^^^^^^^^^^^^^^^^^^^^^^^^^

Then, to run the DDS Router Edge configure the environment as in the previous step.

.. note::

    If deploying Vulcanexus from the Docker image, note that you will need to have a configuration file (``config.yaml``) for the DDS Router Edge accessible from your Docker container.

    This can be achieved by mounting a shared volume when launching the container, by copying the file from the local host to the container in case it is already running, or by editing a file from the Docker container itself.

Setup the Vulcanexus environment, either in a Docker container or on the local host, running the following command:

.. code-block:: bash

    source /opt/vulcanexus/iron/setup.bash

Let's create a the DDS Router configuration file as the one shown below.

.. literalinclude:: /resources/tutorials/cloud/edge_edge_repeater_cloud/dds_router_edge_1.yaml
    :language: yaml

Next, the most relevant aspects of this configuration file are explained.

The ``participants`` are the interfaces of the DDS Router to communicate with other networks. In this case, we have two kinds of participants:

    *   ``local``: this is a simple participant that communicates with all ROS 2 nodes it finds in domain 0.
        For more information about this participant please refer to the `Simple Participant section <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/participants/simple.html#user-manual-participants-simple>`_ of the DDS Router documentation.

    *   ``wan``: it is a participant designed to communicate with a WAN Participant configured as server (repeater in this case).
        It uses the |InitialPeersFastDdsDocs| to establish a point-to-point communication between two DDS entities, two *DDS Routers* in this case.

    For the DDS Router Edge, a connection address shall be defined which must be the same as the one exposed by the Cloud Server.

.. note::

    In this case, the DDS Router will forward all topics found in the network.
    However, it is important to mention that the ROS 2 topics relayed by the DDS Router can be filtered by configuring the ``allowlist`` and ``blocklist``.
    If this is the case please refer to the `DDS Router documentation <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/configuration.html#topic-filtering>`_ for information on how to do this.

The following figure summarizes the deployment on the edge 1.

.. figure:: /rst/figures/tutorials/cloud/edge_1_repeater_deployment.png
   :align: center

Now, run the DDS Router with the configuration file created as an argument.

.. code-block::

    ddsrouter -c <path/to/file>/ddsrouter_edge_1.yaml

Deployment of LAN 2
-------------------

Running the turtlesim_square_move on Edge 2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the ``turtlesim_square_move`` in the Edge 2 machine, which is the controller of the Edge 1 ``turtlesim_node``.
This will send commands to the ROS 2 application to the edge to move the turtle and receive information about the current state of the turtle at any time.

A ROS 2 application that moves the turtle by drawing a square has been developed for this purpose.
The application is based on the `ROS 2 tutorials <https://github.com/ros/ros_tutorials/tree/iron/turtlesim>`_, but has been slightly modified in order to make it easier to understand and adapt to the ROS 2 and modern C++ programming methods.

Then, start by creating the workspace of this application and downloading the source code:

.. code-block:: bash

    mkdir -p turtlesim_move_ws/src && cd turtlesim_move_ws/src
    git clone --branch iron https://github.com/eProsima/vulcanexus.git
    mv vulcanexus/code/turtlesim .
    rm -rf vulcanexus
    cd ..

Once created the workspace, source the Vulcanexus environment and build the ``turtlesim_square_move`` application.

.. code-block:: bash

    source /opt/vulcanexus/iron/setup.bash
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

            ros2 run docs_turtlesim turtlesim_square_move

    .. tab:: LAN

        .. code-block:: bash

            ROS_DOMAIN_ID=1 ros2 run docs_turtlesim turtlesim_square_move

        .. note::

            As stated :ref:`here <warning_lan_edge_edge>`, change the ROS 2 Domain Id if running the edge and cloud applications on the same LAN.


The important points to note in this application are the following:

*   The control application sends the movement commands to the ``turtlesim_node`` through a publisher in the `geometry_msgs/msg/Twist <https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html>`_ topic.
    This topic expresses the velocity at which the turtle has to move divided into linear velocity (``linear``) and angular velocity (``angular``).

*   The application knows the position of the turtle on the map at any moment and whether it is stopped or moving by subscribing to the `turtlesim/msg/pose <https://docs.ros2.org/foxy/api/turtlesim/msg/Pose.html>`_ topic.
    This topic provides information about the turtle's coordinates (``x`` and ``y``) and the turtle's rotation (``theta``). We can also know its linear and angular velocity (``linear_velocity`` and ``angular_velocity``).


Running DDS Router Edge 2
^^^^^^^^^^^^^^^^^^^^^^^^^

As the Repeater server is the same for both edges, the configuration of the DDS Router Edge 2 is very similar to
:ref:`the one for the DDS Router Edge 1 <tutorials_cloud_edge_edge_repeater_cloud_edge_edge_repeater_cloud_running_router_1>`.
In this example both edges use different ports to communicate with the Repeater, simulating 2 different networks available in the Cloud.
However this is not needed, and only one address could be used.
The following snippet shows a configuration file (changing Domain for LAN scenarios):

.. tabs::

    .. tab:: WAN

        .. literalinclude:: /resources/tutorials/cloud/edge_edge_repeater_cloud/dds_router_edge_2_wan.yaml
            :language: yaml

    .. tab:: LAN

        .. literalinclude:: /resources/tutorials/cloud/edge_edge_repeater_cloud/dds_router_edge_2_lan.yaml
            :language: yaml

        .. note::

            As stated :ref:`here <warning_lan_edge_edge>`, set the ROS 2 Domain Id on the ``local`` participant in order to discover the ``turtlesim_square_move`` ROS 2 node.


Now, run the DDS Router with the configuration file created as an argument.

.. code-block::

    ddsrouter -c <path/to/file>/ddsrouter_edge_2.yaml

The following figure summarizes the deployment on the edge 2.

.. figure:: /rst/figures/tutorials/cloud/edge_2_repeater_deployment.png
   :align: center

Deployment of Cloud Repeater
----------------------------

In order to communicate both edges, a DDS Router configured as *Repeater* is used, forwarding the messages from one edge to the other.
This machine should be accessible from the Internet.
In case this device is working under a NAT, check :ref:`previous tutorial <tutorials_cloud_wan_edge_cloud_wan_edge_cloud_configure_transversal_nat>`
for more information about how to configure the NAT to be accessible from the outside.
The following snippet shows the configuration file for this DDS Router:

.. literalinclude:: /resources/tutorials/cloud/edge_edge_repeater_cloud/dds_router_repeater.yaml
    :language: yaml

In this case, there is only one Participant configured as ``repeater``.
This Participant will wait for external Participants to communicate via Initial Peers from its ``listening-addresses``.
Once the discovery of clients occurs, this Repeater will forward the data from one edge to the other.

To run the DDS Router with the configuration file created as an argument, execute the following command after having sourced the Vulcanexus environment:

.. code-block::

    ddsrouter -c <path/to/file>/dds_router_repeater.yaml

.. note::

    The Repeater Participant is not limited by number of listening-addresses, neither by number of edge Routers.
    It can open as many ports and interfaces as needed, and can forward messages from any number of edges, without
    re-sending redundant information or sending back any message.

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
