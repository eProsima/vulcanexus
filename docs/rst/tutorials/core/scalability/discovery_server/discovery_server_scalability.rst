.. _tutorials_discovery_server_scalability:

How to Use a Discovery Server to Optimize ROS 2 Node Discovery
==============================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Facing Scalability Challenges in Your ROS 2 System?
---------------------------------------------------

Have you ever struggled with scaling your ROS 2 network as it grows in size?
As more nodes, topics, and services are added to a ROS 2 system, the underlying DDS (Data Distribution Service) quickly forms a fully-connected graph, meaning that every node discovers and communicates with every other node.
While this setup guarantees robust connectivity, it can also lead to network congestion and high discovery traffic as the system scales, impacting performance and reliability.

The Fast DDS Discovery Server offers a centralized mechanism for managing node discovery, allowing nodes to connect through a server instead of relying on the default multicast-based Simple Discovery Protocol (SDP).
This reduces discovery traffic, provides better control over network management, and improves scalability in complex or large network scenarios.

**Quick Solution Overview**

Launch the Discovery Server locally on the default port (11811) using the following command:

.. code-block:: bash

    fastdds discovery

Overview
--------

In this tutorial, we will explore how to set up and use the Fast DDS Discovery Server with ROS 2.
Both a ``talker`` and a ``listener`` nodes from the ``demo_nodes_cpp`` ROS 2 package are created: the talker node will publish a "hello world" message every second, while the listener node will listen to these messages.
By using the Discovery Server, nodes register and discover each other through a centralized server instead of communicating directly with every other node.

.. figure:: /rst/figures/tutorials/core/discovery_server/Discovery_Server.svg
   :align: center
   :scale: 150%

Background
----------

By default, ROS 2 uses the Simple Discovery Protocol (SDP), which relies on multicast communication to discover other nodes.
While SDP is suitable for many scenarios, it may not be the best choice in complex and large-scale network configurations.
This is where the Fast DDS Discovery Server comes into play. Fast DDS offers an alternative discovery mechanism that allows ROS 2 nodes to connect to a Discovery Server.
This server acts as a centralized point for node registration and discovery, providing more flexibility and control in managing the network of nodes and is particularly advantageous in scenarios such as:

* Large networks

* Complex networks

* WiFi networks

In this tutorial, we will explore how to set up and use the Fast DDS Discovery Server with ROS 2, enabling robust and dynamic node discovery communication.

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`__
* `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`__
* `Docker installation <https://docs.vulcanexus.org/en/latest/rst/installation/docker.html>`__


Run this tutorial
------------------

Once we finish with the set up of Vulcanexus environment we are ready to start with the *talker-listener* demo tutorial.
By sourcing Vulcanexus you will get access to the CLI tool ``fastdds``, together with the ROS 2 environment.
This tool gives access to the `discovery tool <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`__, which can be used to launch a discovery server.

.. important::

    Do not forget to source ROS 2 every time you open another session to have access to the ROS 2 environment and the CLI tool ``fastdds``.

Setup Discovery Server
^^^^^^^^^^^^^^^^^^^^^^
Start by launching a discovery server on port 11811 (default port) and listening on all available interfaces.
The role of the server is to re-distribute the clients (and servers) discovery information to their known clients (and servers).

.. code-block:: bash

    fastdds discovery


Launch Talker Node
^^^^^^^^^^^^^^^^^^^
Execute the Talker demo to publish on the ``/chatter`` topic. When setting the ``ROS_DISCOVERY_SERVER`` environment variable you are indicating that ROS node should act as a client that connects to a discovery server to discover other nodes on the network.
Use the argument ``--remap __node:=talker_discovery_server`` to change the node’s name for this tutorial.

.. code-block:: bash

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server


Launch Listener Node
^^^^^^^^^^^^^^^^^^^^^
Execute the Listener demo to listen to the ``/chatter`` topic.

.. code-block:: bash

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server


