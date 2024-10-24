.. _tutorials_deployment_discovery_server_over_wifi_example:

Discovery Server over Wifi
==========================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

By default, ROS 2 uses the Simple Discovery Protocol (SDP), which relies on multicast communication for discovering
other nodes.
However, when dealing with large, complex, or WiFi-based networks, Fast DDS Discovery Server offers a more robust
and flexible solution.

Fast DDS allows ROS 2 nodes to connect to a centralized Discovery Server that simplifies and accelerates the discovery
phase.
This server-based approach improves network stability, optimizes resource usage, and eliminates the challenges
typically faced in more complex network environments.

With a few configuration steps, Fast DDS Discovery Server enhances the node discovery process and is particularly
well-suited for:

    * Large-scale networks
    * Complex network topologies
    * WiFi networks with potential multicast limitations

By introducing the Discovery Server into your ROS 2 setup, you gain better control over node registration,
communication efficiency, and overall performance, making it a highly effective solution in demanding network scenarios.

Discovery Server
^^^^^^^^^^^^^^^^

The `Discovery Server <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html#discovery-server>`__
in Fast DDS offers a robust alternative to the default ROS 2 `Simple Discovery Protocol (SDP)
<https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#simple-disc-settings>`__,
designed to improve discovery efficiency in large or complex networks. Unlike SDP, which uses multicast to broadcast
discovery information across all nodes, the Discovery Server introduces a Client-Server Architecture that reduces
unnecessary metatraffic (message exchange among `DomainParticipants
<https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#dds-
layer-domainparticipant>`__ to identify each other) by centralizing the discovery process.

In this architecture:

- A |SERVER| acts as a centralized hub that gathers and redistributes discovery information from connected
  Clients and other servers.
  It ensures that each client only receives the information needed to communicate with its relevant peers,
  thereby significantly reducing network bandwidth usage and improving the discovery phase's speed.
  Servers can also interconnect to form a network of discovery servers, ensuring that information is shared
  across multiple servers while avoiding single points of failure. This interconnection allows scalability
  and fault tolerance, making the discovery process more resilient.

- A |CLIENT| is a participant (ROS 2 node) that sends its discovery data to the server it connects to.
  The client receive back only the data it needs for its specific communication requirements (matching
  topics, publishers, and subscribers), minimizing overhead compared to SDP, where all nodes discover and
  track all other nodes.

- A |SUPER_CLIENT| is a *client* that receives the discovery information known by the *server*, in opposition to
  *clients*, which only receive the information they need.

- In cases where redundancy is essential, |BACKUP| *servers* can persist their discovery database to a file,
  ensuring that the network state is maintained between restarts. This backup capability secures the network
  graph but can increase discovery times due to the overhead of database persistence.

Advantages of Fast DDS Discovery Server
"""""""""""""""""""""""""""""""""""""""

- Efficient Communication: Clients only receive the discovery information they need, rather than all possible data as
  in the SDP approach.

.. figure:: /rst/figures/tutorials/core/discovery_server/discovery_server_complex.svg
   :align: center
   :scale: 100%

Comparison of Simple Discovery Protocol and Discovery Server mechanisms

- Network Optimization: Reduces the bandwidth usage (see `this
  <https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#compare-discovery-server-with-simple-discovery>`__
  comparison of the reduction in traffic when deploying discovery with SDP to Discovery Server) and eliminates the need
  for multicast, making it suitable for networks with strict performance requirements or those that don’t support
  multicast (e.g., WiFi networks).

- Scalability: Well-suited for large and complex networks, where centralized discovery management simplifies node
  communication.

Overview
--------

This tutorial will demonstrate how to establish communication between four ROS 2 ``demo_nodes_cpp`` (two ``talker`` and
two ``listeners``) across two different machines connected via a Wi-Fi network.
The setup will include Machine 1 running a Discovery Server and two ``talker`` nodes, and Machine 2 running two
``listener`` nodes.
Each node will utilize Fast DDS Discovery Server to automatically discover and communicate with each other across
the Wi-Fi network.
The aim of this tutorial is to test the discovery and connectivity of the nodes that have been discovered through
the Discovery Server, ensuring that the ``listener`` nodes on Machine 2 can receive messages from the ``talker`` nodes
on Machine 1.

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`__
* `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`__
* `Docker installation <https://docs.vulcanexus.org/en/latest/rst/installation/docker.html>`__

Run this tutorial
------------------

Configure Discovery Server entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
On Machine 1, start by launching the Fast DDS Discovery Server.
Open a terminal and configure the Fast DDS server as follows:

.. tabs::

    .. tab:: Machine 1

        .. code-block:: bash

            docker run \
                -it \
                --name discovery_server_container \
                --privileged \
                --net host \
                --ipc host \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                ubuntu-vulcanexus:iron-desktop



Once the container is running, configure Fast DDS *server* using the
`Fast DDS Discovery CLI <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`__:
The *server* ``discovery id`` would be set as ``0``:

.. code-block:: bash

    fastdds discovery --server-id 0

The output should look similar to the following:

.. code-block:: bash

    ### Server is running ###
    Participant Type:   SERVER
    Security:           NO
    Server ID:          0
    Server GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
    Server Addresses:   UDPv4:[0.0.0.0]:11811

The output displays the ``server ID`` set, followed by the security and server GUID prefix.
Server address ``0.0.0.0`` tells *Fast DDS* to listen on all available interfaces.
Finally, the ``11811`` default port would be necessary for further configuration.

After setting up the Discovery Server, you need to launch four additional Docker containers, two in each machine,
that will act as clients.
First, open two new terminals in each machine and run the following command to start each container in the same network:

.. code-block:: bash

    docker run \
        -it \
        --name <container_name> \
        --privileged \
        --net host \
        --ipc host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        ubuntu-vulcanexus:iron-desktop

Replace ``<container_name>`` with a unique name for each container.
Once the containers are running, and after sourcing the Vulcanexus environment within each container, the easiest way
to configure the clients to point to the *Discovery Server* is by setting
`Environment Variables <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html>`.
There are two different environment variables that adapt to different needs:

1. ``ROS_DISCOVERY_SERVER``:

    This is the simplest option. It allows you to specify the addresses of the Discovery Servers that clients
    should contact to discover other participants. You can set this variable inside the container as follows:

    .. code-block:: bash

        export ROS_DISCOVERY_SERVER=<server_ip>:<port>"

    Replace ``<server_ip>`` with the IP address of the Discovery Server and <port> with the connection port.

2. ``FASTDDS_ENVIRONMENT_FILE``:

    This variable allows you to specify the path to a JSON file that contains the IP addresses and ports of the
    Discovery Servers the clients should contact.
    It provides more flexibility, as you can modify the locators (i.e., server addresses) dynamically at runtime.
    To use this option, create a JSON file with the following structure:

    .. code-block:: xml

        {
            "ROS_DISCOVERY_SERVER": "<server_ip>:<port>"
        }

    Save the file in a known location and set the environment variable to point to it:

    .. code-block:: bash

        export FASTDDS_ENVIRONMENT_FILE="<json_file_path>"

Regardless of the environment variable used, make sure to set it in each of the containers where the ROS 2 nodes will
be running.
Since both the Discovery Server and the ``talker`` nodes are running on Machine 1, the ``talker`` nodes within Machine
1's containers should be configured to point to ``127.0.0.1`` as the Discovery Server is running locally on the same
host:

.. code-block:: bash

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811

On Machine 2, where the ``listener`` nodes are running, the ``ROS_DISCOVERY_SERVER`` variable should point to the IP
address of Machine 1 on the Wi-Fi network, as the Discovery Server is running on a different machine.
To configure the ``listener`` nodes on Machine 2, set the environment variable to point to Machine 1's Wi-Fi IP address:

.. code-block:: bash

    export ROS_DISCOVERY_SERVER=<machine_1_wifi_ip>:11811

Run ROS 2 demo nodes
^^^^^^^^^^^^^^^^^^^^

After all the configurations have been set, run the ``talker`` and ``listener`` *client* nodes in each machine:

.. tabs::

    .. tab:: Machine 1

        .. tabs::

            .. tab:: Client talker A

                .. code-block:: bash

                    ros2 run demo_nodes_cpp talker --ros-args -r __node:=talker_A -r chatter:=topic_A

            .. tab:: Client talker B

                .. code-block:: bash

                    ros2 run demo_nodes_cpp talker --ros-args -r __node:=talker_B -r chatter:=topic_B

    .. tab:: Machine 2

        .. tabs::

            .. tab:: Client listener A

                .. code-block:: bash

                    ros2 run demo_nodes_cpp listener --ros-args -r __node:=listener_A -r chatter:=topic_A

            .. tab:: Client listener B

                .. code-block:: bash

                    ros2 run demo_nodes_cpp listener --ros-args -r __node:=listener_B -r chatter:=topic_B