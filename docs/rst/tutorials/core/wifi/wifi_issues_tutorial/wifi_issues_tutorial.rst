.. include:: ../../../../exports/alias.include

.. _tutorials_wifi_issues_in_ros2:

How to solve wireless network issues in ROS 2
=============================================

Have you ever faced issues deploying your ROS 2 application over WiFi?
----------------------------------------------------------------------

Deploying ROS 2 applications over WiFi networks can often be challenging.
You may encounter issues with node discovery due to the limitations of multicast, or experience poor performance when streaming large data, such as video, across unstable or lossy networks.
Fortunately, there's a straightforward solution: leveraging the **Discovery Server** and **Large Data** mode in Fast DDS.

**Quick Solution Overview**

To address these issues, follow these simple steps:

1. **Configure a Discovery Server** to replace multicast-based discovery with a more reliable client-server setup.

.. code-block:: bash

    fastdds discovery -t <wifi_ip_address> -q 42100

2. Create the JSON configuration file for each client, setting up **Large Data** mode and the **Discovery Server** to point to:

.. code-block:: xml

    {
        "ROS_DISCOVERY_SERVER": "TCPv4:[<wifi_ip_address>]:42100",
        "ROS_SUPER_CLIENT": "TRUE",
        "FASTDDS_BUILTIN_TRANSPORTS"="LARGE_DATA"
    }

3. Apply the JSON file in each client by setting the environment variable:

.. code-block:: bash

    export FASTDDS_ENVIRONMENT_FILE="<json_file_path>"

Overview
--------

This tutorial will demonstrate how to address common issues encountered when connecting ROS 2 nodes over a WiFi network.
In this tutorial, two ``image_tools`` `publisher` nodes will communicate with two `subscriber` nodes, with each `publisher`-`subscriber` pair running on separate hosts.
The discovery process will use a Discovery Server instead of multicast, and the builtin transport will be configured to TCP, to handle large data transfers over a reliable protocol rather than the default UDP.
This setup provides a network architecture that overcomes node discovery challenges in environments where multicast is not possible, while facilitating the transmission of large data over WiFi or other lossy networks.

.. figure:: /rst/figures/tutorials/core/discovery_server/ros2_wifi.svg
   :align: center
   :scale: 100%

The goal of this tutorial is to configure the Discovery Server and Large Data mode for reliable discovery, connectivity and communication across nodes in constrained network environments, ensuring that the ``image_tools`` subscriber nodes can seamlessly receive large data video frames from the publisher nodes.

.. warning::

    A brief theoretical background of the Discovery Server and TCP configuration will be provided for additional context.
    However, readers may skip directly to the :ref:`tutorials_wifi_issues_in_ros2_prerequisites` section if they prefer to start with the setup steps.

Background
----------

By default, ROS 2 uses the Simple Discovery Protocol (SDP), which relies on multicast communication for discovering other nodes.
However, when dealing with large, complex, or WiFi-based networks, **ROS 2 Discovery Server** offers a more robust and flexible solution.

Fast DDS allows ROS 2 nodes to connect to a centralized Discovery Server that simplifies and accelerates the discovery phase.
This server-based approach improves network stability, optimizes resource usage, and eliminates the challenges
typically faced in more complex network environments.
Additionally, Fast DDS supports a **Large Data mode** configuration, which facilitates the efficient transfer of large data types, such as video streams or point clouds, making it ideal for WiFi architectures where data transmission needs are high.

With a few configuration steps, *ROS 2 Discovery Server* and *Large Data mode* enhance node discovery process and data transfer, particularly well-suited for:

    * Large-scale networks
    * Complex network topologies
    * WiFi networks with potential multicast limitations and large data requirements

By introducing the *Discovery Server* and *Large Data mode* into your ROS 2 setup, you gain better control over node registration, communication efficiency, and large data handling, creating a highly effective solution for demanding network scenarios.

Discovery Server Architecture
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The `Discovery Server <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html#discovery-server>`__ in Fast DDS offers a robust alternative to the default ROS 2 `Simple Discovery Protocol (SDP) <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#simple-disc-settings>`__, designed to improve discovery efficiency in large or complex networks. Unlike SDP, which uses multicast to broadcast discovery information across all nodes, the Discovery Server introduces a Client-Server Architecture that reduces unnecessary metatraffic (message exchange among `DomainParticipants <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#dds-layer-domainparticipant>`__ to identify each other) by centralizing the discovery process.

In this architecture:

- A |SERVER| acts as a centralized hub that gathers and redistributes discovery information from connected Clients and other servers.
  It ensures that each client only receives the information needed to communicate with its relevant peers, thereby significantly reducing network bandwidth usage and improving the discovery phase's speed.

- A |CLIENT| is a node that sends its discovery data to the server it connects to.
  The client receive back only the data it needs for its specific communication requirements (matching topics, publishers, and subscribers), minimizing overhead compared to SDP, where all nodes discover and track all other nodes.

- A |SUPER_CLIENT| is a *client* that receives the discovery information known by the *server*, in opposition to *clients*, which only receive the information they need.

Advantages of ROS 2 DDS Discovery Server
""""""""""""""""""""""""""""""""""""""""

- Efficient Communication: Clients only receive the discovery information they need, rather than all possible data as in the SDP approach.

- Network Optimization: Reduces the bandwidth usage (see `this <https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#compare-discovery-server-with-simple-discovery>`__ comparison of the reduction in traffic when deploying discovery with SDP to Discovery Server).
  Crucially, it eliminates the reliance on multicast, which is essential for environments with strict performance requirements or for networks that do not support multicast—such as many WiFi networks.
  By removing the need for multicast, the Discovery Server makes ROS 2 deployment possible in diverse and constrained network conditions.

- Scalability: Well-suited for large and complex networks, where centralized discovery management simplifies node communication.

.. figure:: /rst/figures/tutorials/core/discovery_server/discovery_server_complex.svg
   :align: center
   :scale: 100%

Comparison of Simple Discovery Protocol and Discovery Server mechanisms

Large Data Mode
^^^^^^^^^^^^^^^

Fast DDS provides a simpler solution for handling large data without requiring extensive network expertise: `Large Data Mode <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/tcp/tcp_with_multicast_discovery.html>`__.
This configuration enables Fast DDS DomainParticipants to utilize UDP multicast for the PDP discovery phase while employing more reliable transport protocols, such as TCP or Shared Memory (SHM), for transmitting large data samples.
This method combines the flexibility of UDP-based discovery with the dependability of TCP or SHM for large data transfers.

*Large Data mode* can be easily enabled using the ``FASTDDS_BUILTIN_TRANSPORTS`` environment variable, XML profiles, or code:

.. code:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

This simplifies the configuration process and helps optimize large data transmission without needing extensive network adjustments.

If further adjustments are necessary, it is important to assess the specific requirements of your architecture to ensure optimal performance.
Fast DDS provides various tuning options to prevent issues such as buffer overflows and retransmissions when using UDP.
Key configurations include increasing socket buffer sizes, adjusting the transmit queue length, and employing flow controllers to manage data transmission rates.

For more details and advanced configurations, refer to the official documentation `here <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html>`__.

.. note::

    For compatibility issues, Discovery Server has to be configured with TCP transport protocol when Large Data is set.

.. _tutorials_wifi_issues_in_ros2_prerequisites:

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`__
* `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`__
* `Docker installation <https://docs.vulcanexus.org/en/latest/rst/installation/docker.html>`__

Additionally, you need to have OpenCV installed.
You can follow the `OpenCV documentation <https://docs.opencv.org/2.4/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html#table-of-content-introduction>`__ for detailed installation instructions.

You will also need the ROS package ``image_tools``, which can be installed by running:

.. code:: bash

    sudo apt-get install ros-jazzy-image-tools

Lastly, ensure you have a working webcam connected to both of the hosts.

Considerations for Video Streaming
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before proceeding with the tutorial, consider these factors and tips to optimize video streaming over WiFi:

1. Network Bandwidth:
    To support video streaming over WiFi, ensure your network bandwidth meets the demands of the video’s bitrate, especially if multiple devices are connected.
    Higher-resolution or higher-frame-rate video streams demand more bandwidth, so consider compressing the video or reducing the resolution if the WiFi network has limitations.
    Future tutorials will cover ways to adjust these settings in ROS 2.
    Additionally, you can `adjust the socket buffer sizes <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html#tuning-socket-buffer>`__ in your system’s network settings to accommodate larger data rates, which can help stabilize high-bandwidth streams.

2. Network Congestion:
    Multiple devices using the same network can lead to congestion, resulting in buffering or dropped frames during video streaming.
    Tools like ``netstat`` or ``iptraf`` can monitor current network traffic, helping you assess if limiting other network activities—such as downloads or additional video streams—would improve performance.

3. Video Compression and Encoding:
    Utilizing efficient video codecs can significantly reduce the amount of data needed for transmission without compromising quality.
    If high-quality video isn't essential, try lowering the encoding bitrate, which can significantly reduce bandwidth requirements.

Run this tutorial
------------------

Configure Discovery Server entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On Host A, start by launching the ROS 2 Discovery Server.
Open a terminal and configure the Fast DDS server as follows:

.. code-block:: bash

    docker run \
        -it \
        --name discovery_server_container \
        --privileged \
        --net host \
        --ipc host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        ubuntu-vulcanexus:jazzy-desktop

Once the container is running, configure Fast DDS *server* using the `Fast DDS Discovery CLI <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`__.

.. code-block:: bash

    fastdds discovery -t 192.168.1.165 -q 42100 -t 127.0.0.1 -q 42101

The output should look similar to the following:

.. code-block:: bash

    ### Server is running ###
    Participant Type:   SERVER
    Security:           YES
    Server GUID prefix: 01.0f.33.0a.3a.14.27.73.00.00.00.00
    Server Addresses:   TCPv4:[192.168.1.165]:42100-42100
                        TCPv4:[127.0.0.1]:42101-42101

The output displays the security and server GUID prefix.
Server address ``TCPv4:[192.168.1.165]`` tells *Fast DDS* to listen on WiFi 192.168.1.165 interface with the ``42100`` connection port.
Server address ``TCPv4:[127.0.0.1]`` tells *Fast DDS* to listen on localhost with the ``42101`` connection port.

After setting up the Discovery Server, you need to launch four additional Docker containers, two in each host, that will act as clients.
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
        ubuntu-vulcanexus:jazzy-desktop

Replace ``<container_name>`` with a unique name for each container.
Once the containers are running, and after sourcing the Vulcanexus environment within each container, the easiest way to configure the clients to point to the *Discovery Server* is by setting `Environment Variables <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html>`__.
To simplify the configuration of all the environmental variables, we will use ``FASTDDS_ENVIRONMENT_FILE``.
This variable allows you to specify the path to a JSON file that contains all the environment variables.
You can specify the IP addresses and ports of the Discovery Servers the clients should contact.
It provides a lot of flexibility, as you can modify the locators (i.e., server addresses) dynamically at runtime.
To use this option, create a JSON file with the following structure:

.. code-block:: xml

    {
        "ROS_DISCOVERY_SERVER": "<server_ip>:<port>"
    }

Then save the file in a known location and set the environment variable to point to it:

.. code-block:: bash

    export FASTDDS_ENVIRONMENT_FILE="<json_file_path>"

Make sure to set it in each of the containers where the ROS 2 nodes will be running.

Since the Discovery Server, a *publisher* and a *listener* ``image_tools`` nodes are running on Host A, these two nodes within Host A's containers should be configured to point to localhost.
Additionally, we will configure all nodes to be |SUPER_CLIENT| and `Large Data Mode <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/tcp/tcp_with_multicast_discovery.html>`__ as builtin transport.
The json file for the nodes in **Host A** will look like:

.. code-block:: xml

    {
        "ROS_DISCOVERY_SERVER": "TCPv4:[127.0.0.1]:42101",
        "ROS_SUPER_CLIENT": "TRUE",
        "FASTDDS_BUILTIN_TRANSPORTS"="LARGE_DATA"
    }

On Host B, where other *publisher* and *subscriber* ``image_tools`` nodes will be running, the ``ROS_DISCOVERY_SERVER`` variable should point to the IP address of Host A on the WiFi network, as the Discovery Server is running on a different machine.
The json file for the nodes in **Host B** will be:

.. code-block:: xml

    {
        "ROS_DISCOVERY_SERVER": "TCPv4:[192.168.1.165]:42100",
        "ROS_SUPER_CLIENT": "TRUE",
        "FASTDDS_BUILTIN_TRANSPORTS"="LARGE_DATA"
    }

Finally, make the export of the ``FASTDDS_ENVIRONMENT_FILE`` in every node terminal, pointing to the JSON file path:

.. code-block:: bash

    export FASTDDS_ENVIRONMENT_FILE="<json_file_path>"

Run ROS 2 demo nodes
^^^^^^^^^^^^^^^^^^^^

After all the configurations have been set, run the *publisher* and  *subscriber* client nodes in each host:

Host A
""""""

Run the following command to start the ``publisher`` node on topic ``/image_1``:

.. code-block:: bash

    ros2 run image_tools cam2image --ros-args -r __node:=publisher_1 -r image:=/image_1

Then run the ``subscriber`` node on topic ``/image_2``:

.. code-block:: bash

    ros2 run image_tools showimage --ros-args -r __node:=subscriber_2 -r image:=/image_2

Host B
""""""

Run the following command to start the ``publisher`` node on topic ``/image_2``:

.. code-block:: bash

    ros2 run image_tools cam2image --ros-args -r __node:=publisher_2 -r image:=/image_2

Then run the ``subscriber`` node on topic ``/image_1``:

.. code-block:: bash

    ros2 run image_tools showimage --ros-args -r __node:=subscriber_1 -r image:=/image_1


This will start publishing images from the webcams.
If you don't have a camera connected, you can use the following command to publish predefined images:

.. code-block:: bash

    ros2 run image_tools cam2image --ros-args -p burger_mode:=True -r __node:=<publisher> -r image:=<topic>

You should see terminal outputs like:

.. code:: bash

    Publishing image #1
    Publishing image #2
    Publishing image #3
    ...

Conclusion
----------

By following this tutorial, you’ve configured ROS 2 nodes to communicate reliably over WiFi using Fast DDS’s Discovery Server and Large Data mode.
This setup replaces multicast-based discovery with a server-client model, making ROS 2 node discovery more robust in complex network conditions or over WiFi networks with limited multicast support.
The Large Data mode, meanwhile, enhances the handling of high-bandwidth data, such as video streams, over TCP, ensuring that large data transfers remain stable on lossy or congested networks.

This architecture provides a scalable, adaptable solution for deploying ROS 2 applications in network environments where stability and efficiency are critical.
If you’re interested in further optimizing or exploring more advanced configurations, consult the Fast DDS documentation on `Discovery Server <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html#discovery-server>`__ and `Large Data Mode <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/tcp/tcp_with_multicast_discovery.html>`__, where additional performance tuning options are detailed.
