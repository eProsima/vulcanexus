.. include:: ../../../exports/alias.include

.. _tutorials_deployment_discovery_server_over_wifi_example:

How to solve wireless network issues in ROS2
============================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Overview
--------

This tutorial will demonstrate how to address common issues encountered when connecting ROS 2 nodes over a Wi-Fi network.
In this tutorial, two ``image_tools`` `publisher` nodes will communicate with two `subscriber` nodes, with each `publisher`-`subscriber` pair running on separate hosts.
The discovery process will use a Discovery Server instead of multicast, and the builting transport will be configured to TCP, to handle large data transfers over a reliable protocol rather than the default UDP.
This setup provides a network architecture that overcomes node discovery challenges in environments where multicast is not possible, while facilitating the transmission of large data over Wi-Fi or other lossy networks.

.. figure:: /rst/figures/tutorials/core/discovery_server/ros2_wifi.svg
   :align: center
   :scale: 100%

The goal of this tutorial is to configure the Discovery Server and Large Data mode for reliable discovery, connectivity and communication across nodes in constrained network environments, ensuring that the ``image_tools`` subscriber nodes can seamlessly receive large data video frames from the publisher nodes.

A brief theoretical overview of the Discovery Server and TCP configuration will be provided for additional context.
However, readers may skip directly to the prerequisites section if they prefer to start with the setup steps.

Background
----------

By default, ROS 2 uses the Simple Discovery Protocol (SDP), which relies on multicast communication for discovering other nodes.
However, when dealing with large, complex, or WiFi-based networks, **ROS2 Discovery Server** offers a more robust and flexible solution.

Fast DDS allows ROS 2 nodes to connect to a centralized Discovery Server that simplifies and accelerates the discovery phase.
This server-based approach improves network stability, optimizes resource usage, and eliminates the challenges
typically faced in more complex network environments.
Additionally, Fast DDS supports a **Large Data mode** configuration, which facilitates the efficient transfer of large data types, such as video streams or point clouds, making it ideal for WiFi architectures where data transmission needs are high.

With a few configuration steps, *ROS2 Discovery Server* and *Large Data mode* enhance node discovery process and data transfer, particularly well-suited for:

    * Large-scale networks
    * Complex network topologies
    * WiFi networks with potential multicast limitations and large data requirements

By introducing the *Discovery Server* and *Large Data mode* into your ROS 2 setup, you gain better control over node registration, communication efficiency, and large data handling, creating a highly effective solution for demanding network scenarios.

Discovery Server Arquitecture
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The `Discovery Server <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html#discovery-server>`__ in Fast DDS offers a robust alternative to the default ROS 2 `Simple Discovery Protocol (SDP) <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#simple-disc-settings>`__, designed to improve discovery efficiency in large or complex networks. Unlike SDP, which uses multicast to broadcast discovery information across all nodes, the Discovery Server introduces a Client-Server Architecture that reduces unnecessary metatraffic (message exchange among `DomainParticipants <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#dds-layer-domainparticipant>`__ to identify each other) by centralizing the discovery process.

In this architecture:

- A |SERVER| acts as a centralized hub that gathers and redistributes discovery information from connected Clients and other servers.
  It ensures that each client only receives the information needed to communicate with its relevant peers, thereby significantly reducing network bandwidth usage and improving the discovery phase's speed.

- A |CLIENT| is a node that sends its discovery data to the server it connects to.
  The client receive back only the data it needs for its specific communication requirements (matching topics, publishers, and subscribers), minimizing overhead compared to SDP, where all nodes discover and track all other nodes.

- A |SUPER_CLIENT| is a *client* that receives the discovery information known by the *server*, in opposition to *clients*, which only receive the information they need.

Advantages of ROS2 DDS Discovery Server
"""""""""""""""""""""""""""""""""""""""

- Efficient Communication: Clients only receive the discovery information they need, rather than all possible data as in the SDP approach.

- Network Optimization: Reduces the bandwidth usage (see `this <https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#compare-discovery-server-with-simple-discovery>`__ comparison of the reduction in traffic when deploying discovery with SDP to Discovery Server).
  Crucially, it eliminates the reliance on multicast, which is essential for environments with strict performance requirements or for networks that do not support multicast—such as many WiFi networks.
  By removing the need for multicast, the Discovery Server makes ROS2 deployment possible in diverse and constrained network conditions.

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

    For compability issues, Discovery Server has to be configured with TCP transport protocol when Large Data is set.

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`__
* `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`__
* `Docker installation <https://docs.vulcanexus.org/en/latest/rst/installation/docker.html>`__

Additionally, you need to have OpenCV installed.
You can follow the `OpenCV documentation
<https://docs.opencv.org/2.4/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html#table-of-content-introduction>`__
for detailed installation instructions.

You will also need the ROS package ``image_tools``, which can be installed by running:

.. code:: bash

    sudo apt-get install ros-humble-image-tools

Lastly, ensure you have a working webcam connected to both of the hosts.

Considerations for Video Streaming
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before proceeding with the tutorial, it's important to consider the following factors when streaming large video images over Wi-Fi:

1. Network Bandwidth:
    Ensure that the Wi-Fi network has sufficient bandwidth to support the video stream's bitrate, especially when multiple devices are connected.
    Higher-resolution or higher-framerate video streams demand more bandwidth, so consider compressing the video or reducing the resolution if the Wi-Fi network has limitations.

2. Network Congestion:
    Multiple devices using the same network can lead to congestion, resulting in buffering or dropped frames during video streaming.
    To enhance streaming performance, limit other network activities (such as large downloads or additional streams) on the same Wi-Fi network while the video is being transmitted.

3. Video Compression and Encoding:
    Utilizing efficient video codecs can significantly reduce the amount of data needed for transmission without compromising quality.
    If high-quality video isn't essential, adjusting the encoding settings to a lower bitrate will further decrease bandwidth requirements.

Run this tutorial
------------------

Configure Discovery Server entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On Host A, start by launching the ROS2 Discovery Server.
Open a terminal and configure the Fast DDS server as follows:

.. tabs::

    .. tab:: Host A

        .. code-block:: bash

            docker run \
                -it \
                --name discovery_server_container \
                --privileged \
                --net host \
                --ipc host \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                ubuntu-vulcanexus:humble-desktop

Once the container is running, configure Fast DDS *server* using the `Fast DDS Discovery CLI <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`__:
The *server* ``discovery id`` would be set as ``0`` and will be listening on localhost with default TCP port 42100:

.. code-block:: bash

    fastdds discovery --server-id 0 -t 192.168.1.165

The output should look similar to the following:

.. code-block:: bash

    ### Server is running ###
    Participant Type:   SERVER
    Security:           NO
    Server ID:          0
    Server GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
    Server Addresses:   TCPv4:[192.168.1.165]:42100-42100

The output displays the ``server ID`` set, followed by the security and server GUID prefix.
Server address ``192.168.1.165`` tells *Fast DDS* to listen on Wifi 192.168.1.165 interface.
Finally, the ``42100-42100`` default port would be necessary for further configuration.

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
        ubuntu-vulcanexus:humble-desktop

Replace ``<container_name>`` with a unique name for each container.
Once the containers are running, and after sourcing the Vulcanexus environment within each container, the easiest way to configure the clients to point to the *Discovery Server* is by setting `Environment Variables <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html>`__.
There are two different environment variables that adapt to different needs:

1. ``ROS_DISCOVERY_SERVER``:

    This is the simplest option.
    It allows you to specify the addresses of the Discovery Servers that clients should contact to discover other nodes. You can set this variable inside the container as follows:

    .. code-block:: bash

        export ROS_DISCOVERY_SERVER=<server_ip>:<port>"

    Replace ``<server_ip>`` with the IP address of the Discovery Server and <port> with the connection port.

2. ``FASTDDS_ENVIRONMENT_FILE``:

    This variable allows you to specify the path to a JSON file that contains all the environment variables.
    You can specify the IP addresses and ports of the Discovery Servers the clients should contact.
    It provides more flexibility, as you can modify the locators (i.e., server addresses) dynamically at runtime.
    To use this option, create a JSON file with the following structure:

    .. code-block:: xml

        {
            "ROS_DISCOVERY_SERVER": "<server_ip>:<port>"
        }

    Save the file in a known location and set the environment variable to point to it:

    .. code-block:: bash

        export FASTDDS_ENVIRONMENT_FILE="<json_file_path>"

Regardless of the environment variable used, make sure to set it in each of the containers where the ROS 2 nodes will be running.
To facilitate the environment setup, and since more than one environment variable needs to be set, we will use the ``FASTDDS_ENVIRONMENT_FILE`` variable to include them all in a single JSON file.

Since the Discovery Server, a *publisher* and a *listener* ``image_tools`` nodes are running on Host A, these two nodes within Host A's containers should be configured to point to ``127.0.0.1`` as the Discovery Server is running locally on the same host.
Aditionally, we will configure all nodes to be |SUPER_CLIENT| and `Large Data Mode <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/tcp/tcp_with_multicast_discovery.html>`__ as builtin transport.
On Host B, where other *publisher* and *subscriber* ``image_tools`` nodes will be running, the ``ROS_DISCOVERY_SERVER`` variable should point to the IP address of Host A on the Wi-Fi network, as the Discovery Server is running on a different machine.
The json file for the nodes in both **Host A** and **Host B** will look like:

.. code-block:: xml

    {
        "ROS_DISCOVERY_SERVER": "TCPv4:[192.168.1.165]:42100",
        "ROS_SUPER_CLIENT": "TRUE",
        "FASTDDS_BUILTIN_TRANSPORTS"="LARGE_DATA"
    }

Finally, make te export of the ``FASTDDS_ENVIRONMENT_FILE`` in every node terminal, pointing to the JSON file path:

.. code-block:: bash

    export FASTDDS_ENVIRONMENT_FILE="<json_file_path>"

Run ROS 2 demo nodes
^^^^^^^^^^^^^^^^^^^^

After all the configurations have been set, run the *publisher* and  *subscriber* client nodes in each host:

.. tabs::

    .. tab:: Host A

        .. tabs::

            .. tab:: Client publisher 1

                .. code-block:: bash

                    ros2 run image_tools cam2image --ros-args -r __node:=publisher_1 -r image:=image1

            .. tab:: Client subscriber 2

                .. code-block:: bash

                    ros2 run image_tools showimage --ros-args -r __node:=subscriber_2 -r image:=image2

    .. tab:: Host B

        .. tabs::

            .. tab:: Client publisher 2

                .. code-block:: bash

                    ros2 run image_tools cam2image --ros-args -r __node:=publisher_2 -r image:=image2

            .. tab:: Client subscriber 1

                .. code-block:: bash

                    ros2 run image_tools showimage --ros-args -r __node:=subscriber_1 -r image:=image1

This will start publishing images from the webcams.
If you don't have a camera connected, you can use the following command to publish predefined images:

.. tabs::

    .. tab:: Host A

        .. tabs::

            .. tab:: Client publisher 1

                .. code-block:: bash

                    ros2 run image_tools cam2image --ros-args -p burger_mode:=True -r __node:=publisher_1 -r image:=image1

    .. tab:: Host B

        .. tabs::

            .. tab:: Client publisher 2

                .. code-block:: bash

                    ros2 run image_tools cam2image --ros-args -p burger_mode:=True -r __node:=publisher_2 -r image:=image2


You should see terminal outputs like:

.. code:: bash

    Publishing image #1
    Publishing image #2
    Publishing image #3
    ...
