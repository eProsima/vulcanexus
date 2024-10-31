.. include:: ../../../../exports/alias.include

.. _tutorials_large_data_video_streaming:

How to handle large data video streaming in ROS 2
=================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Have you ever faced problems streaming video in ROS 2?
------------------------------------------------------

If you've experienced issues streaming video in ROS 2, such as video freezing, packet loss, or jitter, especially over WiFi, you can enable Fast DDS’s **Large Data Mode** with a simple environment variable configuration.
This allows you to bypass some of the limitations of UDP transport by using TCP or Shared Memory (SHM) for large data transmission, enhancing the reliability and quality of your video stream.

**Quick Solution Overview**

To apply this solution, set the ``FASTDDS_BUILTIN_TRANSPORTS`` environment variable to ``LARGE_DATA`` in every ROS 2 node involved in transmitting large data messages:

.. code:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

Overview
--------

In this tutorial, we will demonstrate how to efficiently transmit large data, specifically a video stream, over WiFi between two hosts using Fast DDS.
The tutorial will guide you through two scenarios:

1. *Default UDP Transmission*: We will first configure a basic video transmission using the default UDP transport, allowing you to observe the potential issues, such as packet loss or jitter, commonly encountered when transmitting large data over unreliable networks like WiFi.

2. *Large Data Mode*: Next, we will enable Fast DDS's Large Data Mode, which leverages TCP or Shared Memory for data transmission, offering more reliability for large data transfers. You will see how this configuration improves the stability and quality of the video reception on the receiving host.

3. *Advanced Large Data Mode*: Finally, we will explore additional custom configurations within the Large Data Mode to further optimize video streaming performance. By adjusting network parameters, such as socket buffer sizes and non-blocking mode, and setting a TCP negotiation timeout, this configuration enhances stability and responsiveness, especially when streaming large data over networks prone to instability.

By comparing the three performances, we'll understand the impact of transport configuration on large data transfers and learn how to optimize Fast DDS for reliable communication in scenarios that involve sending large data over networks prone to instability, such as WiFi.

.. warning::

    A brief theoretical overview of the Large Data Mode will be provided for additional context.
    However, readers may skip directly to the :ref:`tutorials_large_data_video_streaming_prerequisites` section if they prefer to start with the setup steps.

Background
----------

When working with large data transfers in DDS (Data Distribution Service), it's important to consider certain aspects of how data is transmitted, especially when relying on UDP as the default network protocol.
UDP offers flexibility and control over key aspects of message delivery, such as allowing users to configure messages as either best-effort or reliable, depending on their needs. This control is a major advantage for many real-time applications.

However, due to the nature of UDP, there are certain factors that need attention when handling larger data, such as images or point clouds, commonly seen in robotics or other high-bandwidth applications.
While UDP is flexible, it has not received the same performance optimizations as TCP, which is widely used in modern networking.
For example, Linux systems often allocate small default buffer sizes for UDP (around 256 KB), which may not be enough for larger data like images or point clouds. Combined with unreliable networks, like WiFi, this can cause issues like repeated retransmissions and buffer overflows.

Manual tuning options
^^^^^^^^^^^^^^^^^^^^^

For handling large data efficiently, Fast DDS offers various tuning options to avoid issues like buffer overflows and retransmissions when using UDP.
Key configurations include increasing socket buffer sizes, adjusting the transmit queue length, and using flow controllers to control data transmission rates.
For more details and advanced configurations, refer to the official documentation `here <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html>`__.

Large Data Mode
^^^^^^^^^^^^^^^

In addition to manual tuning options, Fast DDS offers a more straightforward solution for large data handling that doesn’t require deep network knowledge: the `Large Data Mode <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/tcp/tcp_with_multicast_discovery.html>`__.
This configuration allows Fast DDS DomainParticipants to perform the PDP discovery phase using UDP multicast, while transmitting large data samples over more reliable transports like TCP or Shared Memory (SHM).
This approach provides the flexibility of UDP-based discovery with the reliability of TCP or SHM for large data transfers.

Large Data Mode can be easily enabled using the ``FASTDDS_BUILTIN_TRANSPORTS`` environment variable, XML profiles, or code:

.. code:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

This simplifies the configuration process and helps optimize large data transmission without needing extensive network adjustments.

Large Data with Configuration options
"""""""""""""""""""""""""""""""""""""

In addition to the default setup, Large Data Mode offers advanced configuration options to enhance performance when handling large data, such as video streaming.
Fast DDS provides these key parameters to adjust transport behavior in ``LARGE_DATA`` mode:

- ``max_msg_size``: Maximum message size before fragmentation, up to (2^32)-1 B bytes for TCP and SHM, and 65500 KB for UDP.
- ``sockets_size``: Size of the send and receive buffers; should match or exceed max_msg_size and defines the SHM size.
- ``non_blocking``: Avoids application blocking when buffers are full, though some message loss may occur. Default is false.
- ``tcp_negotiation_timeout``: Sets a timeout for TCP port negotiation, helping prevent initial message loss.

Tuning these parameters for large messages and enabling non-blocking mode can optimize performance for high-bandwidth data flows like video.

The following snippets show how to configure the ``LARGE_DATA`` mode:

.. code-block:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=1MB&sockets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50

.. _tutorials_large_data_video_streaming_prerequisites:

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

    sudo apt-get install ros-humble-image-tools

Lastly, ensure you have a working webcam connected to one of the hosts.

Run this tutorial
------------------

UDP Streaming
^^^^^^^^^^^^^

In the first part of the tutorial, we will use the default UDP settings.
Follow these steps:

1. Source your Vulcanexus setup file on both computers:

.. code:: bash

    . <path to Vulcanexus install space>/setup.bash

2. On the receiving computer, run the subscriber node:

.. code:: bash

    ros2 run image_tools showimage

At this point, nothing will happen yet, as ``showimage`` is waiting for a publisher on the image topic.
Remember to terminate this process with ``Ctrl-C`` later; closing the window alone will not stop it.

3. On the sending computer, source the install file again and run the publisher node:

.. code:: bash

    ros2 run image_tools cam2image

This will start publishing images from your webcam.
If you don't have a camera connected, you can use the following command to publish predefined images:

.. code:: bash

    ros2 run image_tools cam2image --ros-args -p burger_mode:=True

You should see terminal output like:

.. code:: bash

    Publishing image #1
    Publishing image #2
    Publishing image #3
    ...

This occurs because the default buffer sizes for UDP are relatively small, which can lead to overflow when dealing with larger data packets, such as video streams.
As a result, packets may be dropped during transmission, causing interruptions in the video feed.
Furthermore, the inherent unreliability of UDP, especially in WiFi environments, can significantly compound these challenges, making it difficult to sustain a stable connection for real-time streaming.

Large Data Mode
^^^^^^^^^^^^^^^

In the second part, we will switch to using the Large Data mode for improved performance.
This is done by exporting the environment variable in the terminal on both computers:

.. code:: bash

    export FASTDDS_BUILTIN_TRANSPORTS="LARGE_DATA"

With this setting, run the same publisher and subscriber commands again.
This time, you should experience smooth and uninterrupted video streaming with no freezing or image loss.

Advanced Large Data Mode
^^^^^^^^^^^^^^^^^^^^^^^^

In the final part of this tutorial, we’ll guide you through more advanced settings to improve video streaming performance in ROS 2 using Fast DDS.
These additional steps are aimed at optimizing reliability and quality, particularly over unstable or high-bandwidth networks.

Increasing Socket Buffer Sizes on Linux
"""""""""""""""""""""""""""""""""""""""

First, we’ll increase the socket buffer sizes, which can help prevent packet loss and improve the stability of the video stream.
To adjust the send and receive socket buffers on Linux, follow these steps:

1. Check the current buffer sizes:

    * For send buffers:

    .. code:: bash

        sudo sysctl -a | grep net.core.wmem_max

    * For receive buffers:

    .. code:: bash

        sudo sysctl -a | grep net.core.rmem_max

2. Increase the buffer sizes if needed:

    * For send buffers:

    .. code:: bash

        sudo sysctl -w net.core.wmem_max=12582912

    * For receive buffers:

    .. code:: bash

        sudo sysctl -w net.core.rmem_max=12582912

Large Data Mode with Custom Configuration
"""""""""""""""""""""""""""""""""""""""""

After adjusting the socket buffer sizes, we’ll configure Fast DDS’s Large Data Mode with additional parameters to further optimize performance.
Use the following environment variable to apply these settings:

.. code:: bash

    export FASTDDS_BUILTIN_TRANSPORTS="LARGE_DATA?max_msg_size=1MB&sockets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50"

This configuration sets the ``maximum_message_size`` and ``socket_buffer_sizes`` to handle larger data without fragmentation, enables ``non-blocking`` mode to avoid blocking the application when buffers are full, and adds a ``timeout`` for TCP port negotiation to prevent initial packet loss.

Conclusion
----------

In this tutorial, we explored various configurations to optimize large data transmission, specifically video streaming, using Fast DDS over WiFi in ROS 2.
We began with a basic UDP transmission setup to highlight the common challenges of packet loss and jitter in high-bandwidth scenarios.
By enabling Fast DDS’s Large Data Mode, we saw how switching to TCP can improve the reliability of video transmission.

For even better performance, we applied advanced configurations within Large Data Mode, such as increasing maximum message sizes and socket buffer sizes, enabling non-blocking mode, and setting a TCP negotiation timeout.
These settings provide greater stability and responsiveness, making Fast DDS a powerful tool for handling high-throughput data like video over potentially unreliable networks.

With these techniques, you can enhance the reliability of ROS 2 video streaming, reducing issues related to packet loss, freezing, and jitter.
Whether streaming video or other large data types, these configurations can be applied to build more robust communication systems in your ROS 2 applications.
