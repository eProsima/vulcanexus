.. include:: ../../../../exports/alias.include

.. _tutorials_image_compression:

How to compress images for video streaming in ROS 2
===================================================

Have you ever faced problems trying to send large data images in a video in ROS 2?
----------------------------------------------------------------------------------

Transmitting uncompressed data can easily overwhelm network bandwidth, especially in WiFi setups or environments with limited resources.
This often leads to issues like lag, choppy video playback, or even dropped frames, making it hard to achieve smooth and reliable video streaming for real-time applications.

**Quick Solution Overview**

A quick solution to this problem is **image compression**.
By compressing images before transmission, you can drastically reduce data size while maintaining good image quality.
You can easily compress images in ROS 2 using `OpenCV <https://opencv.org/>`__, a powerful library for image processing.
OpenCV provides tools to encode images into various compressed formats such as JPEG and PNG.
By leveraging these features, you can significantly reduce the size of image data being transmitted over the network.
Here's how you can compress an image into JPEG format with 90% quality:

.. code-block:: cpp

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90}; // 90% JPEG quality
    cv::imencode(".jpg", image, buf, params);

The result is a compressed byte array (compressed image) that you can easily package into a ROS 2 message and publish.
This approach integrates seamlessly with the ROS 2 ecosystem and is particularly effective for applications like video streaming, where reducing bandwidth usage is critical.

Overview
--------

In a previous `tutorial <https://docs.vulcanexus.org/en/latest/rst/tutorials/core/wifi/large_data/large_data.html>`__, we discussed specific configurations of *Fast DDS* to improve video stream transmission over WiFi.
Building on that foundation, this tutorial will focus on **image compression** techniques, which play a crucial role in optimizing video transmission, especially in scenarios with limited bandwidth or congested WiFi networks.
Proper image compression can significantly reduce the data load, enhancing the stability and performance of video streaming in challenging network conditions.

This tutorial presents an example of video retransmission between a publisher and a subscriber on two different hosts via WiFi.
It compares the fluidity of the video stream and the bandwidth usage when publishing with uncompressed images versus compressed images.
By running the system with both image types, you will observe how the use of compression can reduce the load on the network and improve streaming performance.

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`__
* `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`__
* `Docker installation <https://docs.vulcanexus.org/en/latest/rst/installation/docker.html>`__

Additionally, you will need to install the following ROS 2 packages in your working environment:

* `image_transport_tutorials <https://github.com/ros-perception/image_transport_tutorials/tree/kilted?tab=readme-ov-file>`__: This package provides basic tutorials and examples on using image transport in ROS 2.
* `image_transport_plugins <https://github.com/ros-perception/image_transport_plugins>`__: This package provides the necessary plugins for different image transport transports (e.g., raw, compressed, etc.).

The steps to install these packages from sources will be provided in the :ref:`tutorials_image_compression_run_this_tutorial` section.
They will serve as a valuable guide for learning how to send compressed images and work with various image transport methods in your Vulcanexus environment.

.. _tutorials_image_compression_run_this_tutorial:

Run this tutorial
------------------

Setting Up the Workspace
^^^^^^^^^^^^^^^^^^^^^^^^

On both **Host A** and **Host B** run *Vulcanexus* Docker image with:

.. code-block:: bash

    xhost +local:docker
    docker run \
        -it \
        --privileged \
        --net host \
        --ipc host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        ubuntu-vulcanexus:kilted-desktop

And create a workspace and clone the necessary repositories so you can inspect and manipulate the code.
You'll need to install both the ``image_transport_tutorials`` and ``image_transport_plugins`` packages in the same workspace.

Create a new workspace for the tutorials:

.. code-block:: bash

    mkdir -p ~/image_transport_ws/src
    cd ~/image_transport_ws/src

Clone the ``image_transport_tutorials`` repository:

.. code-block:: bash

    git clone -b kilted https://github.com/ros-perception/image_transport_tutorials.git

Clone the ``image_transport_plugins`` repository:

.. code-block:: bash

    git clone -b kilted https://github.com/ros-perception/image_transport_plugins.git

Install Dependencies:

.. code-block:: bash

    cd ~/image_transport_ws/
    source /opt/vulcanexus/kilted/setup.bash
    sudo apt update
    rosdep install -i --from-path src --rosdistro kilted -y

Build the WorkSpace:

.. code-block:: bash

    colcon build

.. note::

    Remember to source both your ROS 2 setup file (/opt/ros/kilted/setup.bash) and the install/setup.bash file in the image_transport_ws workspace in every terminal where you will run the nodes.

Running the publisher node
^^^^^^^^^^^^^^^^^^^^^^^^^^

On **Host A**, you will run a publisher node named ``publisher_from_video``, which will publish video frames from a specified video display.
This publisher will publish frames to four different topics, each corresponding to a different type of image compression:

- ``/camera/image``: Raw image data
- ``/camera/image/compressed``: Compressed image data
- ``/camera/image/compressedDepth``: Depth-compressed image data
- ``/camera/image/theora``: Theora-compressed video stream

To start the publisher, provide the path to the video file as an argument, and the node will continuously stream frames to these topics.
This setup allows Host B to subscribe to and display the image data on any of the specified topics, depending on the desired compression format.

In general, the `CompressedPublisher` class uses OpenCV functions to encode images into compressed formats.
Here’s a high-level look at how OpenCV can be used to compress images:

1. **Convert the Image Format**: The image is first captured in a standard format like `BGR8` (Blue-Green-Red, 8-bits per channel). This raw image is typically in an OpenCV `cv::Mat` object.

2. **Choose Compression Parameters**: For each compression format (e.g., JPEG, PNG), OpenCV allows you to specify parameters like quality or compression level. These parameters control the balance between file size and image quality.

3. **Encode the Image**: The `cv::imencode` function in OpenCV is used to convert the `cv::Mat` object into a compressed format. This function takes the image format (such as `.jpg` or `.png`) and the compression parameters as arguments, and outputs a compressed byte array. For example:

.. code-block:: cpp

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90}; // 90% JPEG quality
    cv::imencode(".jpg", image, buf, params);

In this example, the image matrix is compressed as a JPEG with 90% quality, and the result is stored in ``buf``.

4. **Publish the Compressed Image:** Finally, the compressed image is packaged into a message and published. Subscribers on the other end can decode this byte stream back into an image format they can use.

To simulate a real video streaming frequency, we will adjust the publishing frequency of the publisher node ``publisher_from_video`` from 5 Hz to 30 Hz.
This adjustment will allow you to observe the advantages of compressed versus uncompressed image transmission at higher rates.
To achieve this, modify the line in ``publisher_from_video.cpp`` file that specifies the publishing rate:

.. code:: cpp

    rclcpp::WallRate loop_rate(5);

Change it to:

.. code:: cpp

    rclcpp::WallRate loop_rate(30);

Then compile again and run the publisher node:

.. code:: bash

    cd ~/image_transport_ws/
    colcon build
    ros2 run image_transport_tutorials publisher_from_video 0  # 0: Display

Running the subscriber node
^^^^^^^^^^^^^^^^^^^^^^^^^^^

On **Host B**, you will set up a subscriber node ``my_subscriber`` to receive and display the video frames published by Host A.
The ``my_subscriber`` node can subscribe to image messages with various transport types, enabling you to view the published images on your screen regardless of the compression format.

To start ``my_subscriber`` listening on ``/camera/image`` topic (default), use the following command:

.. code:: bash

    ros2 run image_transport_tutorials my_subscriber

Once the ``my_subscriber`` node is running, Host B should display the images being streamed from the publisher node on Host A.
You may notice that the display appears slightly choppy or less smooth.
This is due to the high frequency (30 Hz) of image publishing on Host A, which can create a bottleneck when transmitting large, uncompressed image data over the network.

To check the messages being sent and their bandwidth usage, source Vulcanexus in a new terminal and use the following command:

.. code:: bash

    ros2 topic bw /camera/image

This command will display the bandwidth usage of the topic, showing how much data is being transmitted per second.
For example, you might see output like this:

.. code-block::

    Subscribed to [/camera/image]
    28.51 MB/s from 29 messages
        Message size mean: 0.92 MB min: 0.92 MB max: 0.92 MB
    28.08 MB/s from 59 messages
        Message size mean: 0.92 MB min: 0.92 MB max: 0.92 MB
    27.92 MB/s from 89 messages
        Message size mean: 0.92 MB min: 0.92 MB max: 0.92 MB

This shows the transmission rate and the message size.
If you observe that the transmission rate is very high, especially for raw image data, you may notice that the WiFi bandwidth is insufficient to handle the amount of data being transmitted.
This can result in a choppy or delayed display on Host B.

Now, to improve the fluidity of the display and reduce bandwidth consumption, you can run the ``my_subscriber`` node with compressed images.
To do this, run the following command on Host B:

.. code:: bash

    ros2 run image_transport_tutorials my_subscriber --ros-args --remap __name:=compressed_listener -p _image_transport:=compressed

By using compressed images, the bandwidth usage will be significantly reduced, and you should notice a smoother display with reduced lag, even on limited network conditions

Again, to check the messages being sent and their bandwidth usage, run in a new terminal:

.. code:: bash

    ros2 topic bw /camera/image/compressed

This time you will see a reduction in the bandwidth usage:

.. code-block::

    Subscribed to [/camera/image/compressed]
    2.27 MB/s from 30 messages
        Message size mean: 0.07 MB min: 0.07 MB max: 0.07 MB
    2.24 MB/s from 60 messages
        Message size mean: 0.07 MB min: 0.07 MB max: 0.07 MB
    2.24 MB/s from 90 messages
        Message size mean: 0.07 MB min: 0.07 MB max: 0.08 MB

Conclusions
-----------

In this tutorial, we have explored the impact of image compression on video streaming performance in ROS 2, specifically in a WiFi-based setup between two hosts.
By comparing uncompressed and compressed image transmissions, you should now appreciate how compression can significantly reduce bandwidth consumption and improve fluidity in video streaming.

Using uncompressed images requires a high bandwidth, which can lead to bottlenecks and latency issues in network-constrained environments, particularly when transmitting at higher frame rates.
In contrast, compressed image formats demonstrate clear advantages by lowering the data rate and enabling smoother video playback with less lag.
This approach is particularly beneficial in applications where network resources are limited or shared.

By applying these compression techniques, you can enhance the stability and efficiency of video streaming in ROS 2 environments, making it a valuable practice for real-time applications in robotics and other bandwidth-sensitive fields.
You will find more approaches to improve your large data sending over WiFi in the rest of tutorials of this `section <https://docs.vulcanexus.org/en/latest/rst/tutorials/core/wifi/wifi_tutorials.html>`__.
