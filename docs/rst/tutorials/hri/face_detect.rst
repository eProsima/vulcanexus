.. _tutorials_hri_face_detect:

Vulcanexus HRI Face Detection Tutorial
======================================

Quickview
---------

This tutorial will explore the process of implementing face detection capabilities within the Vulcanexus framework.
It will cover the necessary steps to set up the environment, integrate the face detection model and visualize the results in RViz2.

A deeper explanation of the face detection package can be found in the :ref:`vulcanexus_hri_face_detect` section.

Prerequisites
-------------

Before starting this tutorial, ensure you have the following prerequisites:

- A working installation of Vulcanexus with `HRI modules <https://docs.vulcanexus.org/en/latest/rst/introduction/metapackages.html#vulcanexus-hri>`_.
- Docker and Docker Compose installed on your system.
- Familiarity with ROS 2 concepts and tools.
- Webcam or a ROS 2 compatible video input device.

Initial Setup
-------------

The tutorial will make use of the Vulcanexus docker image in order to simplify the setup process.
However, it is also possible to run the tutorial on a native installation of Vulcanexus.

In order to run the tutorial, it is also necessary to have a ROS 2 node publishing images on a known topic.
Images are expected to be in ``sensor_msgs/msg/Image`` format and topic ``/color/image_raw``.
The next section describes how to set up a simple image publisher using a webcam.
If you already have a node publishing images, you can skip this section and proceed to the next one (:ref:`tutorials_hri_face_detect_run`).

To improve communication performance, we will increase sockets buffer sizes by setting an XML configuration file before running any ROS 2 nodes.
First, ensure your system has enabled system sockets buffer sizes of 11 MB or higher (see `this guide <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html#finding-out-system-maximum-values>`_).
Then, download the following XML file and save it as ``~/image.xml``:

.. code-block:: bash

   wget https://raw.githubusercontent.com/eProsima/agile-hri/refs/heads/main/hri_detection_display/config/image.xml -O ~/image.xml

Then, export the following environment variable to every terminal where ROS 2 nodes will be run:

.. code-block:: bash

   export FASTDDS_DEFAULT_PROFILES_FILE="~/image.xml"

Use a webcam as image source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case of using Docker, it is needed to give access to the webcam device inside the container.
Usually, the webcam is located at ``/dev/video0``.
Run Vulcanexus docker image with the following command, replacing ``<VULCANEXUS_DISTRO>`` with your Vulcanexus distribution:

.. code-block:: bash

    docker run --device /dev/video0 -it --rm --net host --ipc host ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri

To publish images from a webcam, you can use the `usb_cam <https://github.com/ros-drivers/usb_cam>`_ package.
Install it with the following command:

.. code-block:: bash

    apt update && \
    apt install -y ros-${VULCANEXUS_DISTRO}-usb-cam

.. note::

  Package *usb_cam* is already included in the Vulcanexus Docker image.

After sourcing the Vulcanexus installation, run the ``usb_cam`` node to publish images from the webcam:

.. code-block:: bash

    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && \
    ros2 run usb_cam usb_cam_node_exe --ros-args -r /image_raw:=/color/image_raw -r /image_raw/compressed:=/color/image_raw/compressed -r /camera_info:=/color/camera_info

After this command, the node should be publishing 640x480 images on the ``/image_raw`` topic at 30 frames per second.
To check it, attach a new terminal to the running container and subscribe to the topic following the steps below:

.. code-block:: bash

    docker exec -it "$(docker ps --quiet | head -n 1)" bash
    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash
    ros2 topic list
    ros2 topic echo /color/image_raw --no-arr

.. warning::

  If there are other docker containers running in your system, ``"$(docker ps --quiet | head -n 1)"`` should be replaced with the container ID of the Vulcanexus container to avoid attaching to the wrong container.

If images are being published correctly, you should see the output of the ``ros2 topic echo`` command updating at 30 Hz.
At this point, this node can be left running in the background while proceeding to the next section.
Nonetheless, if you prefer to run the entire tutorial using only Docker and one terminal, you can stop the Docker container and enable the ``local_webcam`` service included in the ``compose.yaml`` file used in the following section.

.. _tutorials_hri_face_detect_run:

Run the Face Detection Node
---------------------------

Now that there is a node publishing images on the network, we can proceed to run the face detection node.

The face detection node subscribes to a topic of type ``sensor_msgs/msg/Image`` and publishes the detected faces as ``hri_msgs/msg/Face2DList`` messages on a keyed topic name ``/humans/faces``.

``hri_face_detect`` package requires to also run the ``hri_id_manager`` node, which is responsible for managing the IDs of the detected faces.
The ID manager provides a service that assigns a unique ID to each detected face.
If the ``hri_pose_detect`` node is also running, the ID manager will ensure that the same ID is assigned to the same person across both packages.

To run both nodes simultaneously, create a ``compose.yaml`` file with the following content:

.. code-block:: yaml

    x-common-config:
       &common-config
        network_mode: host
        ipc: host

    x-common-variables:
        &common-variables
        ROS_DOMAIN_ID: 42

    services:
        id_manager:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            environment:
                <<: *common-variables
            command: >
                ros2 launch hri_id_manager id_manager.launch.py

        face_detect:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            environment:
                <<: *common-variables
                FASTDDS_DEFAULT_PROFILES_FILE: ~/image.xml
            command: >
                ros2 launch hri_face_detect face_detect.launch.py rgb_camera_topic:=/color/image_raw

        # Run ($xhost local:root) before starting this container to be able to display RViz2
        detection_display_2D:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            privileged: true
            volumes:
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            environment:
                <<: *common-variables
                DISPLAY: ":1"
                LIBGL_ALWAYS_SOFTWARE: 1
                FASTDDS_DEFAULT_PROFILES_FILE: ~/image.xml
            profiles: ["display"]
            command: >
                ros2 launch hri_detection_display person_detection_display.launch.py rgb_camera_topic:=/color/image_raw rviz:=True

        # Not needed if there is already a node publishing images on /color/image_raw
        local_webcam:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            environment:
                <<: *common-variables
                FASTDDS_DEFAULT_PROFILES_FILE: ~/image.xml
            devices:
            - /dev/video0
            profiles: ["usb_cam"]
            command: >
                ros2 run usb_cam usb_cam_node_exe --ros-args -r /image_raw:=/color/image_raw -r /image_raw/compressed:=/color/image_raw/compressed -r /camera_info:=/color/camera_info

.. note::

  The ``ROS_DOMAIN_ID`` environment variable at the top of the file can be used to set the ROS 2 domain ID for all the nodes.

To start the face detection node along with the ID manager, run the following command in the same directory where the ``compose.yaml`` file is located:

.. code-block:: bash

    docker compose up -d

At this point, the face detection node should be running and publishing the detected faces on the ``/humans/faces`` topic.
To verify it, attach a new terminal to one of the running containers and subscribe to the topic:

.. code-block:: bash

    docker exec -it "$(docker ps --quiet | head -n 1)" bash
    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash
    ros2 topic list
    ros2 topic echo /humans/faces --no-arr

.. warning::

  If there are other docker containers running in your system, ``"$(docker ps --quiet | head -n 1)"`` should be replaced with the container ID of one of the Vulcanexus containers to avoid attaching to the wrong container.

If faces are being detected correctly, you should see the output of the ``ros2 topic echo`` command updating at 30 Hz with the list of detected faces.

However, the most intuitive way to visualize the detected faces is by using RViz2.
To do so, Vulcanexus includes an HRI package called ``hri_detection_display`` that provides a convenient way to visualize the detected faces in RViz2.

This package subscribes to the ``/humans/faces`` topic and displays the detected faces as bounding boxes over the input images.
To execute it, run the following command in the same terminal where the ``compose.yaml`` file is located:

.. code-block:: bash

    docker compose --profile display up -d

This command will start the ``hri_detection_display`` node in a new container and launch RViz2 to visualize the detected faces.

The ``compose.yaml`` file includes an optional service ``local_webcam`` which can be used to publish images from a webcam, alternatively to the setup explained in the previous section.
It is provided in case the user wants to run the entire tutorial using only Docker and one terminal.
To enable it, run the following command instead:

.. code-block:: bash

    docker compose --profile display --profile usb_cam up -d

To stop all the running containers, use the following command:

.. code-block:: bash

    docker compose --profile display --profile usb_cam down

.. note::

    If you are running the tutorial on a native installation of Vulcanexus, you can run the nodes in separate terminals instead of using Docker Compose.
    Just make sure to source the Vulcanexus installation in each terminal before running the nodes and use the ``compose.yaml`` file as a reference for the commands to run each node.
