.. _tutorials_hri_pose_detect:

Vulcanexus HRI Pose Detection Tutorial
======================================

Quickview
---------

This tutorial will explore the process of implementing pose detection capabilities within the Vulcanexus framework.
It will cover the necessary steps to set up the environment, integrate the pose detection model and visualize the results in RViz2.

A deeper explanation of the pose detection package can be found in the :ref:`vulcanexus_hri_pose_detect` section.

Prerequisites
-------------

Before starting this tutorial, ensure you have the following prerequisites:

- A working installation of Vulcanexus with `HRI modules <https://docs.vulcanexus.org/en/latest/rst/introduction/metapackages.html#vulcanexus-hri>`_.
- Docker and Docker Compose installed on your system.
- Familiarity with ROS 2 concepts and tools.
- Webcam or a ROS 2 compatible video input device.
- GPU hardware to load the pose detection model.

Initial Setup
-------------

The tutorial will make use of the Vulcanexus docker image in order to simplify the setup process.
However, it is also possible to run the tutorial on a native installation of Vulcanexus.

In order to run the tutorial, it is also necessary to have a ROS 2 node publishing images on a known topic.
Images are expected to be in ``sensor_msgs/msg/Image`` format and topic ``/color/image_raw``.
The next section describes how to set up a simple image publisher using a webcam.
If you already have a node publishing images, you can skip this section and proceed to the next one (:ref:`tutorials_hri_face_detect_run`).

Use a webcam as image source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case of using Docker, it is needed to give access to the webcam device inside the container.
Usually, the webcam is located at ``/dev/video0``.
Run Vulcanexus docker image with the following command, replacing ``{VULCANEXUS_DISTRO}`` with your Vulcanexus distribution:

.. code-block:: bash

    docker run --device /dev/video0 -it --rm --net host --ipc host ubuntu-vulcanexus:{VULCANEXUS_DISTRO}-hri

To publish images from a webcam, you can use the `*usb_cam* <https://github.com/ros-drivers/usb_cam>`_ package.
Install it with the following command:

.. code-block:: bash

    apt update && \
    apt install -y ros-${VULCANEXUS_DISTRO}-usb-cam

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

.. _tutorials_hri_pose_detect_run:

Run the Pose Detection Node
---------------------------

Now that there is a node publishing images on the network, we can proceed to run the pose detection node.

The pose detection node subscribes to a topic of type ``sensor_msgs/msg/Image`` and publishes the detected bodies as ``hri_msgs/msg/Skeleton2DList`` messages on a keyed topic name ``/humans/bodies``.

Similarly to the face recognition module, ``hri_pose_detect`` package requires to also run the ``hri_id_manager`` node, which is responsible for managing the IDs of the detected poses.
The ID manager provides a service that assigns a unique ID to each detected pose.
If the ``hri_face_detect`` node is also running, the ID manager will ensure that the same ID is assigned to the same person across both packages.

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
            image: ubuntu-vulcanexus:{VULCANEXUS_DISTRO}-hri
            <<: *common-config
            environment:
                <<: *common-variables
            command: >
                ros2 launch hri_id_manager id_manager.launch.py

        pose_detect_2D:
            image: ubuntu-vulcanexus:{VULCANEXUS_DISTRO}-hri
            <<: *common-config
            environment:
                <<: *common-variables
            deploy:
                resources:
                    reservations:
                        devices:
                        - driver: nvidia
                          count: 1
                          capabilities: [gpu]
            command: >
                ros2 launch hri_pose_detect pose_detect.launch.py rgb_camera_topic:=/color/image_raw

        # Run ($xhost local:root) before starting this container to be able to display RViz2
        detection_display_2D:
            image: ubuntu-vulcanexus:{VULCANEXUS_DISTRO}-hri
            <<: *common-config
            privileged: true
            volumes:
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            environment:
                <<: *common-variables
                DISPLAY: ":1"
                LIBGL_ALWAYS_SOFTWARE: 1
            profiles: ["display"]
            command: >
                ros2 launch hri_detection_display person_detection_display.launch.py rgb_camera_topic:=/color/image_raw rviz:=True

        # Not needed if there is already a node publishing images on /color/image_raw
        local_webcam:
            image: ubuntu-vulcanexus:{VULCANEXUS_DISTRO}-hri
            <<: *common-config
            environment:
                <<: *common-variables
            devices:
            - /dev/video0
            profiles: ["usb_cam"]
            command: >
                ros2 run usb_cam usb_cam_node_exe --ros-args -r /image_raw:=/color/image_raw -r /image_raw/compressed:=/color/image_raw/compressed -r /camera_info:=/color/camera_info

.. note::

  The ``ROS_DOMAIN_ID`` environment variable at the top of the file can be used to set the ROS 2 domain ID for all the nodes.

To start the pose detection node along with the ID manager, run the following command in the same directory where the ``compose.yaml`` file is located:

.. code-block:: bash

    docker compose up -d

At this point, the pose detection node should be running and publishing the detected bodies on the ``/humans/bodies`` topic.
To verify it, attach a new terminal to one of the running containers and subscribe to the topic:

.. code-block:: bash

    docker exec -it "$(docker ps --quiet | head -n 1)" bash
    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash
    ros2 topic list
    ros2 topic echo /humans/bodies --no-arr

.. warning::

  If there are other docker containers running in your system, ``"$(docker ps --quiet | head -n 1)"`` should be replaced with the container ID of one of the Vulcanexus containers to avoid attaching to the wrong container.

If bodies are being detected correctly, you should see the output of the ``ros2 topic echo`` command updating at 30 Hz with the list of detected bodies.

However, the most intuitive way to visualize the detected bodies is by using RViz2.
To do so, Vulcanexus includes an HRI package called ``hri_detection_display`` that provides a convenient way to visualize the detected bodies in RViz2.

This package subscribes to the ``/humans/bodies`` topic and displays the detected bodies as bounding boxes over the input images.
It also paints the skeleton of each detected body by connecting visible keypoints with lines.
To execute it, run the following command in the same terminal where the ``compose.yaml`` file is located:

.. code-block:: bash

    docker compose --profile display up -d

This command will start the ``hri_detection_display`` node in a new container and launch RViz2 to visualize the detected faces.

To stop all the running containers, use the following command:

.. code-block:: bash

    docker compose --profile display --profile usb_cam down

.. note::

    If you are running the tutorial on a native installation of Vulcanexus, you can run the nodes in separate terminals instead of using Docker Compose.
    Just make sure to source the Vulcanexus installation in each terminal before running the nodes and use the ``compose.yaml`` file as a reference for the commands to run each node.

Use 3D Pose Detection
---------------------

The ``hri_pose_detect`` package also supports a 3D pose detection model that can be used to detect human bodies in 3D space.

To use the 3D model, the input images must be accompanied by depth images and camera information.
The depth images must be published on a topic of type ``sensor_msgs/msg/Image`` and name ``/depth/image_raw``.
Conventional webcams do not provide depth information, so specialized hardware is required to continue with this section.

To run the 3D pose detection node, substitute the ``pose_detect_2D`` service in the ``compose.yaml`` file as follows:

.. code-block:: yaml

    pose_detect_3D:
        image: ubuntu-vulcanexus:{VULCANEXUS_DISTRO}-hri
        <<: *common-config
        environment:
            <<: *common-variables
        deploy:
            resources:
                reservations:
                    devices:
                    - driver: nvidia
                      count: 1
                      capabilities: [gpu]
        command: >
            ros2 launch hri_pose_detect pose_detect.launch.py use_depth:=True rgb_camera_topic:=/color/image_raw depth_camera_topic:=/depth/image_raw

Similarly, the ``hri_detection_display`` package includes an extension to support 3D display, using ROS 2 markers to represent human bodies.
To use this feature replace the ``detection_display_2D`` service as follows:

.. code-block:: yaml

    detection_display_3D:
        image: ubuntu-vulcanexus:{VULCANEXUS_DISTRO}-hri
        <<: *common-config
        privileged: true
        volumes:
        - /tmp/.X11-unix:/tmp/.X11-unix:rw
        environment:
            <<: *common-variables
            DISPLAY: ":1"
            LIBGL_ALWAYS_SOFTWARE: 1
        profiles: ["display"]
        command: >
            ros2 launch hri_detection_display skel_3D_display.launch.py pub_static_tf:=True

After modifying the ``compose.yaml`` file, re-run the previous command to start the 3D pose detection node along with the ID manager:

.. code-block:: bash

    docker compose --profile display up -d

.. note::

    If you are running the tutorial on a native installation of Vulcanexus, you can run the nodes in separate terminals instead of using Docker Compose.
    Just make sure to source the Vulcanexus installation in each terminal before running the nodes and use the ``compose.yaml`` file as a reference for the commands to run each node.
