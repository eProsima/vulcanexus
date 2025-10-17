.. _tutorials_hri_emotion_detect:

Vulcanexus HRI Emotion Detection Tutorial
=========================================

Quickview
---------

This tutorial will explore the process of implementing emotion recognition capabilities within the Vulcanexus framework.
It will cover the necessary steps to set up the environment, integrate the emotion recognition model and visualize the results in RViz2.

A deeper explanation of the emotion detection package can be found in the :ref:`vulcanexus_hri_emotion_detect` section.

Prerequisites
-------------

Before starting this tutorial, ensure you have the following prerequisites:

- Face detection tutorial completed (:ref:`tutorials_hri_face_detect`).

Initial Setup
-------------

Vulcanexus Emotion Detection package depends on the Face Detection package (:ref:`tutorials_hri_face_detect`).
Hence, it is recommended to complete the face detection tutorial before proceeding with this one as it covers the initial setup steps.

This tutorial will continue from the end of the face detection tutorial, where there was a node publishing images on the network, as well as the face detection and ID manager nodes running.

Run the Emotion Detection node
------------------------------

To run the emotion detection node create a new ``emotion-compose.yaml`` file (or add the service to the existing ``compose.yaml`` file) with the following content:

.. code-block:: yaml

    x-common-config:
       &common-config
        network_mode: host
        ipc: host

    x-common-variables:
        &common-variables
        ROS_DOMAIN_ID: 42

    services:
        emotion_detect:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            environment:
                <<: *common-variables
            profiles: ["emotion"]
            command: >
            ros2 launch hri_emotion_detect emotion_detect.launch.py rgb_camera_topic:=/color/image_raw

And run the following command in the same directory where the file is located:

.. code-block:: bash

    docker compose -f emotion-compose.yaml --profile emotion up -d

This command will start the ``hri_emotion_detect`` node in a new container.
The node will start publishing detected emotions on the ``/humans/faces/emotions`` topic, which will be automatically detect by the detection display node if it is running, and will update the visualization in RViz2 to show the detected emotions.


.. note::

    If you are running the tutorial on a native installation of Vulcanexus, you can run the nodes in separate terminals instead of using Docker Compose.
    Just make sure to source the Vulcanexus installation in each terminal before running the nodes and use the ``emotion-compose.yaml`` file as a reference for the commands to run each node.
