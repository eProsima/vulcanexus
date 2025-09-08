.. _vulcanexus_hri_introduction:

Vulcanexus HRI Overview
=======================

**Human-Robot Interaction** (HRI) is a multidisciplinary research field that studies how humans and robots can effectively perceive, communicate, and collaborate.
It encompasses a wide range of domains, including computer vision, natural language processing, emotion recognition, and behavioral modeling.
The central goal of HRI is to enable robots to interact with humans in a natural, safe, and socially acceptable manner.

Vulcanexus provides a set of tools and libraries to facilitate the development of Human-Robot Interaction applications, leveraging the latest capabilities of Fast DDS to smooth the integration of HRI applications within the ROS 2 ecosystem.
This section introduces the key concepts and components of HRI within the Vulcanexus framework.

HRI Components
--------------

Vulcanexus HRI library includes the following components:

- **Face detection**: A module for detecting and recognizing human faces in images and video streams.
- **Human pose recognition**: A module for recognizing human body poses and gestures.
- **Emotion recognition**: A module for recognizing human emotions based on facial expressions and body language.
- **Displays**: A module for rendering visual information on screens and other display devices.
- **Speech-to-Text (STT)**: A module for converting spoken language into text.
- **Text-to-Speech (TTS)**: A module for converting text into spoken language

These components can be used individually or combined to create complex HRI applications.

Getting Started with HRI
------------------------

To get started with HRI in Vulcanexus, you can follow the tutorials provided in the :ref:`Vulcanexus HRI Tutorials <tutorials_hri_hri_tutorials>` section.
These tutorials provide step-by-step guidance on how to set up and use the various HRI components within the Vulcanexus framework.

.. _vulcanexus_hri_face_detect:

Face Detection
---------------

Face detection is accomplished by the `*hri_face_detect* <https://github.com/eProsima/agile-hri>`_ package.
It relies on `YuNet face detector <https://github.com/ShiqiYu/libfacedetection>`_ to perform face detection.
This Convolutional Neural Network (CNN) model is optimized for real-time applications and can run efficiently on CPU.
Hence, there is no need to set up any GPU acceleration for this package.

This package optimizes the `ROS REP-155 <https://ros.org/reps/rep-0155.html>`_ by introducing `keys <https://docs.ros.org/en/rolling/Tutorials/Advanced/Topic-Keys/Topic-Keys-Tutorial.html>`, only compatible with `Fast DDS <https://github.com/eProsima/Fast-DDS>`_.
By using keyed topics, this package is able to provide face recognition for multiple targets by just publishing in one topic, which significantly reduces the network load.

Face detections are published as ``hri_msgs/msg/Face2DList`` messages on a keyed topic name ``/humans/faces``.
**Face2DList** message is part of the `hri_msgs <https://github.com/eProsima/agile-hri>`_ package.
It consists of a list of ``hri_msgs/msg/Face2D`` messages - each containing information about a detected face, such as the landmarks and unique ID - and a list of ``hri_msgs/msg/NormalizedRegionOfInterest2D`` messages representing the boundary box of the face.
Face landmarks refer to the following specific points on a human face:

- Left eye
- Right eye
- Nose
- Left mouth corner
- Right mouth corner

The unique ID is an identifier assigned to each detected face, which can be used to track the same face across multiple frames.
It is provided via a ROS 2 service by the `*hri_id_manager* <https://github.com/eProsima/agile-hri>`_ node, which is responsible for managing the IDs of the detected faces and ensuring that the same ID is assigned to the same person across different HRI packages, such as :ref:`vulcanexus_hri_pose_detect`.
