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

- **HRI Msgs**: A package that defines the custom message types used in Vulcanexus HRI applications.
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

.. _vulcanexus_hri_msgs:

HRI Messages
------------

The ``hri_msgs`` package represents the cornerstone of the Human-Robot Interaction (HRI) modules distributed within *Vulcanexus*.
It originates from the `ROS4HRI <https://github.com/ros4hri>`_ initiative and has been extended to provide a more robust and efficient framework for implementing HRI applications in ROS 2.

One of the key innovations integrated into Vulcanexus ``hri_msgs`` is the use of **Fast DDS keyed topics**, a mechanism that introduces a unique identifier, or *key*, to each message instance.
Keys enable multiple objects of the same logical kind to be published within a single topic, rather than requiring the creation of multiple topics for each individual object.
For more information about keyed topics, please refer to the :ref:`topic_keys` section.
This design significantly reduces the number of middleware entities, minimizes discovery overhead, and optimizes bandwidth consumption.

Within the HRI ecosystem, this feature proves particularly advantageous. For example:

- In :ref:`vulcanexus_hri_face_detect`, faces are represented as ``Face2D`` keyed messages, each instance corresponding to a unique face ID.
- In :ref:`vulcanexus_hri_pose_detect`, detected body skeletons are published as ``Skeleton2D`` keyed messages, with each instance tied to a specific individual.

By centralizing all HRI-related message definitions, ``hri_msgs`` simplifies development and ensures consistency across the various perception and interaction modules.
The integration of Fast DDS keys allows nodes to manage multiple entities within a single stream, avoiding redundant topics, reducing computational load, and ensuring scalability for real-world applications where many humans may interact simultaneously with a robot.

More information about the message definitions and the enhancements introduced in this package can be found in :ref:`enhancements_hri_msgs`.

.. _vulcanexus_hri_face_detect:

Face Detection
--------------

Face detection is accomplished by the `hri_face_detect <https://github.com/eProsima/agile-hri>`_ package.
It relies on `YuNet face detector <https://github.com/ShiqiYu/libfacedetection>`_ to perform face detection.
This Convolutional Neural Network (CNN) model is optimized for real-time applications and can run efficiently on CPU.
Hence, there is no need to set up any GPU acceleration for this package.

This package optimizes the `ROS REP-155 <https://ros.org/reps/rep-0155.html>`_ by introducing `keys <https://docs.ros.org/en/rolling/Tutorials/Advanced/Topic-Keys/Topic-Keys-Tutorial.html>`_, only compatible with `Fast DDS <https://github.com/eProsima/Fast-DDS>`_.
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
It is provided via a ROS 2 service by the `hri_id_manager <https://github.com/eProsima/agile-hri>`_ node, which is responsible for managing the IDs of the detected faces and ensuring that the same ID is assigned to the same person across different HRI packages, such as the :ref:`vulcanexus_hri_pose_detect` package.

.. _vulcanexus_hri_pose_detect:

Human Pose recognition
----------------------

Human pose recognition is accomplished by the `hri_pose_detect <https://github.com/eProsima/agile-hri>`_ package.
It relies on `YOLOv8-pose <https://docs.ultralytics.com/models/yolov8/>`_ to detect and track human poses in real-time.
The package is designed to work seamlessly within the ROS 2 ecosystem and can be easily integrated with other HRI components.
This model, however, requires GPU acceleration to run efficiently.

Following the same line of development as the ``hri_face_detect`` package, this library also optimizes the `ROS REP-155 <https://ros.org/reps/rep-0155.html>`_ by introducing `keys <https://docs.ros.org/en/rolling/Tutorials/Advanced/Topic-Keys/Topic-Keys-Tutorial.html>`_, only compatible with `Fast DDS <https://github.com/eProsima/Fast-DDS>`_.
By using keyed topics, this package is able to provide pose recognition for multiple targets by just publishing in one topic, which significantly reduces the network load.

Pose or body detections are published as ``hri_msgs/msg/Skeleton2DList`` messages on a keyed topic name ``/humans/bodies``.
**Skeleton2DList** message is part of the `hri_msgs <https://github.com/eProsima/agile-hri>`_ package.
It consists of a list of ``hri_msgs/msg/Skeleton2D`` messages - each containing information about a detected body, such as the landmarks and unique ID - and a list of ``hri_msgs/msg/NormalizedRegionOfInterest2D`` messages representing the boundary box of the body.
An additional list of floats is also included, representing the mean depth of the body detected, in case depth information is available.

``hri_pose_detect`` also supports 3D pose detection by leveraging depth information from RGB-D cameras.
When enabled, the package computes the 3D coordinates of each keypoint using the camera's intrinsic parameters and the depth value at the keypoint's location.
An additional message is then published on topic ``/humans/bodies/skel3D`` containing a list of ``hri_msgs/msg/Skeleton3D`` messages with the 3D coordinates of each keypoint.
Leveraging depth information and YOLO advanced tracking capabilities, this package is able to accurately estimate the orientation of the detected bodies.

Body landmarks refer to the following specific points on a human body:

- Nose
- Neck
- Right Shoulder
- Right Elbow
- Right Wrist
- Left Shoulder
- Left Elbow
- Left Wrist
- Right Hip
- Right Knee
- Right Ankle
- Left Hip
- Left Knee
- Left Ankle
- Left Eye
- Right Eye
- Left Ear
- Right Ear

Similarly to the Face Detection module, the unique ID is an identifier assigned to each detected body, which allows to track the same body across multiple frames.
It is also provided via a ROS 2 service by the `hri_id_manager <https://github.com/eProsima/agile-hri>`_ node, which is responsible for managing the IDs of the detected bodies and ensuring that the same ID is assigned to face detected by the :ref:`vulcanexus_hri_face_detect` package when run simultaneously.

.. _vulcanexus_hri_emotion_detect:

Emotion Recognition
-------------------

Emotion recognition is accomplished by the `hri_emotion_detect <https://github.com/eProsima/agile-hri>`_ package.
`OpenCV Zoo Facial Expression Recognition model <https://github.com/opencv/opencv_zoo/tree/main/models/facial_expression_recognition>`_ is used as main model to perform emotion recognition.

The output of the emotion recognition module is published as ``hri_msgs/msg/Expression`` messages on the topic ``/humans/faces/emotion``.
This message contains a string describing the recognized emotion for a specific individual, identified by their unique ID, which is used as key of the message.
The message also includes a float value representing the confidence score of the recognition, ranging from 0 to 1.

This package can be considered as an extension of the :ref:`vulcanexus_hri_face_detect` package, as it requires the face detection module to be running in order to obtain the face images for emotion recognition.
When both packages are run simultaneously, the same unique ID is assigned to the same person across both packages, allowing to correlate the detected faces with their recognized emotions, which simplifies representantion and visualization of the results.

.. _vulcanexus_hri_detection_display:

HRI Displays
------------

Vulcanexus provides an optional display module to visualize the results of the HRI components in RViz2.
The `hri_detection_display <https://github.com/eProsima/agile-hri>`_ package is responsible for this functionality.

It includes specific nodes to parse the data received from the HRI components, merge it and render it in RViz2 in a user-friendly manner.
The display module can visualize the following information:

- Detected faces and unique IDs.
- Detected bodies with landmarks and unique IDs.
- Recognized emotions associated with detected faces.

The display module subscribes to the topics published by the HRI components and updates the visualization in real-time as new data is received.
It has several parameters to customize the visualization, such as enabling/disabling specific components or dynamically applying filters to control what is displayed (i.e., disallow back turned people, allow half body detections, etc.).
It is the fastest and easiest way to visualize the output of the HRI components and can be used for debugging, testing, or demonstration purposes.

.. _vulcanexus_hri_stt:

Speech-to-Text (STT)
--------------------

The Speech-to-Text (STT) module is provided by the `hri_stt <https://github.com/eProsima/agile-hri>`_ package.
It relies on `faster-whisper <https://github.com/SYSTRAN/faster-whisper>`_ as speech recognition model.
Thanks to its efficient design and low resource consumption, this model can run efficiently locally.
Nevertheless, a **Nvidia GPU is required** for inferences, as it depends on the *CUBLAS* and *CUDNN* libraries.

The STT module wraps this model into a ROS 2 node that implements a ROS 2 action server.
This server receives real-time audio streams and produces transcriptions.
This package also incorporates voice activity detection (VAD) functionalities to detect when users are speaking and when they are silent.
This feature helps to improve the accuracy of the transcriptions by reducing background noise and irrelevant audio data, but also automatically detecting when the speech has ended, allowing to send the transcription result back to the client.
Although it includes a default VAD model responsible for this task, the package can also operate with microphones that feature integrated VAD solutions.
The default model is based on `Silero VAD <https://github.com/snakers4/silero-vad>`_.
By leveraging VAD, the STT module can segment the audio input in real-time whenever short pauses are detected, enabling it to speed up the overall transcription process while maintaining contextual accuracy.

The STT action server is designed to auto configure itself based on the characteristics of the audio input source, simplifying the setup process to its maximum, which makes it ideal for rapid integration in robotic applications.

This package also includes utility scripts to download the required models or list the available audio input devices, to facilitate the initial setup.

.. _vulcanexus_hri_tts:

Text-to-Speech (TTS)
--------------------

The Text-to-Speech (TTS) module is provided by the `hri_tts <https://github.com/eProsima/agile-hri>`_ package.
It relies on `TTS models from Coqui <https://coqui.ai/>`_ for speech synthesis.
Although these models can run on CPU, a **Nvidia GPU is recommended** to achieve optimal performance and obtain real-time speech synthesis.

The TTS package includes three different nodes which serve different purposes:

- A TTS action server that receives text input and converts it to speech, publishing the generated audio as an Audio message on a ROS 2 topic.
- A TTS subscriber node that listens to the audio topic and plays the received audio through a specified speaker.
- Another version of the TTS action server that receives text input and converts it to speech, but storing the result in ``.wav`` file on the path specified.
  This node is useful for offline processing or when the audio needs to be stored for later use, for example when using *presets* (check :ref:`tutorials_hri_tts_presets`).

This package also includes utility scripts to download the required model or list the available audio output devices, to facilitate the initial setup.

TTS publisher
^^^^^^^^^^^^^

The TTS publisher is the main node of the package, as is the one responsible for running the TTS model and generating the speech.
It implements a ROS 2 action server that receives text input and converts it to speech.
The generated speech is then published as an Audio message on the ``/hri_tts/audio`` topic.
Note that the TTS publisher does not play the generated audio, nor sends the audio data back to the client that requested the speech synthesis.
The audio is published on a different topic in order to decouple the speech synthesis from the audio playback, allowing the user to choose how to play the generated audio without blocking the action server.

The parameter ``wait_for_finished`` is available to control whether the action server should wait for the audio playback to finish before sending the response back to the client.
By default, this parameter is set to ``True``, meaning that the action server will wait for the audio to finish playing before responding with ``SUCCEEDED``.
It recognizes the end of the audio playback by subscribing to the topic ``/hri_tts/finished``.
Hence, the TTS subscriber node must be running and publishing ``True`` on this topic when the audio playback is complete for the action server to acknowledge it and be able to respond with ``SUCCEEDED``.
Otherwise, if no message is received on this topic within a timeout period, the action server will respond with ``ABORTED``.
The timeout period is calculated based on the duration of the audio generated and 5 additional seconds.

If the parameter is set to ``False``, the action server will respond inmediately after the audio is published, without waiting for the playback to finish.
The user can choose the desired behavior based on the specific requirements of their application and workflow.

TTS subscriber
^^^^^^^^^^^^^^

The TTS subscriber is an optional node that can be used to play the generated audio through a specified speaker.
It subscribes to the ``/hri_tts/audio`` topic and plays the received audio.
It only requires to specify the audio output device to be used for playback, which can be listed using the included utility script.

The TTS package also includes extra features to enhance its performance and usability, like the ability to use *presets* when generating the speech.
Presets are already created audio files (stored locally) that can be directly played instead of generating the speech from scratch.
This feature is useful when some audios need to be played repeatedly, as it avoids the need to generate the same audio multiple times, saving both time and computational resources.
Check :ref:`tutorials_hri_tts_presets` for more information about this feature and how to implement it.

This node is designed to work seamlessly with the TTS publisher, but it can also be used independently to play audio from other sources, as long as the audio is published as an Audio message on the ``/hri_tts/audio`` topic.

TTS Generator
^^^^^^^^^^^^^

Lastly, the TTS package includes another version of the TTS action server that generates the speech and stores it in a ``.wav`` file on the specified path.
This node is useful for offline processing or when the audio needs to be stored for later use.
The logic and configuration of this node is similar to the TTS publisher, but instead of publishing the generated audio on a topic, it saves it in a file.
The path used to store the file can be specified by ROS 2 parameters.
