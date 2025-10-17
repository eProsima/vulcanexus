.. _tutorials_hri_tts:

Vulcanexus HRI Text-to-Speech (TTS) Tutorial
============================================

Quickview
---------

This tutorial will explore the process of implementing text-to-speech capabilities within the Vulcanexus framework.
It will cover the necessary steps to set up the environment, integrate the TTS model, and visualize the results in RViz2.

The TTS package includes three different nodes which serve different purposes:

- A TTS action server that receives text input and converts it to speech, publishing the generated audio as an Audio message on a ROS 2 topic.
- A TTS subscriber node that listens to the audio topic and plays the received audio through a specified speaker.
- Another version of the TTS action server that receives text input and converts it to speech, but storing the result in ``.wav`` file on the path specified.
  This node is useful for offline processing or when the audio needs to be stored for later use, for example when using *presets* (check :ref:`tutorials_hri_tts_presets`).

A deeper explanation of the TTS package can be found in the :ref:`vulcanexus_hri_tts` section.

Prerequisites
-------------

Before starting this tutorial, ensure you have the following prerequisites:

- A working installation of Vulcanexus with `HRI modules <https://docs.vulcanexus.org/en/latest/rst/introduction/metapackages.html#vulcanexus-hri>`_.
- Docker and Docker Compose installed on your system.
- GPU with CUDA support (not mandatory, but recommended).
- Familiarity with ROS 2 concepts and tools.
- A speaker or a compatible audio output device.

Initial Setup
-------------

The tutorial will make use of the Vulcanexus docker image in order to simplify the setup process.
However, it is also possible to run the tutorial on a native installation of Vulcanexus.

The TTS package in Vulcanexus is designed to convert text into speech in real-time.
Similarly to the TTS package, no AI model is included in the default Vulcanexus installation due to their size, so it is necessary to download the models before running the tutorial.
This applies to both Vulcanexus apt package and the docker image.

The structure of this package and tutorial is similar to the TTS ones, following the same logic and steps needed to set up the environment.
The following sections describe how to select the audio output device and download the models.

.. _tutorial_hri_tts_docker_setup:

Docker setup
^^^^^^^^^^^^

Create a ``tts-compose.yaml`` file with the following content, replacing ``<VULCANEXUS_DISTRO>`` with your Vulcanexus distribution:

.. code-block:: yaml

    x-common-config:
       &common-config
        network_mode: host
        ipc: host

    x-common-variables:
        &common-variables
        ROS_DOMAIN_ID: 42

    services:
        tts_publisher:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            deploy:
                resources:
                    reservations:
                        devices:
                        - driver: nvidia
                          count: 1
                          capabilities: [gpu]
            environment:
                <<: *common-variables
            volumes:
                - /home/user/.local/share/tts:/root/.local/share/tts    # Coqui TTS models
            profiles: ["tts_pub"]
            command: >
                ros2 launch hri_tts tts.launch.py wait_for_finished:=False

        tts_subscriber:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            environment:
                <<: *common-variables
            devices:
                - /dev/snd
            profiles: ["tts_sub"]
            command: >
                ros2 launch hri_tts tts.launch.py launch_pub:=False launch_sub:=True card_number:=1

Note that the docker enables GPU support by reserving one NVIDIA GPU device and giving it to the container.
The ``volumes`` and ``devices`` sections of the  tts file will be explained in the following sections.

Also note that the file has two services: ``tts_publisher`` and ``tts_subscriber``.

- The ``tts_publisher`` service runs an action server that waits for text input to be converted to speech.
  It acts similarly to the STT server, but in the opposite direction.
  It publishes the transcribed speech as an Audio message of type ``hri_msgs/msg/Audio`` on the ``/hri_tts`` topic.
- On the other hand, the ``tts_subscriber`` service runs a node that subscribes to the ``/hri_tts`` topic and plays the received audio through the specified speaker.
  This latest node is not required to perform text-to-speech conversion, but it is included in the package to provide a complete end-to-end example of the TTS functionality.

Downloading the models
^^^^^^^^^^^^^^^^^^^^^^

The TTS node will automatically download the model if they are not found in the system.
However, this step can take some time and it requires internet access.
To avoid this delay when running the node for the first time, it is possible to download the models beforehand.

The ``hri_tts`` package in Vulcanexus comes along a python script that automatically downloads the required models to run the node.
When running the script natively, the models are downloaded to the user's home hidden local directory (``/home/user/.local/share``).
The models will be loaded from that directory if they are already downloaded when running the node, accelerating the startup time.
To invoke the script, run the following command:

.. code-block:: bash

    ros2 run hri_tts download_tts_models

When using Docker, there exists two options to persist the models:

1. Run Vulcanexus HRI docker image and execute the download script inside the container.
This will download the models inside the container. Then, the docker image needs to be committed to a new image to persist the models.

.. code-block:: bash

    # Run the container
    docker run -it --rm ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri bash
    # Inside the container, run the download script and commit the container to a new image
    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash
    ros2 run hri_tts download_tts_models
    # In another terminal, find the container ID of the running container with `docker ps`
    # and replace <container_id> in the following command
    docker commit <container_id> ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri-tts

2. Alternatively, the user's home hidden local directory can be mounted in the container, allowing the models to be downloaded directly in the native host after running the download script from within the container.
This approach requires to always mount the model directory when running the container, as shown in the ``tts-compose.yaml`` file under the ``volumes`` section.
Otherwise, the models will be downloaded inside the container and will be lost when the container is removed.
To download the models locally, run the following command:

.. code-block:: bash

    docker run -it --rm -v /home/user/.local/share/tts:/root/.local/share/tts \
        --entrypoint bash ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri \
        -lc "source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && ros2 run hri_tts download_tts_models"

Selecting the speaker
^^^^^^^^^^^^^^^^^^^^^

First of all, note that in the ``tts-compose.yaml`` file, the ``/dev/snd`` device is mounted inside the container that will be running the ``tts_subscriber`` service to allow access to the audio output devices.

.. important::

    Docker does not recognize new audio devices connected to the system after the container is created, so it is necessary to connect the audio device before running the container.

The TTS requires to specify the audio output device to be used.
To do so, the card number of the device must be provided as a parameter when launching the node.
This parameter is mandatory.
If not provided, the node configuration will fail and abort the execution.
Additionally, the ``tts_subscriber`` node has an optional parameter ``device_index`` to specify the device to used in case the card has multiple devices.
If not provided, the node will use device 0 by default.
To find the card number of the audio output device, it is possible to use the ``list_speakers`` script included in the ``hri_tts`` package.

To list the available audio output devices, run the following command:

.. code-block:: bash

    docker run -it --rm --device /dev/snd --entrypoint bash ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri \
      -lc "source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && ros2 run hri_tts list_speakers"

This command will print an indexed list of all available audio output devices in the system.
The card number is indicated in the output as ``card=<number>``, and the device number as ``dev=<number>``.
This can also be found at the end of each line as ``plughw:<card>,<dev>``.

For example, imagine an output like the following:

.. code-block:: console

    [card=0,dev=0] Name: sof-hda-dsp - Details: HDA Analog (*) | hw:0,0 | plughw:0,0
    [card=0,dev=1] Name: sof-hda-dsp - Details: HDA Digital (*) | hw:0,1 | plughw:0,1
    [card=0,dev=3] Name: sof-hda-dsp - Details: HDMI1 (*) | hw:0,3 | plughw:0,3
    [card=0,dev=4] Name: sof-hda-dsp - Details: HDMI2 (*) | hw:0,4 | plughw:0,4
    [card=0,dev=5] Name: sof-hda-dsp - Details: HDMI3 (*) | hw:0,5 | plughw:0,5
    [card=1,dev=0] Name: Logi USB Headset - Details: USB Audio | hw:1,0 | plughw:1,0

In this case, if the desired audio output device is the ``Logi USB Headset``, the parameter ``card_number`` must be set to ``1`` and the parameter ``device_index`` does not need to be specified as it defaults to 0 (``plughw:1,0``).
However, if the desired device was ``HDMI2``, then the ``card_number`` parameter should be set to ``0`` and the ``device_index`` parameter to ``4`` (``plughw:0,4``).

At this point, modify the ``tts-compose.yaml`` file to set the card number parameter with the card number of the audio input device to be used, replacing the value in the following line.
If needed, add also the device index parameter.

.. code-block:: yaml

    command: >
        ros2 launch hri_tts tts.launch.py launch_pub:=False launch_sub:=True card_number:=<value> device_index:=<value>

.. _tutorials_hri_tts_run:

Run the publisher node
----------------------

To run the TTS node main node, which is responsible for text-to-speech synthesis, execute the following command in the same directory where the ``tts-compose.yaml`` file is located:

.. code-block:: bash

    docker compose -f tts-compose.yaml --profile tts_pub up -d

This command will start the ``hri_tts`` node in a new container, enabling the action server to receive transcription requests.
To check that the node is running correctly, attach a new terminal to the running container and run:

.. code-block:: bash

    ros2 action list

The output should include the ``/hri_tts`` action server.

To test the node, the ROS 2 command line tool can be used to send a request to the action server.
To do so, run:

.. code-block:: bash

    ros2 action send_goal /hri_tts hri_msgs/action/Tts "speech: 'Hello World from Vulcanexus'"

This command will prompt the node to start creating speech from the received text.
Once the speech synthesis is completed, the result will be received in the terminal as ``SUCCEEDED``.
It is important to note that the speech is not played automatically. It is not even returned to the client as the action response, but published as a separated ``hri_msgs/msg/Audio`` message on the ``/hri_tts/audio`` topic.

The reason behind this design choice is that the TTS node is only responsible for converting text to speech, while the playback of the audio is handled by a separate node.
Each audio duration depends on the length of the input text, but it does not match the time taken to generate the speech, which is usually much shorter.
Hence, a successful response from the action server only indicates that the speech synthesis has been completed and the audio message was sent, not that the audio has been played.

This node has a parameter called ``wait_for_finished`` which controls this behavior, and determines whether the action server should wait for the audio playback to finish before sending the response back to the client.
If this parameter is set to ``True``, the action server will not respond with ``SUCCEEDED`` until the audio has finished playing, which is indicated by the topic ``/hri_tts/finished``.
Once the audio playback is completed, a message containing a boolean with value ``True`` will be published on this topic, and the action server will then respond with ``SUCCEEDED``.

If no message is received on this topic within a timeout period, the action server will respond with ``ABORTED``.
The timeout period is calculated based on the duration of the audio generated and 5 additional seconds.
To check this behavior, the ``wait_for_finished`` parameter can be set to ``True`` in the ``tts-compose.yaml`` file.
Then recreate the container and repeat the test command, noting that the response will be delayed and it will finally return ``ABORTED``.

.. code-block:: bash

    docker compose -f tts-compose.yaml --profile tts_pub up -d --force-recreate

Now, send again the goal through the ROS 2 CLI and check that the response is ``ABORTED`` after a few seconds.

Run the subscriber node
-----------------------

To run the TTS subscriber node, which is responsible for playing the received audio through the specified speaker, execute the following command in a new terminal in the same directory where the ``tts-compose.yaml`` file is located:

.. code-block:: bash

    docker compose -f tts-compose.yaml --profile tts_sub up -d

This command will start the ``hri_tts`` subscriber node in a new container, enabling it to listen for audio messages on the ``/hri_tts/audio`` topic and play them through the specified audio output device.
It will also publish ``True`` on the ``/hri_tts/finished`` topic once the audio playback is complete, allowing the running action server to respond with ``SUCCEEDED`` if the ``wait_for_finished`` parameter is set to ``True``.

To test it, make sure the TTS publisher of previous section is still active and run again the action client command:

.. code-block:: bash

    ros2 action send_goal /hri_tts hri_msgs/action/Tts "speech: 'Hello World from Vulcanexus'"

This time, the audio should be played through the selected speaker, and the action server should respond with ``SUCCEEDED`` once the playback is complete.
You can try sending different text inputs to the action server to hear the corresponding speech output.

To stop all the running containers, use the following command:

.. code-block:: bash

    docker compose -f tts-compose.yaml --profile tts_pub --profile tts_sub down

.. _tutorials_hri_tts_presets:

Using presets
-------------

As it has been explained, the TTS publisher node publishes the generated speech as a ``hri_msgs/msg/Audio`` message on the ``/hri_tts/audio`` topic.
This message includes several fields, such as ``data`` to store the audio as *raw bytes* or other metadata like the *sample rate* or the *number of channels*.
One of these fields is called ``preset``, which is an *uint_8* that can be used to indicate the TTS subscriber to reproduce a specific preset, instead of the audio data included in the message (which is ignored in this case).

This feature is useful when some audios need to be played repeatedly, as it avoids the need to generate the same audio multiple times, saving both time and computational resources.
For example, if you want to program your robot to notify through speech when it has finished a task, you can create a preset for that specific notification and use it whenever needed, without having to generate and send the same audio every time.

The design of this feature allows to define custom presets totally compatible with the TTS main workflow, where no modifications are needed to the TTS publisher main class.
The TTS publisher will send the preset ID through the ``preset`` field of the Audio message if any of the following texts is received in the input text:

.. code-block:: python

    preset_phrases = {
        "_example": 100,
        "_hello": 101,
        "_bye": 102,
        "_finish": 103,
        "_stt_error": 104,
        "_music": 105,
    }

To check this behavior, replace the speech of previouses commands with any of the keys of the ``preset_phrases`` dictionary, like the following command:

.. code-block:: bash

    ros2 action send_goal /hri_tts hri_msgs/action/Tts "speech: '_example'"

You should hear the audio associated with the preset ID 100 being played through the selected speaker.

If needed, the ``preset_phrases`` list can be extended by adding new phrases and assigning them a unique preset ID (between 1 and 255).
However, it is needed to clone the ``hri_tts`` package, modify the source code by expanding the ``preset_phrases`` dictionary and re-compile the package to do so.
Additionally, the preset audio file needs to be added to the appropriate directory within the package, which is called ``audio_files``.

To generate new preset audio files, the TTS node can be used as an audio file generator.
This mode of operation follows the same patther as the TTS publisher, but it saves the generated audio in a ``.wav`` file instead of publishing it as an Audio message.

Run the generator node
----------------------

To run the TTS generator node, create a new file ``tts-gen-compose.yaml`` with the following content:

.. code-block:: yaml

    x-common-config:
       &common-config
        network_mode: host
        ipc: host

    x-common-variables:
        &common-variables
        ROS_DOMAIN_ID: 42

    services:
        tts_generator:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            deploy:
                resources:
                    reservations:
                        devices:
                        - driver: nvidia
                          count: 1
                          capabilities: [gpu]
            environment:
                <<: *common-variables
            volumes:
                - /home/user/.local/share/tts:/root/.local/share/tts    # Coqui TTS models
            command: >
                ros2 run hri_tts tts_gen --ros-args -p file_path:=/root/test_file.wav

Then, make sure to stop the container running the TTS publisher if it is still active, as both nodes implement the same action server and they cannot run simultaneously.
Finally, run the following command in the same directory where the file is located:

.. code-block:: bash

    docker compose -f tts-gen-compose.yaml up -d

Now, send a goal to the action server to generate a new audio file as previously explained:

.. code-block:: bash

    ros2 action send_goal /hri_tts hri_msgs/action/Tts "speech: 'This is a testing audio that will be saved in a wav file'"

You can check that the file ``test_file.wav`` has been created in the specified path.

.. warning::

    The TTS generator node will overwrite the output file if it already exists.
    It also requires an existing path to save the file, so ensure that the specified directory exists before running the node.

.. note::

    To attach a new terminal to one of the running containers, open a new terminal and run:
    ``docker exec -it "$(docker ps --quiet | head -n 1)" bash``.
    If there are other docker containers running in your system, ``"$(docker ps --quiet | head -n 1)"`` should be replaced with the container ID of the Vulcanexus container to avoid attaching to the wrong container.
