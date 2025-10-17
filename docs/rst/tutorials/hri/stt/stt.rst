.. _tutorials_hri_stt:

Vulcanexus HRI Speech-to-Text (STT) Tutorial
=============================================

Quickview
---------

This tutorial will explore the process of implementing speech-to-text capabilities within the Vulcanexus framework.
It will cover the necessary steps to set up the environment, integrate the speech-to-text model, and visualize the results.

A deeper explanation of the STT package can be found in the :ref:`vulcanexus_hri_stt` section.

Prerequisites
-------------

Before starting this tutorial, ensure you have the following prerequisites:

- A working installation of Vulcanexus with `HRI modules <https://docs.vulcanexus.org/en/latest/rst/introduction/metapackages.html#vulcanexus-hri>`_.
- Docker and Docker Compose installed on your system.
- GPU with CUDA support (not mandatory, but recommended).
- Familiarity with ROS 2 concepts and tools.
- A microphone or a compatible audio input device.

Initial Setup
-------------

The tutorial will make use of the Vulcanexus docker image in order to simplify the setup process.
However, it is also possible to run the tutorial on a native installation of Vulcanexus.

The STT package in Vulcanexus is designed to transcript audio in real-time from a microphone.
If no audio input device is available, the node will not be able to record any audio stream and will not publish any transcription.

Also, no AI model is included in the Vulcanexus installation by default due to their size, so it is necessary to download the models before running the tutorial.
This applies to both Vulcanexus apt package and the docker image.
The following sections describe how to select the microphone and download the models.

.. _tutorial_hri_stt_docker_setup:

Docker setup
^^^^^^^^^^^^

Create a ``stt-compose.yaml`` file with the following content, replacing ``<VULCANEXUS_DISTRO>`` with your Vulcanexus distribution:

.. code-block:: yaml

    x-common-config:
       &common-config
        network_mode: host
        ipc: host

    x-common-variables:
        &common-variables
        ROS_DOMAIN_ID: 42

    services:
        stt_server:
            image: ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri
            <<: *common-config
            deploy:
                resources:
                    reservations:
                        devices:
                        - driver: nvidia
                          count: 1
                          capabilities: [gpu]
            privileged: true
            environment:
                <<: *common-variables
                LD_LIBRARY_PATH: "/usr/local/lib/python3.12/dist-packages/nvidia/cublas/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cudnn/lib:${LD_LIBRARY_PATH}"
            devices:
                - /dev/snd
            volumes:
                - /home/user/.cache/torch:/root/.cache/torch                # Silero models
                - /home/user/.cache/huggingface:/root/.cache/huggingface    # Whisper models
            command: >
                ros2 launch hri_stt stt_server.launch.py microphone:="Your Microphone Name here"

Note that the docker enables GPU support by reserving one NVIDIA GPU device and setting the appropriate ``LD_LIBRARY_PATH`` environment variable.
The ``volumes`` and ``devices`` sections of the file will be explained in the following sections.

Downloading the models
^^^^^^^^^^^^^^^^^^^^^^

The STT node will automatically download the model if they are not found in the system.
However, this step cant take some time and it requires internet access.
To avoid this delay when running the node for the first time, it is possible to download the models beforehand.

The ``hri_stt`` package in Vulcanexus comes along a python script that automatically downloads required models to run the node.
When running the script natively, the models are downloaded to the user's home hidden cache directory (``/home/user/.cache/torch`` and ``/home/user/.cache/huggingface``).
The models will be loaded from that directory if they are already downloaded when running the node, accelerating the startup time.
To invoke the script, run the following command:

.. code-block:: bash

    ros2 run hri_stt download_stt_models

When using Docker, there exists two options to persist the models:

1. Run Vulcanexus HRI docker image and execute the download script inside the container.
This will download the models inside the container. Then, the docker image needs to be committed to a new image to persist the models.

.. code-block:: bash

    # Run the container
    docker run -it --rm ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri bash
    # Inside the container, run the download script and commit the container to a new image
    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash
    ros2 run hri_stt download_stt_models
    # In another terminal, find the container ID of the running container with `docker ps`
    # and replace <container_id> in the following command
    docker commit <container_id> ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri-stt

2. Alternatively, the user's home hidden cache can be mounted in the container, allowing the models to be downloaded directly in the native host after running the download script from within the container.
This approach requires to always mount the cache directories when running the container, as shown in the ``stt-compose.yaml`` file under the ``volumes`` section.
Otherwise, the models will be downloaded inside the container and will be lost when the container is removed.
To download the models locally, run the following command:

.. code-block:: bash

    docker run -it --rm -v /home/user/.cache/torch:/root/.cache/torch \
        -v /home/user/.cache/huggingface:/root/.cache/huggingface \
        --entrypoint bash ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri \
        -lc "source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && ros2 run hri_stt download_stt_models"

Selecting the microphone
^^^^^^^^^^^^^^^^^^^^^^^^

First of all, note that in the ``stt-compose.yaml`` file, the ``/dev/snd`` device is mounted inside the container to allow access to the audio input devices.

.. important::

    Docker does not recognize new audio devices connected to the system after the container is created, so it is necessary to connect the audio device before running the container.

The STT node auto-configures itself by selecting a valid sampling rate and extracting the channels used by the microphone when a valid device is provided.
However, it is required to specify the audio input device to be used.
To do so, the name of the device must be provided as a parameter when launching the node.
This parameter is mandatory.
If not provided, the node configuration will fail and abort the execution.
To find the name of the audio input device, it is possible to use the ``list_microphones`` executable included in the ``hri_stt`` package.

To list the available audio input devices, run the following command:

.. code-block:: bash

    docker run -it --rm --device /dev/snd --entrypoint bash ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri \
        -lc "source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && ros2 run hri_stt list_microphones"

This command will print an indexed list of all available audio input devices in the system.
The name of the device is the string printed between brackets at the beginning of each line.
The parameter name must contain the name printed here, without including the brackets.
Note that the whole name is not required to select the device, but just a substring that uniquely identifies the device.
For example, if the output of the command is:

.. code-block:: console

    - Index 0: [sof-hda-dsp: - (hw:0,0)] - Channels: 2, - Rate: 48000.0 Hz
    - Index 1: [sof-hda-dsp: - (hw:0,3)] - Channels: 0, - Rate: 48000.0 Hz
    - Index 2: [sof-hda-dsp: - (hw:0,4)] - Channels: 0, - Rate: 48000.0 Hz
    - Index 3: [sof-hda-dsp: - (hw:0,5)] - Channels: 0, - Rate: 48000.0 Hz
    - Index 4: [sof-hda-dsp: - (hw:0,6)] - Channels: 2, - Rate: 48000.0 Hz
    - Index 5: [sof-hda-dsp: - (hw:0,7)] - Channels: 2, - Rate: 16000.0 Hz
    - Index 6: [USB Composite Device: Audio (hw:1,0)] - Channels: 1, - Rate: 48000.0 Hz
    - Index 7: [sysdefault] - Channels: 128, - Rate: 48000.0 Hz

Then, passing ``USB Composite Device`` as the device name will univocally select the microphone.
Also note that the string is case-sensitive.

At this point, modify the ``stt-compose.yaml`` file to set the microphone parameter with the name of the audio input device to be used, replacing the value in the following line:

.. code-block:: yaml

    command: >
        ros2 launch hri_stt stt_server.launch.py microphone:="Your Microphone Name here"

.. _tutorials_hri_stt_run:

Run the node
------------

To run the STT node, execute the following command in the same directory where the ``stt-compose.yaml`` file is located:

.. code-block:: bash

    docker compose -f stt-compose.yaml up -d

This command will start the ``hri_stt`` node in a new container, enabling the action server to receive transcription requests.
To check that the node is running correctly, attach a new terminal to the running container and run:

.. code-block:: bash

    ros2 action list

The output should include the ``/hri_stt`` action server.

To test the node, the ROS 2 command line tool can be used to send a request to the action server.
To do so, run:

.. code-block:: bash

    ros2 action send_goal /hri_stt hri_msgs/action/Stt "start: true"

This command will prompt the node to start transcribing audio from the specified microphone.
The node will automatically stop recording after a few seconds of silence.
Once the transcription is complete, the result will be received in the terminal.

.. note::

    To attach a new terminal to one of the running containers, open a new terminal and run:
    ``docker exec -it "$(docker ps --quiet | head -n 1)" bash``.
    If there are other docker containers running in your system, ``"$(docker ps --quiet | head -n 1)"`` should be replaced with the container ID of the Vulcanexus container to avoid attaching to the wrong container.
