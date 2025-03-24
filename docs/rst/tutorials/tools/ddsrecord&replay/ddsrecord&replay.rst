.. _tutorials_tools_ddsrecord&replay:

ROS 2 recording with DDS Record & Replay
========================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

.. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSRecord&Replay.png
    :align: center

*Vulcanexus* integrates the `DDS Record & Replay <https://dds-recorder.readthedocs.io/en/latest/>`_, a powerful tool that efficiently saves DDS data published into a DDS environment in a MCAP format database.
Thus, the exact playback of the recorded network events is possible as the data is linked to the timestamp at which the original data was published.
This tutorial provides step-by-step instructions to use *Vulcanexus* for monitoring a ROS 2 turtlesim demo.

Prerequisites
-------------

Ensure that the *Vulcanexus* installation includes *Vulcanexus Tools* (either ``vulcanexus-jazzy-desktop``, ``vulcanexus-jazzy-tools``, or ``vulcanexus-jazzy-base``).
Also, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash

Initial Setup
-------------

In this tutorial, we will use a Docker container to launch and control multiple ROS 2 components across four separate terminals.
Each terminal will serve a specific purpose, ensuring a structured and modular approach to the demonstration.

1. **Terminal 1: Launching the Turtlesim Simulation**:
    This first terminal will run the turtlesim graphical interface, where two turtles (A and B) will move within the simulation.

    .. code-block:: bash

        # Terminal 1
        ros2 run turtlesim turtlesim_node

    .. note::
        As the docker requires a graphical interface, running the docker container with the following flags is recommended:
        :code:`-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all --env QT_X11_NO_MITSHM=1`.

        It may also be necessary to export the following environment variable inside the container: :code:`export LIBGL_ALWAYS_SOFTWARE=1`.

2. **Terminal 2: Spawning and Controlling the Turtles**:
    In this second terminal, we will first remove the default turtle1 and spawn two new turtles, A and B.
    Then, we will use this terminal to manually control turtle A.

    .. code-block:: bash

        # Terminal 2
        ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle1'}"
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.54, y: 6.54, theta: 0.0, name: 'A'}"
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.54, y: 4.54, theta: 0.0, name: 'B'}"
        ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/A/cmd_vel

3. **Terminal 3: Synchronizing Turtle Movements (Mimic Mode)**:
    In the third terminal, we will enable B to mimic the movements of A thanks to the mimic node provided by the turtlesim package.

    .. code-block:: bash

        # Terminal 3
        ros2 run turtlesim mimic --ros-args --remap input/pose:=/A/pose --remap output/cmd_vel:=/B/cmd_vel

4. **Terminal 4: Recording and Replaying Movements**:
    Finally, in the fourth terminal, we will record the movements of turtles A and B and replay them using *DDS Record & Replay*.


Record Data
-----------

.. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSRecord.png
    :align: center

In this section, we will use the *DDSRecorder* to capture the movement commands sent to the turtles.
This will allows us to replay the exact motion sequences later:

.. code-block:: bash

    # Terminal 4
    ddsrecorder -c <path_to_recorder_config>.yaml

We will use different recording configurations to control which topics are recorded:

1. **Record all topics**:
    This configuration will record all topics in the specified ROS2 domain and will save them to a mcap file timestamped and followed by the specified filename.

    .. code-block::

        dds:
          domain: 0

        recorder:
          output:
            filename: "tutorial"
            path: "."

2. **Record commanded velocities**:
    To achieve this configuration, we will use the allowlist option to specify the topics we want to record, in this case any topic ending with "cmd_vel".

    .. code-block::

        dds:
          domain: 0

          allowlist:
            - name: "*/cmd_vel"

        recorder:
          output:
            filename: "tutorial"
            path: "."

3. **Record only turtle A's movements**:
    This configuration will record only the /A/cmd_vel topic. To achieve this we could use the allowlist as in the previous example (the recommended approach) or, just for demonstration purposes, we could use the blocklist to exclude all the topics in the ``B`` namespace using the wildcard character ``*``.

    .. code-block::

        dds:
          domain: 0

          allowlist:
          # - name: "rt/A/cmd_vel" # Recommended approach
            - name: "*/cmd_vel"

          blocklist:
            - name: "*/B/*"

        recorder:
          output:
            filename: "tutorial"

    .. note::
        The ROS2 topics and types suffer a mangling process when converted to DDS topics and types.
        This is why in the allowlist we would need to use the namespace ``rt`` to refer to the ROS2 topic ``/A/cmd_vel``.

.. important::
    To check all the available options for the recorder configuration, you can refer to the `DDS Recorder documentation <https://dds-recorder.readthedocs.io/en/latest/rst/recording/usage/configuration.html>`_.

Replay Data
-----------

.. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSReplay.png
    :align: center

Once we have recorded the data, we can replay it using the *DDSReplayer* tool:

.. code-block:: bash

    # Terminal 4
    ddsreplayer -c <path_to_recorder_config>.yaml -i <path_to_mcap_file>.mcap

We will use different replay configurations to control how the data is replayed:

1. **Replay all data**:
    This configuration will replay all the data recorded in the mcap file in the specified ROS2 domain.

    .. code-block::

        dds:
          domain: 0

2. **Replay only movement commands**:
    This configuration will replay only the movement commands recorded in the mcap file.

    .. code-block::

        dds:
          domain: 0

          allowlist:
            - name: "*/cmd_vel"

    .. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSRecord&Replay_all.gif
        :align: center

3. **Replay only turtle A's movements**:
    This configuration will replay only the movement commands of turtle A recorded in the mcap file.

    .. code-block::

        dds:
          domain: 0

          allowlist:
            - name: "rt/A/cmd_vel"

    .. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSRecord&Replay_A.gif
        :align: center

4. **Replay in a different ROS2 domain**:
    This configuration will replay the data in a different ROS2 domain, in our case we will use ``ROS_DOMAIN_ID=2``.

    .. code-block::

        dds:
          domain: 2

          allowlist:
            - name: "rt/A/cmd_vel"

    .. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSRecord&Replay_domain.gif
        :align: center

5. **Replay at a different rate**:
    This configuration will replay the data at a different rate, in our case we will use a rate of 2.0.

    .. code-block::

        dds:
          domain: 2

          allowlist:
            - name: "rt/A/cmd_vel"

        replayer:
          rate: 2.0

    .. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSRecord&Replay_rate.gif
        :align: center

6. **Replay from a specific time until a specific time**:
    This configuration will replay the data from a specific time until a specific time, in our case we will concatenate two different replays, the first one will end at a specific time and the second one will start at that same time, so the turtle will complete its whole route in two different replays.

    .. code-block::

        dds:
          domain: 2

          allowlist:
            - name: "rt/A/cmd_vel"

        replayer:
          begin-time:
            datetime: <start_time>
            # format: "%Y-%m-%d_%H-%M-%S"  # Optional
            # local: true                  # Optional
            # milliseconds: <milliseconds> # Optional

    .. code-block::

        dds:
          domain: 2

          allowlist:
            - name: "rt/A/cmd_vel"

        replayer:
          end-time:
            datetime: <end_time>
            # format: "%Y-%m-%d_%H-%M-%S"  # Optional
            # local: true                  # Optional
            # milliseconds: <milliseconds> # Optional

    .. figure:: /rst/figures/tutorials/tools/ddsrecord&replay/DDSRecord&Replay_time.gif
        :align: center


.. important::
    To check all the available options for the replayer configuration, you can refer to the `DDS Replayer documentation <https://dds-recorder.readthedocs.io/en/latest/rst/replaying/usage/configuration.html>`_.

