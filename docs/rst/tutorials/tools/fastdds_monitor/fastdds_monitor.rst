.. _tutorials_tools_fastdds_monitor:

ROS 2 network statistics inspection with ROS 2 Monitor
======================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Vulcanexus integrates the `ROS 2 Monitor <https://fast-dds-monitor.readthedocs.io/en/latest/>`_, a powerful tool for monitoring and studying ROS 2 networks. Since ROS 2 relies on the `DDS specification <https://www.omg.org/spec/DDS/1.4/About-DDS/>`_ for communication between nodes, this tool simplifies the process of observing network activity..
With automatic entity discovery within a local network, users can easily identify active Participants, their Endpoints, the Topics they utilize, and even the network interfaces used for communication.
Moreover, the `ROS 2 Statistics Module <https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/statistics.html>`_ enables the collection of statistical data from every endpoint in the network. This data is very useful for analyzing the DDS network performance and diagnosing possible communication issues.

This tutorial prov ides step-by-step instructions to use Vulcanexus for monitoring a ROS 2 talker/listener demo.

Prerequisites
-------------

Ensure that the Vulcanexus installation includes Vulcanexus Tools (either ``vulcanexus-jazzy-desktop``, ``vulcanexus-jazzy-tools``, or ``vulcanexus-jazzy-base``).
Also, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash

Launch ROS 2 Monitor
-----------------------

Initiate *ROS 2 Monitor* running the following command:

.. code-block:: bash

    fastdds_monitor

Once *ROS 2 Monitor* is launched, start a monitor in domain :code:`0` (default domain).

.. figure:: /rst/figures/tutorials/tools/monitor_screenshots/Init_domain.png
    :align: center

Execute ROS 2 demo nodes with statistics
----------------------------------------

In order to activate the publication of statistical data, `eProsima Fast DDS <https://fast-dds.docs.eprosima.com/en/latest/>`_ requires an environment variable specifying which kinds of statistical data are to be reported.
Consequently, before launching the ROS 2 nodes, remember to set ``FASTDDS_STATISTICS`` environment variable.
Run the following commands in different terminals (remember to source the Vulcanexus environment):

.. code-block:: bash

    export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;\
    SUBSCRIPTION_THROUGHPUT_TOPIC;RTPS_SENT_TOPIC;RTPS_LOST_TOPIC;\
    HEARTBEAT_COUNT_TOPIC;ACKNACK_COUNT_TOPIC;NACKFRAG_COUNT_TOPIC;\
    GAP_COUNT_TOPIC;DATA_COUNT_TOPIC;RESENT_DATAS_TOPIC;SAMPLE_DATAS_TOPIC;\
    PDP_PACKETS_TOPIC;EDP_PACKETS_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC"

    ros2 run demo_nodes_cpp listener

.. code-block:: bash

    export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;\
    SUBSCRIPTION_THROUGHPUT_TOPIC;RTPS_SENT_TOPIC;RTPS_LOST_TOPIC;\
    HEARTBEAT_COUNT_TOPIC;ACKNACK_COUNT_TOPIC;NACKFRAG_COUNT_TOPIC;\
    GAP_COUNT_TOPIC;DATA_COUNT_TOPIC;RESENT_DATAS_TOPIC;SAMPLE_DATAS_TOPIC;\
    PDP_PACKETS_TOPIC;EDP_PACKETS_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC"

    ros2 run demo_nodes_cpp talker

Monitoring network
------------------

Now, the two new Participants are visible in the *ROS 2 Monitor*'s DDS Panel.

.. figure:: /rst/figures/tutorials/tools/monitor_screenshots/Participants.png
    :align: center

Alias
^^^^^

Participants in ROS 2 are named :code:`/` by default.
In order to differentiate them, it is possible to change the Participant's aliases within the *ROS 2 Monitor*.
In this case, the :code:`talker` Participant would be the one with a :code:`chatter` writer, and the :code:`listener` Participant would be the one with a :code:`chatter` reader.

.. figure:: /rst/figures/tutorials/tools/monitor_screenshots/Alias.png
    :align: center

Physical data
^^^^^^^^^^^^^

In order to see the information of the Host and the physical context where every node is running, go to the Explorer Panel and activate the Physical Panel.
There, the Host, User and Process of each node are displayed.

.. figure:: /rst/figures/tutorials/tools/monitor_screenshots/Physical.png
    :align: center

Statistical data
^^^^^^^^^^^^^^^^

To show statistical data about the communication between the :code:`talker` and the :code:`listener`, follow the steps to `create dynamic series chart <https://fast-dds-monitor.readthedocs.io/en/latest/rst/getting_started/tutorial.html#tutorial-create-dynamic-series>`_.

.. figure:: /rst/figures/tutorials/tools/monitor_screenshots/Statistics.png
    :align: center

Introspect metatraffic topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*ROS 2 Monitor* filters by default the topics used for sharing metatraffic, as well as the endpoints related to them, so users can inspect their network easily.
These topics are the ones that ROS 2 uses for discovery and configuration purposes, such as :code:`ros_discovery_info`, as well as those used by Fast DDS to report statistical data.

In order to see these topics in the monitor, click *View->Show Metatraffic* menu button.
Now, these topics are shown in the logical panel. Furthermore, the Readers and Writers associated to them are now listed under their respective Participants.

.. figure:: /rst/figures/tutorials/tools/monitor_screenshots/Metatraffic.png
    :align: center
