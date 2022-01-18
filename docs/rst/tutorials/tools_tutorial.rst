.. _vulcanexus_tools_tutorial:

ROS 2 network statistics using Vulcanexus
=========================================

.. contents:: Table of Contents
    :depth: 2
    :local:

Background
----------

Vulcanexus integrates *eProsima Fast DDS Monitor* which is a useful tool for monitoring and studying a ROS 2 network as ROS 2 relies on DDS communication protocol to communicate different nodes.
The automatic discovery of entities in a local network allows to easily see the different Participants that are running, as its Endpoints, the Topics that each one is using, and even the network interfaces they are employing to communicate with each other.
Additionally, one could receive statistical data from every endpoint in the network.
This data is very useful to analyze the performance and seek any possible communication problem in the network.

This tutorial provides step-by-step instructions to use Vulcanexus to monitor a ROS 2 talker/listener demo.

Prerequisites
-------------

Ensure that the Vulcanexus installation includes the tools (either ``vulcanexus-galactic-desktop``, ``vulcanexus-galactic-tools``, or ``vulcanexus-galactic-base``).
Also, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/galactic/setup.bash

Run tutorial
------------

Launch Fast DDS Monitor
^^^^^^^^^^^^^^^^^^^^^^^

Initiate Fast DDS Monitor running the following command:

.. code-block:: bash

    fastdds_monitor

Once Fast DDS Monitor is launched, start a monitor in domain :code:`0` (default domain).

.. figure:: /rst/figures/tools_tutorial/screenshots/init_domain.png
    :align: center

Execute ROS 2 demo nodes with statistics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to activate the publication of statistical data, *eProsima Fast DDS* requires an environment variable specifying those kinds of statistical data to be reported.
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
^^^^^^^^^^^^^^^^^^

Now one should see in the DDS Panel two new Participants.

.. figure:: /rst/figures/tools_tutorial/screenshots/participants.png
    :align: center

Alias
"""""

Participants in ROS 2 are named :code:`/` by default.
In order to differentiate them one could change the alias of the Participant.
The :code:`ros2-galactic-talker` would be the one with one writer, and the :code:`ros2-galactic-listener` the one with a reader.

.. figure:: /rst/figures/tools_tutorial/screenshots/alias.png
    :align: center

Physical data
"""""""""""""

In order to see the information of the host and the physical context where every node is running, go to the Explorer Panel and activate the Physical Panel.
There, the host, user and process of each node are displayed.

.. figure:: /rst/figures/tools_tutorial/screenshots/physical.png
    :align: center

Statistical data
""""""""""""""""

To show statistical data about the communication between the :code:`ros2-galactic-talker` and the :code:`ros2-galactic-listener`, follow the steps to `create dynamic series chart <https://fast-dds-monitor.readthedocs.io/en/latest/rst/getting_started/tutorial.html#tutorial-create-dynamic-series>`_ and plot this statistical data in a real time chart.

.. figure:: /rst/figures/tools_tutorial/screenshots/statistics.png
    :align: center

Introspect metatraffic topics
"""""""""""""""""""""""""""""

Fast DDS Monitor filters by default the topics used for sharing metatraffic and the endpoints related to them so the user can inspect their network easily.
These topics are the ones that ROS 2 uses for discovery and configuration purposes, such as :code:`ros_discovery_info`, as well as those used by Fast DDS to report statistical data.

In order to see these topics in the monitor, click *View->Show Metatraffic* menu button.
Now, these topics are shown in the logical panel, and also the Readers and Writers associated to them under their respective Participants.

.. figure:: /rst/figures/tools_tutorial/screenshots/metatraffic.png
    :align: center
