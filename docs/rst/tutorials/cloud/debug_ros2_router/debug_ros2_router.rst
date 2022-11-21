.. include:: ../../../exports/alias.include

.. _tutorials_debug_ros2_router:

Debugging ROS 2 Router
======================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see *ROS 2 Router* documentation :ref:`here <vulcanexus_router>`).
That is, ROS 2 nodes such as publishers and subscriptions, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic areas on their own dedicated local networks as if they were all on the same network through the use of *ROS 2 Router*.

This tutorial explains how to debug the *ROS 2 Router* to detect errors in the network configuration that may cause the discovery of the ROS 2 nodes and the deployed *ROS 2 Routers* to not be discovered, preventing the correct behaviour of automatic data routing.

To accomplish this task, this tutorial introduces a simple use case that configures the DDS Router to obtain as much information as possible from its execution.
Thus, the user will be able to apply the configuration that best suits his needs when debugging the DDS Router and understanding its behavior.

Moreover, depending on the user's knowledge, it will be possible to configure the amount of information that the *ROS 2 Router* will report, such as precise and accurate information about the discovery of ROS 2 nodes, the topics automatically found in the network, or the data being relayed.

As already mentioned, the approach of this tutorial is straightforward and is illustrated in the following figure:

.. figure:: /rst/figures/tutorials/cloud/change_domain_echo.png
   :align: center

This tutorial will use the ``demo_nodes_cpp`` package, available in the Vulcanexus Desktop distribution.
First, a ROS 2 ``talker`` is launched and then a ``listener`` node is started in a different ROS 2 Domain.
This will prevent the two from communicating.
At this point, the *ROS 2 Router* will be deployed as a bridge between the two Domains and will enable the ``talker``-``listener`` communication.
Please take into account that a specific configuration will be applied to the *ROS 2 Router* in order to see its status and operation at runtime.

.. warning::

    It is important to mention that the performance of the *ROS 2 Router* will be affected due to the generation of extra code to present this information so it is not recommended to compile the *ROS 2 Router* following this tutorial in critical or production environments.


Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Deploy ROS 2 nodes
------------------

First let's run the ROS 2 ``talker`` and ``listener`` nodes.

Environment setup
^^^^^^^^^^^^^^^^^

Setup the Vulcanexus environment to have the ``demo_nodes_cpp`` package available.
For this, there are two possible options:

#.  Running the Vulcanexus Docker image.

    Run the Vulcanexus Docker image with:

    .. code-block:: bash

        docker run -it ubuntu-vulcanexus:humble-desktop

    Then, within the container, source the Vulcanexus installation with:

    .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash

#.  Setting up the development environment on the local host.
    For this second option, it is necessary to have installed the ``vucanexus-humble-desktop`` package, since this is the one that includes all the simulation tools, demos and tutorials.

    Source the following file to setup the Vulcanexus environment:

    .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash

Running ROS 2 nodes
^^^^^^^^^^^^^^^^^^^

Once the environment has been setup using one of the above options, run the ROS 2 ``talker`` node in one terminal.

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

Then, on another terminal, run the ROS 2 ``listener`` node in ROS 2 Domain ``1``.

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener


At this point, the ``listener`` should not receive any data.
If not, please go again through the previous steps.


Deploy ROS 2 Router
-------------------

Then, create the *ROS 2 Router* configuration file as the one shown below.

.. note::

    If deploying Vulcanexus from the Docker image, note that you will need to have a configuration file (config.yaml) for the DDS Router Edge accessible from your Docker container.

    This can be achieved by mounting a shared volume when launching the container, by copying the file from the local host to the container in case it is already running, or by editing a file from the Docker container itself.

.. literalinclude:: /resources/tutorials/cloud/change_domain/change_domain_with_echo.yaml
    :language: yaml

Main aspect of this configuration is the Echo Participant.
This type of participant, specific for debugging purposes, is explained in detail below.

Echo Participant
^^^^^^^^^^^^^^^^

The `Echo Participant` is a participant/interface of the *ROS 2 Router* that prints in ``stdout`` all discovery information and/or user data that is received by the *ROS 2 Router*. Therefore, this participant does not perform any discovery or data reception functionality.

In the case of discovery traces, messages such as the following will be displayed:

.. code-block:: bash

    New endpoint discovered: Endpoint{<endpoint_guid>;<endpoint_kind>;<topic_info>}.

For data reception messages, the traces show the following information:

.. code-block:: bash

    Received data in Participant: <participant_id> in topic: <topic>.

These logs contain the `Participant Name` of the participant that has originally received the message, and the `Topic Information` where this message has been received. The `Topic Information` shown the `Topic Name`, `Topic Data Type Name` and the `QoS`.
Additionally, extra information such as the data `Payload` (in hexadecimal format) and source `Endpoint` `Guid` is displayed in ``verbose`` mode:

.. code-block:: bash

    In Endpoint: <endpoint_guid> from Participant: <participant_id> in topic: <topic> payload received: <payload> with specific qos: <specific_qos>.


Running ROS 2 Router
^^^^^^^^^^^^^^^^^^^^

Now, run the *ROS 2 Router* with the configuration file created as an argument.

.. code-block:: bash

    ddsrouter -c <path/to/file>/echo.yaml

At this point you should see some information like the one shown below.
This indicates that the *ROS 2 Router* has discovered the deployed ROS 2 nodes, their topics, the QoS of each topic and is relaying the information coming from the ``talker`` to the ``listener`` (from Domain ``0`` to Domain ``1``).

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.
    New endpoint discovered: Endpoint{01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.1.3;writer;DdsTopic{ros_discovery_info;rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_;Fuzzy{Level(20) TopicQoS{TRANSIENT_LOCAL;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_0}}.
    New endpoint discovered: Endpoint{01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.3.3;writer;DdsTopic{rt/rosout;rcl_interfaces::msg::dds_::Log_;Fuzzy{Level(20) TopicQoS{TRANSIENT_LOCAL;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_0}}.
    New endpoint discovered: Endpoint{01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.5.3;writer;DdsTopic{rr/talker/get_parametersReply;rcl_interfaces::srv::dds_::GetParameters_Response_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_0}}.
    New endpoint discovered: Endpoint{01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.7.3;writer;DdsTopic{rr/talker/get_parameter_typesReply;rcl_interfaces::srv::dds_::GetParameterTypes_Response_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_0}}.
    New endpoint discovered: Endpoint{01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.12.3;writer;DdsTopic{rt/chatter;std_msgs::msg::dds_::String_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_0}}.
    ...
    In Endpoint: 01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.12.3 from Participant: ParticipantId{SimpleParticipant_Domain_0} in topic: DdsTopic{rt/chatter;std_msgs::msg::dds_::String_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}} payload received: Payload{00 01 00 00 10 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 34 31 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    In Endpoint: 01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.12.3 from Participant: ParticipantId{SimpleParticipant_Domain_0} in topic: DdsTopic{rt/chatter;std_msgs::msg::dds_::String_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}} payload received: Payload{00 01 00 00 10 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 34 32 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    ...
    In Endpoint: 01.0f.eb.7d.3c.00.ad.17.01.00.00.00|0.0.12.3 from Participant: ParticipantId{SimpleParticipant_Domain_0} in topic: DdsTopic{rt/chatter;std_msgs::msg::dds_::String_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}} payload received: Payload{00 01 00 00 10 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 34 36 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.

All this information may be overwhelming and not easy to read at first.
Therefore, *Echo Participant* settings can be changed to match what you want to see, as described in the previous section.

For example, if you just want to show which nodes the *ROS 2 Router* is discovering, it is possible to disable the ``data`` and ``verbose`` levels of the *Echo Participant* configuration.


Advance debugging
-----------------

The *ROS 2 Router* can be built with the built-in debug traces so that the internal behavior of the software can be fully monitored.
To do so, the user/developer should recompile the *ROS 2 Router* package that comes with Vulcanexus by adding the required compilation options that enable the debug operation mode.

This provides some additional options to filter the debug information shown by the *ROS 2 Router*. We will explain this in detail below.

.. note::

    Do not mistake the debug mode of *ROS 2 Router* with the ``Debug`` compilation of C++ code, since *ROS 2 Router* is still compiled in ``Release`` mode but with debug traces.

Building ROS 2 Router from sources
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Building the *ROS 2 Router* from sources in Vulcanexus is straightforward as Vulcanexus ships with all the necessary tools for this task, so there is no prerequisite other than the one already shown in this tutorial.

The steps to follow are described below:

1.  Load the Vulcanexus environment.

    -   If you are running Vulcanexus from the distributed Docker image, simply run the Vulcanexus Docker container.

        .. code-block:: bash

            docker run -it ubuntu-vulcanexus:humble-desktop

    -   If you have installed Vulcanexus on your local machine, load the environment with the following command.

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash

1.  Create the development workspace, download the *ROS 2 Router* from GitHub, and build it by executing the following commands:

    .. code-block:: bash

        mkdir -p ddsrouter_ws/src
        cd ddsrouter_ws
        git clone https://github.com/eProsima/DDS-Router src/ddsrouter
        colcon build --cmake-args -DLOG_INFO=ON

Running ROS 2 Router
^^^^^^^^^^^^^^^^^^^^

Update the environment setup to use the built *ROS 2 Router* instead of the one delivered in Vulcanexus.

.. code-block:: bash

    source ddsrouter_ws/install/setup.bash

Monitor ROS 2 Router internal operation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*ROS 2 Router* offers several input arguments to configure the information displayed when it is built with the internal debug traces.
These are:

-   ``--log-verbosity <info|warning|error>``: set the verbosity level so only log messages with equal or higher importance level are shown.
-   ``--log-filer <regex>``: set a regex string as filter.
    Only log messages with a category that matches this regex will be printed (``ERROR`` messages will be always shown unless ``log-verbosity`` argument is set to ``ERROR``).
-   ``--debug``: set ``log-verbosity`` to ``info`` and ``log-filter`` to ``DDSROUTER``.

Thus, *ROS 2 Router* can be run as follows:

.. code-block:: bash

    ddsrouter -c <path/to/file>/echo.yaml --log-verbosity warning --log-filer DDSROUTER_DISCOVERY_DATABASE

The *ROS 2 Router* should prompt some information like the one show below.

.. code-block:: bash

    2022-11-21 12:52:35.912 [DDSROUTER_DISCOVERY_DATABASE Info] Inserting a new discovered Endpoint Endpoint{01.0f.3a.59.2c.00.41.5f.02.00.00.00|0.0.6.4;reader;DdsTopic{rq/slam_toolbox/get_parameter_typesRequest;rcl_interfaces::srv::dds_::GetParameterTypes_Request_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_1}}. -> Function add_endpoint_

    New endpoint discovered: Endpoint{01.0f.3a.59.2c.00.41.5f.02.00.00.00|0.0.6.4;reader;DdsTopic{rq/slam_toolbox/get_parameter_typesRequest;rcl_interfaces::srv::dds_::GetParameterTypes_Request_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_1}}.

    2022-11-21 12:52:35.921 [DDSROUTER_DISCOVERY_DATABASE Info] Inserting a new discovered Endpoint Endpoint{01.0f.3a.59.2c.00.41.5f.02.00.00.00|0.0.8.4;reader;DdsTopic{rq/slam_toolbox/set_parametersRequest;rcl_interfaces::srv::dds_::SetParameters_Request_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};
    SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_1}}. -> Function add_endpoint_

    New endpoint discovered: Endpoint{01.0f.3a.59.2c.00.41.5f.02.00.00.00|0.0.8.4;reader;DdsTopic{rq/slam_toolbox/set_parametersRequest;rcl_interfaces::srv::dds_::SetParameters_Request_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_1}}.

    2022-11-21 12:52:35.933 [DDSROUTER_DISCOVERY_DATABASE Info] Inserting a new discovered Endpoint Endpoint{01.0f.3a.59.2c.00.41.5f.02.00.00.00|0.0.a.4;reader;DdsTopic{rq/slam_toolbox/set_parameters_atomicallyRequest;rcl_interfaces::srv::dds_::SetParametersAtomically_Request_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_1}}. -> Function add_endpoint_

    New endpoint discovered: Endpoint{01.0f.3a.59.2c.00.41.5f.02.00.00.00|0.0.a.4;reader;DdsTopic{rq/slam_toolbox/set_parameters_atomicallyRequest;rcl_interfaces::srv::dds_::SetParametersAtomically_Request_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant_Domain_1}}.


