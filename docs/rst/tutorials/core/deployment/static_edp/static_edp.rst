.. _tutorials_deployment_static_edp:

Static Discovery
================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

`OMG DDS Interoperability specification <https://www.omg.org/spec/DDSI-RTPS/>`_ defines the basic discovery mechanism that allows distributed systems to automatically find and match Endpoints sharing the same Topic.
This mechanism is performed in two phases as explained in `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html#discovery-phases>`_:

* **Participant Discovery Phase (PDP)**: DDS DomainParticipants discover and acknowledge the existence of remote DomainParticipants.
  Usually done sending periodic announcements to well-known multicast addresses and ports defined in the specification.
* **Endpoint Discovery Phase (EDP)**: each DDS DomainParticipant send information about its own endpoints (DataWriters and DataReaders) through the communication channels established in the previous phase.

Standard discovery has its caveats, like the required multicast traffic being sent periodically, that can saturate the network bandwidth in constraint network architectures with a large number of DDS DomainParticipants.
Vulcanexus leverages latest *eProsima Fast DDS* release to improve ROS 2 middleware layer.
Being aware of the standard discovery mechanism caveats, *eProsima Fast DDS* provides alternative discovery mechanisms:

* *Fast DDS Discovery Server* reduces the discovery metatraffic using a centralized client-server discovery mechanism instead of a distributed one.
  More information can be found in :ref:`Using Fast DDS Discovery Server as discovery protocol tutorial <ros2-advanced-tutorial-fastdds-discovery-server>`.
  This schema reduces discovery metatraffic from both PDP and EDP phases because the information is sent exclusively to the DomainParticipants in a need-to-know basis.
* Setting an `initial peers list <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#simple-initial-peers>`_ and `disabling multicast <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/disabling_multicast.html#disabling-all-multicast-traffic>`_ also decreases the discovery traffic for the PDP phase, sending each DomainParticipant the announcement to the locators where the announcement is expected.
* *Fast DDS Static Discovery* removes completely any EDP discovery metatraffic by configuring the endpoints, topics and data types beforehand.
  This tutorial deals on how to use this discovery mechanism within Vulcanexus.
  More information about `Static EDP discovery <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/static.html#static-discovery-settings>`_ can be found in Fast DDS documentation.

.. TODO: Initial peers with no multicast tutorial

Prerequisites
-------------

This tutorial uses ROS 2 ``demo_nodes_cpp`` package which is provided with Vulcanexus Desktop distribution.
If another Vulcanexus distribution is used, please ensure that this package and its dependencies are installed.

`Wireshark <https://www.wireshark.org/>`_ is also used to capture and analyze the network traffic.

Please, remember to source the environment in every terminal used during this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

Overview
--------

This tutorial will use ROS 2 ``demo_nodes_cpp`` ``talker`` and ``listener`` applications to showcase the different configurations:


1. Talker-listener communication using :ref:`tutorials_deployment_static_edp_simple_discovery`.
   Shared memory transport is explicitly disabled to ensure EDP discovery traffic can be captured.
2. Talker-listener communication using :ref:`tutorials_deployment_static_edp_discovery`.
3. Talker-listener :ref:`full-fledged communication <tutorials_deployment_static_edp_fullfledged_discovery>`.

.. _tutorials_deployment_static_edp_simple_discovery:

Standard discovery
------------------

*eProsima Fast DDS* enables by default a `Shared Memory (SHM) transport <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html#shared-memory-transport>`_ and a `UDP transport <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/udp/udp.html#udp-transport>`_.
With this configuration, PDP discovery metatraffic will always be performed through the UDP transport.
However, if the SHM transport requirements are met, EDP discovery metatraffic and user data traffic will be sent through this transport.
In order to capture the traffic and analyze it, the SHM transport is going to be disabled.
Save the following XML configuration file at the desired location, which will be referred as ``<path_to_xml_config_file>`` from here onwards, with ``vulcanexus_disable_shm.xml`` name.

.. literalinclude:: /resources/tutorials/core/deployment/static_edp//vulcanexus_disable_shm.xml
    :language: xml

This configuration disables the builtin transports and registers a user-defined UDPv4 transport.

After launching Wireshark (with administrative privileges to be able to capture network traffic), start capturing the traffic going through ``any`` interface.
The traffic shown can be filtered using as parameter ``rtps`` to see only traffic of interest.
Next, run the talker and the listener:

.. tabs::

    .. tab:: Shell 1 (Listener)

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash
            FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_config_file>/vulcanexus_disable_shm.xml ros2 run demo_nodes_cpp listener

    .. tab:: Shell 2 (Talker)

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash
            FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_config_file>/vulcanexus_disable_shm.xml ros2 run demo_nodes_cpp talker

The traffic captured will show several ``DATA(p)`` messages which are the periodic DomainParticipant announcements sent to multicast (PDP) and to any DomainParticipant already matched (Liveliness).
In this case, the EDP discovery traffic will also be captured as ``DATA(w)``, DataWriter discovery metatraffic, and ``DATA(r)``, DataReader discovery metatraffic.
The screenshot below has been taken running this tutorial.

.. figure:: /rst/figures/tutorials/core/static_edp/standard_discovery_traffic.png

.. _tutorials_deployment_static_edp_discovery:

Static EDP discovery
--------------------

Now, Static EDP discovery is going to be enabled in order to remove any EDP discovery metatraffic.
Within Vulcanexus ecosystem, Static EDP can only be enabled through XML configuration.
Whereas different Endpoint profiles can be defined within the same XML file using the Topic name, it is not possible to have different DomainParticipant profiles within the same file when using Vulcanexus.
Consequently, a different configuration file is required for each DomainParticipant, because the Static EDP settings include the path to the static XML configuration file which defines the Endpoints, Topics, and Data types.

Save the following XML configuration files in the ``path_to_xml_config_file`` under the names ``talker_profile.xml`` and ``listener_profile.xml`` respectively:

.. tabs::

  .. tab:: talker_profile.xml

    .. literalinclude:: /resources/tutorials/core/deployment/static_edp//talker_profile.xml
      :language: xml

  .. tab:: listener_profile.xml

    .. literalinclude:: /resources/tutorials/core/deployment/static_edp//listener_profile.xml
      :language: xml

Besides setting the EDP discovery to ``STATIC`` and defining the path to the Static configuration XML file, the Endpoint ``userDefinedID`` must be set so the Endpoints can be identified in this Static configuration file.
Every remote Endpoint must be defined within this configuration file, with the Topic and Data type name so matching can be performed by the local DomainParticipant statically without the need for the remote participant to send the Endpoint discovery information.
The specific endpoint QoS has also to be defined or the default QoS will be assumed.

Save the static configuration XML file to the same location under the name ``static_edp_info.xml``:

.. literalinclude:: /resources/tutorials/core/deployment/static_edp//static_edp_info.xml
  :language: xml

It is important to be aware of `ROS 2 Topic mangling rules <https://design.ros2.org/articles/topic_and_service_names.html#mapping-of-ros-2-topic-and-service-names-to-dds-concepts>`_.
Whereas the endpoint profile has to be set using the ROS 2 Topic name, the static discovery configuration must used the DDS Topic name.

Run again the talker-listener demo loading the corresponding XML configuration file using ``FASTRTPS_DEFAULT_PROFILES_FILE`` environment variable:

.. tabs::

    .. tab:: Shell 1 (Listener)

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash
            FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_config_file>/listener_profile.xml ros2 run demo_nodes_cpp listener

    .. tab:: Shell 2 (Talker)

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash
            FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_config_file>/talker_profile.xml ros2 run demo_nodes_cpp talker

If the traffic is captured again using Wireshark, this time no ``DATA(w)`` or ``DATA(r)`` is exchanged, but communication is established.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/deployment/static_edp/static_edp.mp4">
        Your browser does not support the video tag.
    </video>

.. _tutorials_deployment_static_edp_fullfledged_discovery:

Full-fledged Static EDP discovery
---------------------------------

ROS 2 provides several introspection tools like ``ros2 topic list`` or ``ros2 node list``.
Running these commands with the previous example will show only the default ROS 2 topics (``/parameter_events`` and ``/rosout``) and no node.
The ``/chatter`` topic and the ``/talker`` and ``/listener`` nodes would not appear even though they are up and running and communicating.
The issue is related to the internal endpoints that ROS 2 ecosystem, and by extension Vulcanexus, launch within each node to ensure these tools work as expected.
Also, the vast majority of these tools depend on another DomainParticipant known as the ROS 2 Daemon.
This section will complete the previous XML files so a full-fledged communication within Vulcanexus ecosystem is achieved, with working introspection tools.
Information about every internal ROS 2 endpoint has been included.
These endpoints are related to topic and node discovery, ROS 2 logging module, and parameters services and events.

Below, the complete XML files are shown.
Please, replaced the previous XML configuration files with these ones and the new one required to configure the ROS 2 daemon DomainParticipant.

.. tabs::

  .. tab:: talker_profile.xml

    .. literalinclude:: /resources/tutorials/core/deployment/static_edp//complete_talker_profile.xml
      :language: xml

  .. tab:: listener_profile.xml

    .. literalinclude:: /resources/tutorials/core/deployment/static_edp//complete_listener_profile.xml
      :language: xml

  .. tab:: ros2_daemon_profile.xml

    .. literalinclude:: /resources/tutorials/core/deployment/static_edp//ros2_daemon_profile.xml
      :language: xml

  .. tab:: static_edp_info.xml

    .. literalinclude:: /resources/tutorials/core/deployment/static_edp//complete_static_edp_info.xml
      :language: xml

Run again the talker-listener demo loading the corresponding XML configuration file using ``FASTRTPS_DEFAULT_PROFILES_FILE`` environment variable.
This time a third terminal is required to run the ROS 2 daemon.

.. tabs::

    .. tab:: Shell 1 (Listener)

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash
            FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_config_file>/listener_profile.xml ros2 run demo_nodes_cpp listener

    .. tab:: Shell 2 (Talker)

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash
            FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_config_file>/talker_profile.xml ros2 run demo_nodes_cpp talker

    .. tab:: Shell 3 (ROS 2 Daemon)

        .. code-block:: bash

            source /opt/vulcanexus/humble/setup.bash
            ros2 daemon stop
            FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_config_file>/ros2_daemon_profile.xml ros2 daemon start
            ros2 topic list
            ros2 node list

Now, besides the internal ROS 2 Topics, the ``/chatter`` topic will be listed as well as the ROS 2 nodes ``/talker`` and ``/listener``.
