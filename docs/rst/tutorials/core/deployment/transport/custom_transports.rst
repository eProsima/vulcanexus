.. _tutorials_core_deployment_custom_transports:

Customizing Network Transports
==============================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

.. _tutorials_core_deployment_custom_transports_background:

Background
----------

Vulcanexus applications, by default, enable two different transports; a UDPv4 transport for inter-host communication, and a shared memory transport (SHM) for inter-process communications with other applications running on the same host.
Although the default-enabled transports may be suited for most use-cases, they do not intend to cover the entire spectrum of Vulcanexus deployments.
Among the many use-cases that may benefit from a custom transport layer, there are:

1. Applications communicating over IPv6 networks
2. NAT traversing communications over TCP (see :ref:`intro_metapackages_cloud`)
3. No-network deployments
4. Communications enabling TLS
5. Limiting Vulcanexus traffic to a subset of the host's network interfaces
6. Limiting IP fragmentation (specially relevant for reliable data delivery over lossy networks)

For these reasons, Vulcanexus allows for the configuration of network and SHM transports leveraging Fast DDS capabilities to configure the middleware's `transport layer <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/transport.html>`_.
This is achieved by defining the appropriate transport description in a `Fast DDS XML configuration file <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_.

This tutorial showcases a configuration in which the Vulcanexus communication is limited to a single network interface (localhost) while avoiding Vulcanexus-sent UDP datagrams to be fragmented at the IP level.
This is achieved by setting an interface whitelist, and by limiting the size of the Vulcanexus RTPS (the underlying DDS wire-protocol) datagrams so that complete UDP packets fit into the system's MTU (typically 1500 B).

The latter is specially beneficial to achieve reliable communication over DDS (through a UDPv4 network transport) when communicating over a lossy network.
This is because, although the underlying RTPS protocol can be configured as reliable, UDPv4 is a best-effort protocol over IP (another best-effort protocol), and as such is susceptible to network losses.
If the RTPS datagrams (which become payloads of UDPv4 datagrams) have the maximum allowed size for a UDP payload (~65 kiB), then the resulting packet would need to be fragmented into 44 fragments (assuming an 1500 B MTU).
In this scenario, losing one of those 44 fragments entails losing the entire UDP datagram, and consequently the entire RTPS datagram.
If the network's packet drop rate is 1/44 or higher, no UDP datagram will ever be reconstructed, resulting in no RTPS datagram ever received, and therefore the RTPS reliability cannot succeed at all.
By fragmenting the DDS data into several self-contained, smaller-than-the-MTU UDP datagrams, 43 out of every 44 RTPS fragments will be received at first try (in the considered scenario), and the RTPS reliability will be able to retransmit the missing ones successfully.

.. _tutorials_core_deployment_custom_transports_prerequisites:

Prerequisites
-------------

The first prerequisite is to have Vulcanexus Humble installed (see `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_ or `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`_).

Please, remember to source the environment in every terminal used during this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

Install the `ROS 2 image demo package <https://github.com/ros2/demos/tree/humble/image_tools>`_ (administrative privileges may be required):

.. code-block:: bash

    apt update && apt install -y ros-humble-image-tools