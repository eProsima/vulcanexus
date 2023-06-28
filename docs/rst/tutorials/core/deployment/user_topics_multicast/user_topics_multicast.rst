.. _tutorials_core_deployment_user_topics_multicast:

Enabling multicast communication
================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

.. _tutorials_core_deployment_user_topics_multicast_background:

Background
----------

.. note::

   This documentation assumes basic knowledge of UDP/TCP/IP concepts, namely unicast and multicast.

When communicating nodes across different hosts, Vulcanexus applications, by default, utilize UDPv4 based unicast communication with one another (between nodes on the same host, the shared memory transport is used by default, which defaults to multicast communications).
This is because multicast based communication is not always a possibility in certain deployment for a myriad of reasons, among them multicast not performing well over certain WiFi networks or IT constrains.
However, for deployments in which multicast is an available and reliable means of communication, Vulcanexus generated network traffic can be significantly reduced by leveraging multicast communications, which may also reduce Vulcanexus overhead in terms of CPU utilization and latency while increasing message throughput as the messages need to be copied to the network buffer but once (in opposition to unicast based communications, in which each message needs to be copied once per recipient).

This tutorial showcases how to enable multicast communication between Vulcanexus nodes communicating over a UDPv4 transport by :ref:`tutorials_xml_profiles_intro`.

.. _tutorials_core_deployment_user_topics_multicast_prerequisites:

Prerequisites
-------------

For accomplishing this tutorial, two available hosts with Docker and a Vulcanexus image are required.
Please refer to the installation steps detailed in :ref:`docker_installation`.

.. _tutorials_core_deployment_user_topics_multicast_config:

XML Configuration
-----------------

Save the following XML configuration file at the desired location, which will be referred as ``<path_to_xml_config_file>`` from here onwards.

.. important::

    The Data Reader profile name shall match the topic's Fully Qualified Name (FQN) (see :ref:`vulcanexus_configure_pubsub_profile_names`)

.. literalinclude:: /resources/tutorials/core/deployment/user_topics_multicast/user_topics_multicast.xml
    :language: xml

.. _tutorials_core_deployment_user_topics_multicast_example:

Run the example
---------------

This tutorial leverages the Vulcanexus `humble-desktop` Docker image to demonstrate the use of the aforementioned XML
configuration file to achieve multicast communication across different hosts.

First, run two Vulcanexus `humble-desktop` with:

.. tabs::

  .. tab:: Container one (Subscription)

    .. code-block:: bash

        docker run \
            --interactive \
            --tty \
            --rm \
            --volume <path_to_xml_config_file_dir>:/root/xml_config \
            ubuntu-vulcanexus:humble-desktop

  .. tab:: Container two (Publication)

    .. code-block:: bash

        docker run \
            --interactive \
            --tty \
            --rm \
            ubuntu-vulcanexus:humble-desktop

Then, within the container, run the talker-listener demo.

.. tabs::

  .. tab:: Container one (Subscription)

    .. code-block:: bash

        source /vulcanexus_entrypoint.sh
        export FASTRTPS_DEFAULT_PROFILES_FILE=/root/xml_config/<xml_config_file>
        ros2 run demo_nodes_cpp listener

  .. tab:: Container two (Publication)

    .. code-block:: bash

        source /vulcanexus_entrypoint.sh
        ros2 run demo_nodes_cpp talker

Optionally, as shown in the following video, it is possible to use Wireshark to sniff the RTPS traffic to corroborate the specified multicast address in indeed used:

.. raw:: html

    <video width=100% height=auto autoplay loop controls>
        <source src="../../../../../_static/resources/tutorials/core/deployment/user_topics_multicast/user_topics_multicast.mp4">
        Your browser does not support the video tag.
    </video>

