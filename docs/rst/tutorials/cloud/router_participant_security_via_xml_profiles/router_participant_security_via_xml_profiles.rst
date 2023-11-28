.. _tutorials_ddsrouter_participant_via_xml_profiles:

ROS 2 Router configuration of DomainParticipants Security via XML profiles
==========================================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is a cross-platform non-graphical application developed by eProsima that allows users to connect isolated DDS networks.
These networks could be isolated one to each other due to the Transport Protocol (UDP, TCP, etc.), the Discovery Protocol (Simple, Discovery Server, etc.) or the DDS Domain Id used by each DDS entity.
Configuring the DDS Router to have one Participant for each isolated network will allow to stablish the communication and data transmission between the entities.
In compliance with the DDS Security specification, Fast DDS provides secure communication by implementing pluggable security at three levels:
a) DomainParticipants authentication (DDS:Auth:PKI-DH), b) access control of Entities (DDS:Access:Permissions), and c) data encryption (DDS:Crypto:AES-GCM-GMAC).
Fast DDS allows security plugins to be activated through the `DomainParticipantQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#dds-layer-domainparticipantqos>`_ properties.

In this tutorial, the configuration of two DDS Router Participants will be undertaken through the utilization of XML Profiles, thereby imbuing them with enhanced security features.
Specifically, these Participants will facilitate communication between two distinct domains, a capability that would be unattainable in the absence of the Router.
The chosen security configuration in this case will be related to authentication, access control, and data encryption.
To know more about Security plugins, please refer to `Fast DDS Security Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html>`_.

.. figure:: /rst/figures/tutorials/cloud/security_participants.png
   :align: center

This tutorial will use the ``demo_nodes_cpp`` package, available in the Vulcanexus Desktop distribution.
Two ROS 2 nodes, a *talker* and a *listener*, will be launched on different ROS 2 Domains, so that they cannot communicate between each other in a first instance.
Then, the `DDS Router <https://github.com/eProsima/DDS-Router>`_ will be used as a bridge between the two Domains, checking the security features between the Participants and allowing the *listener* to receive the messages from the *talker*.


Prerequisites
-------------

To proceed, please install Vulcanexus with one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Deploy ROS 2 nodes
------------------

Let us start by running the ROS 2 *talker* and *listener* nodes from the ``demo_nodes_cpp`` package.

Environment setup
^^^^^^^^^^^^^^^^^

To run the nodes, we need to set up the Vulcanexus environment so that the ``demo_nodes_cpp`` package is available.
There are two ways to achieve this:

#.  Running the Vulcanexus *ubuntu-vulcanexus:iron-desktop* Docker image, available in `Vulcanexus Download page <https://vulcanexus.org/download>`_.

    Run the Vulcanexus Docker image by executing:

    .. code-block:: bash

        docker run -it ubuntu-vulcanexus:iron-desktop

    And then, source the Vulcanexus installation by executing (inside the container):

    .. code-block:: bash

            source /opt/vulcanexus/iron/setup.bash

#.  Setting up a development environment on the local host.

    To do this, the ``vucanexus-iron-desktop`` package is needed, since it includes all the simulation tools, demos, and tutorials.

    Set up the Vulcanexus environment by executing:

    .. code-block:: bash

            source /opt/vulcanexus/iron/setup.bash

Running ROS 2 nodes
^^^^^^^^^^^^^^^^^^^

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener


At this point, the *listener* should not receive any data from the *talker* since they are in different domains. To connect them, we will use the `DDS Router <https://github.com/eProsima/DDS-Router>`_.


Deploy ROS 2 Router
-------------------

The following YAML configuration file configures a `DDS Router <https://github.com/eProsima/DDS-Router>`_ to create 2 Participans in Domains ``0`` and ``1`` so that it creates a bridge between the isolated networks enabling communication between them.

.. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/ddsrouter.yaml
    :language: yaml

DDS Router Participants Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

DDS Router `Participant <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/user_manual_glossary.html#term-Participant>`_ is a DDS Router entity that works as an interface between a network and the core of the router.
Participants security can only be configured by XML, following the instructions of `Fast DDS Security Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html>`_ to fulfill the pertinent requirements of the chosen security mechanisms.
To generate the security certificates needed for both XML Participants follow this `ROS 2 Tutorial <https://docs.ros.org/en/iron/Tutorials/Advanced/Security/Introducing-ros2-security.html>`_.

Once the certificates are generated, introduce their paths in the XML Participants configuration:

.. tabs::

    .. tab:: Participant A

        .. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/configurations/secure_configurationA.xml
            :language: xml

    .. tab:: Participant B

        .. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/configurations/secure_configurationB.xml
            :language: xml

These XMLs snippet configure security settings for two participants in the Fast DDS middleware, focusing on authentication through the DDS:Auth:PKI-DH plugin.
They activate the PKI-DH plugin, specifying the paths to the participants' identities certificateS, identity CA certificate, and private keys for authentication.
Additionally, they enable access control through the DDS:Access:Permissions plugin, defining paths to the permissions CA certificate, governance files, and permissions files.
The configuration also includes the activation of the DDS:Crypto:AES-GCM-GMAC plugin for data encryption, enhancing communication security within the Fast DDS framework.

Running ROS 2 Router
^^^^^^^^^^^^^^^^^^^^

Run the `DDS Router <https://github.com/eProsima/DDS-Router>`_ with the configuration file available at ``<path/to/file>/ddsrouter.yaml``.

.. code-block:: bash

    ddsrouter --config-path <path/to/file>/ddsrouter.yaml

The output from the `DDS Router <https://github.com/eProsima/DDS-Router>`_ should be something like:

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.

If so, the `DDS Router <https://github.com/eProsima/DDS-Router>`_ has started correctly and it is currently running. Once the `DDS Router <https://github.com/eProsima/DDS-Router>`_ is running, it will forward the messages from the *talker* on domain 0 to the *listener* on domain 1.
In order to close the execution, press ^C or send a signal (:code:`SIGINT 2` or :code:`SIGTERM 15`) to close it.
