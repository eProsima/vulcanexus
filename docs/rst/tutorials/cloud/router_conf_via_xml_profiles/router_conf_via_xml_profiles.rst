.. include:: ../../../exports/alias.include

.. _tutorials_cloud_xml_participant_configuration:

ROS 2 Router configuration via XML profiles
===========================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see the |rosrouter| documentation :ref:`here <vulcanexus_router>`).
That is, ROS 2 nodes such as publishers and subscriptions, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic locations on their own dedicated local networks as if they were all on the same network through the use of |rosrouter|.

This tutorial explains how to configure a Participant with XML. In particular, we will configure two Participants, one without XML on domain ``0`` and one with XML on domain ``1``, and use the |rosrouter| to allow them to communicate between each other.

.. note::

    This tutorial is similar to the :ref:`tutorials_router_change_domain` tutorial, since we will launch a *talker* and a *listener* on different domains and connect them with the |rosrouter|. The difference between both tutorials is that, in this one, one of the Participants will be configured using Fast DDS XML profiles.

The DDS protocol defines Domain Id as a parameter for every *DomainParticipant*.
Different entities in different Domain Ids will never discover each other, and thus they will not communicate with each other.
The |rosrouter| can be used as a bridge between ROS 2 Domains, so that every node in a domain can communicate with every other node on another domain, as illustrated in the following figure:

.. figure:: /rst/figures/tutorials/cloud/change_domain_xml.png
   :align: center

This tutorial will use the ``demo_nodes_cpp`` package, available in the Vulcanexus Desktop distribution.
Two ROS 2 nodes, a *talker* and a *listener*, will be launched on different ROS 2 Domains, so that they cannot communicate between each other.
Then, the |rosrouter| will be used as a bridge between the two Domains, allowing the *listener* to receive the messages from the *talker*.


Prerequisites
-------------

To proceed, please install Vulcanexus with one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Deploy ROS 2 nodes
------------------

Let us start by running the ROS 2 *talker* and *listener* nodes.

Environment setup
^^^^^^^^^^^^^^^^^

To run the nodes, we need to set up the Vulcanexus environment so that the ``demo_nodes_cpp`` package is available.
There are two ways to achieve this:

#.  Running the Vulcanexus Docker image.

    Run the Vulcanexus Docker image by executing:

    .. code-block:: bash

        docker run -it ubuntu-vulcanexus:jazzy-desktop

    And then, source the Vulcanexus installation by executing (inside the container):

    .. code-block:: bash

            source /opt/vulcanexus/jazzy/setup.bash

#.  Setting up a development environment on the local host.

    To do this, the ``vucanexus-jazzy-desktop`` package is needed, since it includes all the simulation tools, demos, and tutorials.

    Set up the Vulcanexus environment by executing:

    .. code-block:: bash

            source /opt/vulcanexus/jazzy/setup.bash

Running ROS 2 nodes
^^^^^^^^^^^^^^^^^^^

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener


At this point, the *listener* should not receive any data from the *talker* since they are in different domains. To connect them, we will use the |rosrouter|.


Deploy ROS 2 Router
-------------------

To deploy the router, we need to create the |rosrouter| configuration.

The following YAML configuration file configures a |rosrouter| to create a Simple Participant on domain ``0``
and a Participant configured by XML on domain ``1``.
This configuration is similar to the one in :ref:`tutorials_router_change_domain`: it generates a bridge between two domains (``0`` and ``1``).

.. literalinclude:: /resources/tutorials/cloud/router_conf_via_xml_profiles/router_conf_via_xml_profiles.yaml
    :language: yaml

Participant XML Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

|rosrouter| supports loading XML configuration files to load profiles.
These profiles are used to configure different DomainParticipants using profile names.
Loading an XML file or setting the raw xml file in the |ddsrouter| YAML configuration file allows to load such profiles.
Here there are the two ways to load them.
For more information check the `Load XML Configuration <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/configuration.html#user-manual-configuration-load-xml>`.

.. literalinclude:: /resources/tutorials/cloud/router_conf_via_xml_profiles/router_conf_via_xml_profiles.yaml
    :language: yaml
    :lines: 3-13


Simple Participant Domain 0
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Simple Participant is configured with a name, a kind, and a domain id (``0`` in this case).

.. literalinclude:: /resources/tutorials/cloud/router_conf_via_xml_profiles/router_conf_via_xml_profiles.yaml
    :language: yaml
    :lines: 17-19


XML Participant Domain 1
^^^^^^^^^^^^^^^^^^^^^^^^

The XML Participant is configured with a name, a kind, and an XML ``profile`` tag that will be used to configure it.
The XML configures the profile ``custom_participant_configuration`` as a default Participant on domain ``1``, so this Participant will run as a ``local`` participant on domain ``1``.

.. literalinclude:: /resources/tutorials/cloud/router_conf_via_xml_profiles/router_conf_via_xml_profiles.yaml
    :language: yaml
    :lines: 21-23


Running ROS 2 Router
^^^^^^^^^^^^^^^^^^^^

Run the |ddsrouter| with the configuration file available at ``<path/to/file>/ros_2_router_xml_config.yaml``.

.. code-block:: bash

    ddsrouter --config-path <path/to/file>/ros_2_router_xml_config.yaml

The output from the |rosrouter| should be something like:

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.

If so, the |rosrouter| has started correctly and it is currently running. Once the |ddsrouter| is running, it will forward the messages from the *talker* on domain 0 to the *listener* on domain 1.
In order to close the execution, press ^C or send a signal (:code:`SIGINT 2` or :code:`SIGTERM 15`) to close it.
