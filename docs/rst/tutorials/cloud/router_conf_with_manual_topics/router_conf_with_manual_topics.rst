.. include:: ../../../exports/alias.include

.. _tutorials_cloud_manual_topics:

ROS 2 Router configuration with Manual Topics
=============================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see the |rosrouter| documentation :ref:`here <vulcanexus_router>`).
That is, ROS 2 nodes such as publishers and subscriptions, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic locations on their own dedicated local networks as if they were all on the same network through the |rosrouter|.

This tutorial explains how to configure a |rosrouter| with manual topics.
In particular, we will configure two participants in different domains and communicate them with a |rosrouter|.
The |rosrouter| will configure a Topic QoS on one of the participants with a manual topic.
These Manual Topics will allow us to configure the QoS of the DDS Router's internal entities.
This configuration can be useful when, for instance, a user wants to limit the transmission frequency on a specific topic for a specific participant.

.. note::

    This tutorial is similar to the :ref:`tutorials_router_change_domain` tutorial, since we will launch a *talker* and a *listener* on different domains and connect them with a |rosrouter|.
    The difference between both tutorials is that, in this one, we will limit the transmission rate for a participant on a topic to avoid clogging the connection.

The DDS protocol defines Domain Id as a parameter for every *DomainParticipant*.
Different entities in different Domain Ids will never discover each other, and thus they will not communicate with each other.
The |rosrouter| can be used as a bridge between ROS 2 Domains, so that every node in a domain can communicate with every other node on another domain, as illustrated in the following figure:

.. figure:: /rst/figures/tutorials/cloud/change_domain_manual_topics.png
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


Deploying a ROS 2 Router
------------------------

To deploy the |rosrouter|, we need to create its configuration file.

Configuration
^^^^^^^^^^^^^

The following YAML configuration file configures a |rosrouter| with two Simple Participants in domains ``0`` and ``1``, and a manual topic to limit the transmission rate on one of the participants for a specific topic.

.. note::

    This configuration enables listeners in domain ``1`` to subscribe to messages published in domain ``0``, at a frequency of 1 Hz on topic ``secret`` , and at an unlimited frequency on any other topic.
    It also enables listeners in domain ``0`` to subscribe to messages published in domain ``1`` at an unlimited frequency on any topic.

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_manual_topics/router_conf_with_manual_topics.yaml
    :language: yaml


Simple Participants
"""""""""""""""""""

The Simple Participants are configured with a name, a kind (``local``), and a domain id (``0`` and ``1``).

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_manual_topics/router_conf_with_manual_topics.yaml
    :language: yaml
    :lines: 10-17


Manual Topic
""""""""""""

We define the manual topic under the tag ``routes``.
This manual topic is configured so that ``ROS_2_Domain_1`` can subscribe to the data published by ``ROS_2_Domain_0`` at a maximum frequency of 1 Hz on topic ``secret``, at an unlimited

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_manual_topics/router_conf_with_manual_topics.yaml
    :language: yaml
    :lines: 3-8


Running the ROS 2 Router
^^^^^^^^^^^^^^^^^^^^^^^^

Run the |ddsrouter| with the configuration file available at ``<path/to/file>/ros_2_router_with_manual_topics.yaml``.

.. code-block:: bash

    ddsrouter --config-path <path/to/file>/ros_2_router_with_manual_topics.yaml

The output from the |rosrouter| should be something like:

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.

If so, the |rosrouter| has started correctly and it is currently running.
In order to close the execution, press ^C or send a signal (:code:`SIGINT 2` or :code:`SIGTERM 15`) to close it.


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

Publish in domain 0 and subscribe in domain 1
"""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

Since the |rosrouter| does not have a manual topic to configure the ``max-tx-rate`` for the topic ``secret`` on the ``ROS_2_Domain_1`` participant, the listener will receive messages at an unlimited rate and it will print them in ``stdout``.


Publish in domain 0 and subscribe in domain 1 on topic secret
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker --ros-args -r chatter:=secret

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener --ros-args -r chatter:=secret

Since the |rosrouter| has a manual topic to configure the ``max-tx-rate`` for the topic ``secret`` on the ``ROS_2_Domain_1`` participant, the listener will receive messages at most at 1Hz and it will print them in ``stdout``.


Publish in domain 1 and subscribe in domain 0 on topic secret
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp talker

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp listener

Since the |rosrouter| does not have a manual topic to configure the ``max-tx-rate`` for the topic ``secret`` on the ``ROS_2_Domain_0`` participant, the listener will receive messages at an unlimited rate and it will print them in ``stdout``.
