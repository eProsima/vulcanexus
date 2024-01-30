.. include:: ../../../exports/alias.include

.. _tutorials_cloud_manual_topics:

Configuring ROS 2 Router's Topic QoS
====================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see the |rosrouter| documentation :ref:`here <vulcanexus_router>`).
That is, ROS 2 nodes such as publishers and subscriptions, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic locations on their own dedicated local networks as if they were all on the same network through the |rosrouter|.

This tutorial explains how to apply some configurations to individual DDS Topics forwarded by the |rosrouter|.
In particular, we will configure two participants in different domains and communicate them with a |rosrouter|.
The |rosrouter| will configure a specific Topic for one of the participants using this new topic settings option.
In particular, in this tutorial, we will fix the maximum transmission rate (``max-tx-rate``) of one of the topics for a specific participant, so that only this participant will apply this maximum publication rate only to this topic.

.. note::

    This tutorial is similar to the :ref:`tutorials_router_change_domain` tutorial, since we will launch a *talker* and a *listener* on different domains and connect them with a |rosrouter|.
    The difference between both tutorials is that, in this one, we will limit the transmission rate for a participant on a topic to avoid clogging the connection.

The DDS protocol defines the Domain Id as a parameter for every *DomainParticipant*.
Different entities in different Domain Ids will never discover each other, and thus they will not communicate with each other.
The |rosrouter| can be used as a bridge between ROS 2 Domains, so that every node in a domain can communicate with every other node on another domain, as illustrated in the following figure:

.. figure:: /rst/figures/tutorials/cloud/change_domain_topic_qos.png
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

        docker run -it ubuntu-vulcanexus:iron-desktop

    The Docker container will automatically setup the Vulcanexus environment on start up, but in case you installed Vulcanexus from binaries please do not forget to source the Vulcanexus installation first by executing:

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

First, run the *talker* and *listener* to communicate using the default topic of the ``demo_nodes_cpp`` ROS 2 package, that is, the ``chatter`` topic.

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

So far, the listener will not receive anything as the ROS 2 publisher and the ROS 2 subscription are deployed on different ROS 2 Domains.

Deploying a ROS 2 Router
------------------------

To deploy the |rosrouter|, we need to create its configuration file.

Configuration
^^^^^^^^^^^^^

The following YAML configuration file configures a |rosrouter| with two Simple Participants in domains ``0`` and ``1``, and a topic configuration to limit the transmission rate on one of the participants for a specific topic.

.. note::

    This configuration enables listeners in domain ``1`` to subscribe to messages published in domain ``0``, at a frequency of 1 Hz on topic ``secret``, and at an unlimited frequency on any other topic.
    It also enables listeners in domain ``0`` to subscribe to messages published in domain ``1`` at an unlimited frequency on any topic.

.. literalinclude:: /resources/tutorials/cloud/conf_router_topic_qos/conf_router_topic_qos.yaml
    :language: yaml


Simple Participants
"""""""""""""""""""

The Simple Participants are configured with a name, a kind (``local``), and a domain id (``0`` and ``1``).

.. literalinclude:: /resources/tutorials/cloud/conf_router_topic_qos/conf_router_topic_qos.yaml
    :language: yaml
    :lines: 10-17


Topic Configuration
"""""""""""""""""""

We define the topic under the tag ``topics``.
This topic is configured so that ``ROS_2_Domain_1`` will publish at a maximum frequency of 1 Hz on topic ``secret``, and at an unlimited frequency on any other topic.

.. literalinclude:: /resources/tutorials/cloud/conf_router_topic_qos/conf_router_topic_qos.yaml
    :language: yaml
    :lines: 3-8


Running the ROS 2 Router
^^^^^^^^^^^^^^^^^^^^^^^^

Run the |ddsrouter| with the configuration file available at ``<path/to/file>/ros_2_router_with_manual_topics.yaml``.

.. code-block:: bash

    ddsrouter -c <path/to/file>/ros_2_router_with_manual_topics.yaml

The output from the |rosrouter| should be something like:

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.

If so, the |rosrouter| has started correctly and it is currently running.
In order to close the execution, press ^C or send a signal (:code:`SIGINT 2` or :code:`SIGTERM 15`) to close it.

The *listener* should now receive the messages sent by the *talker* at an unlimited rate as they are communicating through the ``chatter`` topic.


Deploy ROS 2 nodes on topic secret
----------------------------------

Let us now run the ROS 2 *talker* and *listener* nodes on the topic ``secret``.

Running ROS 2 nodes
^^^^^^^^^^^^^^^^^^^

Publish in domain 0 and subscribe in domain 1 on topic secret
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker --ros-args -r chatter:=secret

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener --ros-args -r chatter:=secret

Since the |rosrouter| has configured the ``secret`` topic with a ``max-tx-rate`` for the ``ROS_2_Domain_1`` participant, the listener will receive messages at most at 1 Hz and it will print them in ``stdout``.


Publish in domain 1 and subscribe in domain 0 on topic secret
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp talker --ros-args -r chatter:=secret

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp listener --ros-args -r chatter:=secret

Since the |rosrouter| does not have a specific topic configuration for the topic ``secret`` on the ``ROS_2_Domain_0`` participant, the listener will receive messages at an unlimited rate and it will print them in ``stdout``.


Next steps
----------

For all possible configurations that can be applied to the |rosrouter| topics please refer to the `Topic QoS <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/configuration.html#topic-qos>`__ section of the DDS Router documentation.
