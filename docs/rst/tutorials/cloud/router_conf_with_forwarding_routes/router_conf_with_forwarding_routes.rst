.. include:: ../../../exports/alias.include

.. _tutorials_cloud_forwarding_routes:

ROS 2 Router configuration with Forwarding Routes
=================================================

Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see the |rosrouter| documentation :ref:`here <vulcanexus_router>`).
That is, ROS 2 nodes such as publishers and subscriptions, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic locations on their own dedicated local networks as if they were all on the same network through the |rosrouter|.

This tutorial explains how to configure a |rosrouter| with forwarding routes.
In particular, we will configure two participants in different domains and communicate them with a |rosrouter|.
The |rosrouter| will have two forwarding routes, one generic and one specific to a topic.
These routes will allow us to define what participants can subscribe to the data that another participant is publishing.
This configuration can be useful when a participant wants to send sensible data over a |rosrouter| to a subset of participants.

.. note::

    This tutorial is similar to the :ref:`tutorials_router_change_domain` tutorial, since we will launch a *talker* and a *listener* on different domains and connect them with a |rosrouter|.
    The difference between both tutorials is that, in this one, we will set up forwarding routes to limit which participants can receive data.

The DDS protocol defines Domain Id as a parameter for every *DomainParticipant*.
Different entities in different Domain Ids will never discover each other, and thus they will not communicate with each other.
The |rosrouter| can be used as a bridge between ROS 2 Domains, so that every node in a domain can communicate with every other node on another domain, as illustrated in the following figure:

.. figure:: /rst/figures/tutorials/cloud/change_domain_forwarding_routes.png
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

The following YAML configuration file configures a |rosrouter| with two Simple Participants in domains ``0`` and ``1``, and establishes forwarding routes between them.

.. note::

    This configuration enables listeners in domain ``1`` to subscribe to messages published in domain ``0`` for any topic.
    It also enables listeners in domain ``0`` to subscribe to messages published in domain ``1`` only for the topic ``secret``.

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_forwarding_routes/router_conf_with_forwarding_routes.yaml
    :language: yaml


Simple Participants
"""""""""""""""""""

The Simple Participants are configured with a name, a kind (``local``), and a domain id (``0`` and ``1``).

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_forwarding_routes/router_conf_with_forwarding_routes.yaml
    :language: yaml
    :lines: 3-11


Generic Forwarding Routes
"""""""""""""""""""""""""

We define the generic forwarding routes under the tag ``routes``.
This route is configured so that ``ROS_2_Domain_1`` can subscribe to the data published by ``ROS_2_Domain_0``.

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_forwarding_routes/router_conf_with_forwarding_routes.yaml
    :language: yaml
    :lines: 15-17

This route is configured so that ``ROS_2_Domain_1`` does not publish the data it receives.
Thus, a subscriber in domain ``0`` would not receive the data published in domain ``1``.

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_forwarding_routes/router_conf_with_forwarding_routes.yaml
    :language: yaml
    :lines: 19


Topic Forwarding Routes
"""""""""""""""""""""""

We define the topic forwarding routes under the tag ``topic-routes`` by declaring the topic's ``name``, ``type``, and ``routes``.

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_forwarding_routes/router_conf_with_forwarding_routes.yaml
    :language: yaml
    :lines: 21-26

.. note::

    The ``type`` tag is required.
    The topic forwarding route will not be set for a topic with the same ``name`` but a different ``type``.

Then, we declare the route for each participant.

.. warning::

    When a topic forwarding route is defined for a specific topic, the generic forwarding route gets completely ignored and the topic forwarding route is used instead.

This route is configured so that ``ROS_2_Domain_0`` will subscribe to the data published by ``ROS_2_Domain_1`` on topic ``secret``.

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_forwarding_routes/router_conf_with_forwarding_routes.yaml
    :language: yaml
    :lines: 28-30

This route is configured so that ``ROS_2_Domain_0`` does not forward the data it receives on topic ``secret``.

.. literalinclude:: /resources/tutorials/cloud/router_conf_with_forwarding_routes/router_conf_with_forwarding_routes.yaml
    :language: yaml
    :lines: 32


Running the ROS 2 Router
^^^^^^^^^^^^^^^^^^^^^^^^

Run the |ddsrouter| with the configuration file available at ``<path/to/file>/ros_2_router_with_forwarding_routes.yaml``.

.. code-block:: bash

    ddsrouter --config-path <path/to/file>/ros_2_router_with_forwarding_routes.yaml

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

Publish in domain 0 and subscribe in domain 1
"""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

Since the |rosrouter| has a generic forwarding route from domain ``0`` and ``1``, it will forward messages from the *publisher* in domain ``0`` to the *subscriber* in domain ``1``, that will print them in ``stdout``.

Publish in domain 1 and subscribe in domain 0
"""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp talker

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp listener

Since the |rosrouter| does not have a generic forwarding route from domain ``1`` and ``0``, it will not forward messages from the *publisher* in domain ``1`` to the *subscriber* in domain ``0``.


Running ROS 2 nodes on topic secret
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Publish in domain 0 and subscribe in domain 1 on topic secret
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker --ros-args -r chatter:=secret

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener --ros-args -r chatter:=secret

Since the |rosrouter| does not have a topic forwarding route on topic ``secret`` from domain ``0`` and ``1``, it will not forward messages from the *publisher* in domain ``0`` to the *subscriber* in domain ``1``.

Publish in domain 1 and subscribe in domain 0 on topic secret
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Run a ROS 2 ``demo_nodes_cpp`` *talker* on domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp talker --ros-args -r chatter:=secret

Run a ROS 2 ``demo_nodes_cpp`` *listener* on domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp listener --ros-args -r chatter:=secret

Since the |rosrouter| has a topic forwarding route on topic ``secret`` from domain ``1`` and ``0``, it will forward messages from the *publisher* in domain ``1`` to the *subscriber* in domain ``0``, that will print them in ``stdout``.
