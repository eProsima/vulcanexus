.. include:: ../../../exports/alias.include

.. _tutorials_router_change_domain_to_discovery_server:

Change ROS 2 Domain to Discovery Server
=======================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://github.com/eProsima/DDS-Router>`_, is an end-user software application that enables the connection of distributed ROS 2 networks (see |rosrouter| documentation :ref:`here <vulcanexus_router>`).
That is, ROS 2 nodes such as publishers and subscriptions, or clients and services, deployed in one geographic location and using a dedicated local network will be able to communicate with other ROS 2 nodes deployed in different geographic areas on their own dedicated local networks as if they were all on the same network through the use of |rosrouter|.

This tutorial explains how to connect ROS 2 nodes using :ref:`Simple Discovery <tutorials_router_change_domain_to_discovery_server_simple_discovery>` with other nodes that are connected vua :ref:`Discovery Server <tutorials_router_change_domain_to_discovery_server_discovery_server>`.
These entities are unable to discover each other if using different discovery mechanisms, and thus they will not communicate to each other.
Using the |rosrouter| as a bridge between these 2 mechanisms, every node will be able to communicate with any other node independently of the Domain where they are deployed.

As already mentioned, the approach of this tutorial is straightforward and is illustrated in the following figure:

.. figure:: /rst/figures/tutorials/cloud/change_domain_to_ds.png
   :align: center

This tutorial will use the ``demo_nodes_cpp`` package, available in the Vulcanexus Desktop distribution.
First, a ROS 2 ``talker`` is launched and then a ``listener`` node connected to a ``discovery server``.
One of the nodes will be executed normally and will use *Simple Discovery* while the other will use |fastdds| `Discovery Server Environmenta Variable <>`__ to connect with the *Discovery Server*.
This will prevent the two from communicating.
At this point, the |rosrouter| will be deployed as a bridge between the two Domains and will enable the ``talker``-``listener`` communication.

DDS Discovery Mechanisms
------------------------

The *DDS* protocol define different `Discovery Mechanisms <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html#discovery-mechanisms>`__ so *Domain Participants* will discover each other automatically.
This mechanism can be configured by the user.

.. _tutorials_router_change_domain_to_discovery_server_simple_discovery:

Simple Discovery
^^^^^^^^^^^^^^^^

This mechanism is the standard behavior of DDS and it uses multicast capabilities so every Node discovers any other node in the network and all its *Endpoints*.
This is the mechanism used by default in ROS 2.
Nodes can be isolated by using different *Domain Ids*.
Please, check the following :ref:`tutorial <tutorials_router_change_domain>` to know how to communicate ROS 2 Nodes in different *Domain Ids*.

.. _tutorials_router_change_domain_to_discovery_server_discovery_server:

Discovery Server
^^^^^^^^^^^^^^^^

Some networks do not support multicast, or are so big that discovering the whole network implies high amount of time.
In this cases, a new *Discovery Mechanism* specific of |fastdds| can be used, called **Discovery Server Discovery Mechanism**.
This mechanisms uses a *Server* or network of *Servers* that centralize the discovery traffic and distributes it to only the required Nodes, while the communication still occurs *Peer to Peer*.
In order to know more about *Discovery Server*, please refer to `Fast DDS Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html>`__ or check a tutorial on how to use it `<https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Advanced/Discovery-Server/Discovery-Server.html>`__ .


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

Running ROS 2 Discovery Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to run the *Discovery Server* use the |fastdds| CLI.
Take care of using the public IP of the machine hosting the *Discovery Server* and an available port.

.. code-block:: bash

    fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11666

.. note::

    The IP used in this tutorial is ``localhost`` and thus it will only work if all the commands are executed
    in the same host.

Running ROS 2 nodes
^^^^^^^^^^^^^^^^^^^

Once the environment has been setup using one of the above options, run the ROS 2 ``talker`` node in one terminal.

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

Then, on another terminal, run the ROS 2 ``listener`` node in ROS 2 Domain ``1``.

.. code-block:: bash

    ROS_DISCOVERY_SERVER="127.0.0.1:11666" ros2 run demo_nodes_cpp listener


At this point, the ``listener`` should not receive any data.
If not, please go again through the previous steps.


Deploy ROS 2 Router
-------------------

Then, create the |rosrouter| configuration file as the one shown below.

.. note::

    There is an available `configuration file <https://github.com/eProsima/DDS-Router/blob/main/resources/configurations/examples/change_domain.yaml>`__ in |ddsrouter| repository.

.. note::

    If deploying Vulcanexus from the Docker image, note that you will need to have a configuration file (config.yaml) for the |rosrouter| Edge accessible from your Docker container.
    This can be achieved by mounting a shared volume when launching the container, by copying the file from the local host to the container in case it is already running, or by editing a file from the Docker container itself.

.. todo:

..     Vulcanexus come already with a configuration file for changing domain from ``0`` to ``1`` installed in ``/opt/vulcanexus/humble/share/ddsrouter_tool/resources/configurations/examples/change_domain.yaml``.
..     This file can be used in :code:`ddsrouter` command with :code:`--config-path` argument.

.. literalinclude:: /resources/tutorials/cloud/change_domain/change_domain.yaml
    :language: yaml

This configuration defines 2 different *Router Participants*, internal "interfaces" for the |rosrouter|.
Each of this Participants will create DDS Entities in each of the domains, and they will forward all the data received from one Domain to the other.
Topics, Data Types, Quality of Services and order of messages will be respected when redirecting the data.

Running ROS 2 Router
^^^^^^^^^^^^^^^^^^^^

Now, run the |rosrouter| with the configuration file created as an argument.

.. code-block:: bash

    ddsrouter --config-path <path/to/file>/change_domain.yaml

At this point you should see some information like the one shown below.
This indicates that the |rosrouter| has started correctly and it is currently running.

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.

In order to close the execution, just press ^C or send a signal (:code:`SIGINT 2` or :code:`SIGTERM 15`) to close it.


Communicating multiple Domains
------------------------------

The |rosrouter| can equally inter-communicate 2 or more Domain Ids.
Just add as many Participants as desired to the configuration file and this will redirect all messages from every Domain to all the others.
In the following figure we could see the use case and the configuration required for communicating 4 different Domains.

.. figure:: /rst/figures/tutorials/cloud/change_domain_4.png
   :align: center

.. literalinclude:: /resources/tutorials/cloud/change_domain/change_domain_4.yaml
    :language: yaml
