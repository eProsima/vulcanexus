.. _tutorials_deployment_ds_wan_tcp:

TCP over WAN with Discovery Server
==================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

In this tutorial, it is explained how to deploy a TCP communication over WAN using Discovery Server as discovery mechanism.
The following diagram describes de main idea of the deployment tutorial.

.. figure:: /rst/figures/tutorials/core/ds_wan_tcp/ds_wan_tcp_simple.svg
    :align: center
    :width: 70%

    TCP communication over WAN with Discovery Server

TCP: Transmission Control Protocol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Transmission Control Protocol is a Internet Protocol which provides mainly reliability in the communication process.
As it is connection-oriented, this protocol has several features that ensures the delivery, order and error check of the packages.
Despite the fact that the latency is higher than other Internet Protocols such as UDP, its use has several advantages in particular scenarios where reliability has greater importance than the latency cost.

WAN: Wide Area Network
^^^^^^^^^^^^^^^^^^^^^^

A Wide Area Network is a telecommunication network extended over a large geographic area.
It usually involves a large number of nodes and redundancy, to ensure the reliability of the network.
The Internet could be considered as a WAN itself.

Discovery Server
^^^^^^^^^^^^^^^^

The :ref:`Discovery Server <vulcanexus_discovery_server>` is a Fast DDS enabled feature that procures an alternative discovery mechanism to the default ROS 2 discovery mechanism, `Simple Discovery Protocol (SDP) <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#simple-disc-settings>`_, which is served by the DDS implementations according to the DDS specification.
Whereas SDP (right figure) provides automatic out-of-the-box discovery by leveraging multicast, the ROS 2 Discovery Server (left figure) provides a centralized hub for managing discovery which drastically reduces network bandwidth utilization when compared to SDP, since the nodes, publishers, and subscribers, only discover those remote ROS 2 entities with which they need to communicate with, as opposed to the SDP model where everyone knows each other.

.. uml::
    :align: center

    hide empty members

    package ds as "Discovery Server"{
        cloud "Server(s)" as s
        (Client 1) as c1
        (Client 2) as c2
        (Client 3) as c3
        (Client 4) as c4

        c1 -[hidden]right- c3
        c2 -[hidden]right- c4
        c1 -[hidden]down- c2
        c3 -[hidden]down- c4

        s -[hidden]up- c1
        s -[hidden]up- c3
        c2 -[hidden]up- s
        c4 -[hidden]up- s

        c1 <--> s
        c3 <--> s
        s <--> c2
        s <--> c4
    }

    package sd as "Simple Discovery"{
        (Context 1) as x1
        (Context 2) as x2
        (Context 3) as x3
        (Context 4) as x4

        x1 <-right-> x3
        x1 <-down-> x2
        x2 <-right-> x4
        x3 <-down-> x4

        x1 <--> x4
        x3 <--> x2
    }

    sd -[hidden]right- ds

A *server* is a context to which the *clients* (and maybe other *servers*) send their discovery information.
The role of the *server* is to re-distribute the *clients* (and *servers*) discovery information to their known *clients* and *servers*.

A *client* is a context that connects to one or more *servers* from which it receives only the discovery information they require to establish communication with matching endpoints.

.. _tutorials_deployment_ds_wan_tcp_overview:

Overview
--------

This tutorial will use ROS 2 ``demo_nodes_cpp`` ``talker`` and ``listener`` applications to establish the communication between *clients* through the *server*.
Each node would be deployed in a docker container, in different networks.

The discovery *server*, would be deployed also in its own docker container, but it will be part of two networks: the WAN and the same network as the ``talker`` node.
This setup allows the discovery *server* to perform routing tasks as a regular router does in a LAN (having a private IP which would be in the ``talker`` LAN IP, and a public IP, which would be in the WAN IP).
Additionally, the same routing element is required in the ``listener`` LAN. A *router* container is included as the intermediary between the WAN and the ``listener`` node network.

Within these defined scenario, the following diagram describes the network setup for deploying the simulation.

.. figure:: /rst/figures/tutorials/core/ds_wan_tcp/ds_wan_tcp_complex.svg
    :align: center
    :width: 70%

    Network setup example to simulate TCP communication over WAN with Discovery Server

The expected behavior is that both ``talker`` and ``listener`` are able to connect to the discovery *server*, discover each other, and send and receive (respectively) the *HelloWorld* example messages over the WAN.

.. important::

    All the communication, including EDP phase, would be performed using TCP.
    See the `Fast DDS discovery phases documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html>`_ for further information.

.. note::

    Docker performs a network configuration to isolate each of the docker networks.
    This tutorial would update some of the ``iptables`` configuration set automatically by docker in order to simulate properly the WAN scenario.


Prerequisites
-------------

First of all, make sure that Vulcanexus Humble is installed.
The docker installation is required for this tutorial (see :ref:`Docker installation <docker_installation>`).

In addition, `docker-compose <https://docs.docker.com/compose/>`_ is used to simplify the example deployment, and ``iptables`` is required to update the network configuration.
These can be installed by running:

.. code-block:: bash

    sudo apt install docker-compose iptables

.. note::

    It is highly recommended to complete the :ref:`Configuring Fast-DDS QoS via XML profiles <tutorials_xml_profiles_intro>` tutorial to learn how to configure ROS 2 via XML configuration files.

Set up the docker networks
--------------------------

Set the specific interface pools by running the following commands.

.. code-block:: bash

    docker network create --subnet=10.1.0.0/16 talker_net
    docker network create --subnet=10.2.0.0/16 listener_net
    docker network create --subnet=10.3.0.0/16 wan_net

The three networks would have been created, all of them isolated from the others.
In order to enable the communication between networks, it is mandatory to update the system ``iptables``.

.. code-block:: bash

    sudo iptables -I DOCKER-ISOLATION-STAGE-2 -i talker_net -o listener_net -j ACCEPT
    sudo iptables -I DOCKER-ISOLATION-STAGE-2 -o talker_net -i listener_net -j ACCEPT
    sudo iptables -I DOCKER-ISOLATION-STAGE-2 -i talker_net -o wan_net -j ACCEPT
    sudo iptables -I DOCKER-ISOLATION-STAGE-2 -o talker_net -i wan_net -j ACCEPT
    sudo iptables -I DOCKER-ISOLATION-STAGE-2 -i listener_net -o wan_net -j ACCEPT
    sudo iptables -I DOCKER-ISOLATION-STAGE-2 -o listener_net -i wan_net -j ACCEPT

This set of commands is enabling both *input* (``-i``) and *output* (``-o``) tables to allow both sides communication between the three networks.

.. note::

    This step is only necessary in this simulation example.
    In a real WAN deployment, this is not needed.

XML configuration files
-----------------------

It is mandatory to set the discovery *server* and the *client* nodes with the following configuration options in order to enable the TCP communication, regardless of whether the deployment is over Docker networks or WAN.

The following XML configuration describes the discovery *server* TCP configuration required.
Create a workspace to run the tutorial, and include an XML configuration file named ``server_configuration.xml`` with the below code.

Note that in the discovery configuration section, the profile is described as *server*, with a specific ``GUID prefix``, and listening locator.
This locator is configured with the IP ``10.1.1.1``, which belongs to the ``talker`` node LAN, and its locator physical port is included in the ``transport_descriptors`` as a known listening port.
This is crucial to ensure the TCP communication.

.. literalinclude:: /../code/ds_wan_tcp_tutorial/server_configuration.xml
    :language: XML

Then, also include the following XML configuration in the workspace, and name the file as ``node_configuration.xml``.
This former configuration describes the *client* configuration for the EDP phase, which involves setting the profile as *client* in the  discovery configuration section, adding the discovery *server* ``GUID prefix`` and listening locator, and the TCP transport descriptor with the *server* locator physical port which ensures the TCP communication.

.. literalinclude:: /../code/ds_wan_tcp_tutorial/node_configuration.xml
    :language: XML

Both XML configuration files will be later used in the ``docker-compose`` instructions to perform the containers deployment.

Create the Docker compose file
------------------------------

Once the XML configuration files have been included in the workspace, create a new file named ``Dockerfile`` and paste the following code.
It contains the required commands to assemble a docker image based on `ROS2 Humble <https://docs.ros.org/en/humble/index.html>`_.
That includes some dependencies, and the recently created XML configuration files.

.. literalinclude:: /../code/ds_wan_tcp_tutorial/Dockerfile
    :language: Dockerfile

Finally, the ``compose.yml`` is where all the containers and their configuration are described:

* ``ros_talker``: the container is included in the created ``talker_net``.
  The IP address has not been set manually, but the IP assigned by default would be ``10.1.0.2``.
  The environment variable ``FASTRTPS_DEFAULT_PROFILES_FILE`` is set with the ``node_configuration.xml`` and ``ROS_DISCOVERY_SERVER`` is set with the discovery *server* information ``"TCPv4:[10.1.1.1]:10111"``.
  The default gateway of this container is redirected to the discovery *server*, which would behave as a router too.

* ``ros_listener``: the container is included in the created ``listener_net``.
  The IP address has not been set manually, but the IP assigned by default would be ``10.2.0.2``.
  As the previous container, the environment variable ``FASTRTPS_DEFAULT_PROFILES_FILE`` is set with the ``node_configuration.xml`` and ``ROS_DISCOVERY_SERVER`` is set with the discovery *server* information ``"TCPv4:[10.1.1.1]:10111"``.
  The default gateway of this container is redirected to the *router* container.

* ``fast_dds_discovery_server``: the container is included in both created ``talker_net`` and ``wan_net``.
  The IP addresses has been set manually as ``10.1.1.1`` in the ``talker_net``, and ``10.3.1.1`` in the ``wan_net``.
  The default gateway of this container is redirected to the *router* container.
  The ``iptables`` has been configured to redirect any traffic from any network and interface.

* ``router``: the container is included in both ``listener_net`` and ``wan_net`` networks.
  The IP addresses has been manually set as ``10.2.1.1`` in the ``listener_net``, and ``10.3.2.1`` in the ``wan_net``.
  The default gateway of this container is redirected to the discovery *server* container.
  The ``iptables`` has been configured to redirect traffic from any network and interface.


Please, include the following ``compose.yml`` file in the workspace.

.. literalinclude:: /../code/ds_wan_tcp_tutorial/compose.yml
    :language: yaml

Run the example
---------------

Make sure that the workspace has been set with all the previous files, and the docker networks and ``iptables`` have been set too.

.. code-block:: bash

    ├ <workspace>
    ·  ├ compose.yml
    ·  ├ Dockerfile
    ·  ├ node_configuration.xml
    ·  └ server_configuration.xml

As explained in the :ref:`tutorials_deployment_ds_wan_tcp_overview`, the expected behavior is that both ``talker`` and ``listener`` are able to connect to the discovery *server*, discover each other, and send and receive (respectively) the *HelloWorld* example messages over the WAN simulation using TCP.

Run the example:

.. code-block:: bash

    cd <workspace>
    docker compose -f compose.yml up --build

.. note::

    The requirements to achieve TCP communication over WAN with Discovery Server as EDP in a real deployment are simply launching the three elements of the communication (``talker``, ``listener`` and discovery *server*) with their corresponding XML configurations applied.
    The remaining ``iptables`` and docker configuration and deployment have been defined to simulate WAN scenario locally.

Clean workspace
---------------

Stop the example by pressing ``Ctrl + C``, and stop the containers by running:

.. code-block:: bash

    docker compose -f compose.yml down

The docker networks, containers and images can be removed using the docker ``prune`` argument, but using the ``rm`` argument plus the identifiers would remove only the ones created for this tutorial:

.. code-block:: bash

    docker network rm listener_net talker_net wan_net
    docker container rm <workspace_name>_fast_dds_discovery_server_1 <workspace_name>_router_1 <workspace_name>_ros_listener_1 <workspace_name>_ros_talker_1
    docker image rm <workspace_name>_fast_dds_discovery_server:latest <workspace_name>_router:latest <workspace_name>_ros_listener:latest <workspace_name>_ros_talker:latest

.. note::

    The ``iptables`` configuration would be removed automatically each time the system gets rebooted.
