.. _tutorials_deployment_ds_locators_runtime:

Modify Discovery Server locators on run-time
============================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

.. _tutorials_deployment_ds_locators_runtime_background:

Background
----------

The :ref:`Discovery Server <vulcanexus_discovery_server>` is a Fast DDS enabled feature that procures an alternative discovery mechanism to the default ROS 2 discovery mechanism, `Simple Discovery Protocol (SDP) <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#simple-disc-settings>`_, which is served by the DDS implementations according to the DDS specification.
Whereas SDP (right figure) provides automatic out-of-the-box discovery by leveraging multicast, ROS 2 Discovery Server (left figure) provides a centralized hub for managing discovery which drastically reduces network bandwidth utilization when compared to SDP, since the nodes, publishers, and subscribers, only discovered those remote ROS 2 entities with which they need to communication (as opposed to the SDP model where everyone knows about each other).

.. contents::
    :local:
    :backlinks: none
    :depth: 1

.. figure:: /rst/figures/intro/discovery-server.svg
    :align: center
    :width: 70%

    Comparison of Discovery Server and Simple Discovery Protocol mechanisms

A *server* is a context to which the *clients* (and maybe other *servers*) send their discovery information.
The role of the *server* is to re-distribute the *clients* (and *servers*) discovery information to their known *clients* and *servers*.

A *client* is a context that connects to one or more *servers* from which it receives only the discovery information they require to establish communication with matching endpoints.

Overview
--------

This tutorial will use ROS 2 ``demo_nodes_cpp`` ``talker`` and ``listener`` applications to establish the communication between *clients* through the *servers*.
Two different discovery server networks would be set.
Each of them would have at least one discovery *server*, and one *client* context, which would be running a ``talker`` node.
The aim of the tutorial is to add a *client* context running a ``listener`` node that receives the ``talker`` node publications coming from the same discovery server network, and on run-time, update the discovery server list to make allow the ``listener`` node to receive also the publications coming from the other discovery server network ``talker`` node.

.. uml::
    :align: center

    hide empty members

    package de0 as "Discovery Server Network A"{
        cloud "Server A" as s0
        (Client talker A) as p0
        (Client listener) as s
        s <-[dashed]right- p0
        s0 <-down- s
        s0 <-down- p0
        p0 -[hidden]left- s0
    }

    package de1 as "Discovery Server Network B"{
        cloud "Server B" as s1
        (Client talker B) as p1
        p1 -up-> s1
    }

    de1 -[hidden]right- de0
    s1 -[hidden]right- s0
    p1 -[hidden]right- p0
    p1 -[hidden]-> s0
    s -[hidden]-> s1

To do so, the ``Client listener`` node discovery servers list would be updated on run-time to include the ``Server B``, so the ``Client listener`` node would be able to receive the new publications from the ``Client talker B`` node.

Prerequisites
-------------

First of all, make sure that Vulcanexus Humble is installed.
The docker installation is required for this tutorial (see :ref:`Docker installation <docker_installation>`).

.. note::

    It is highly recommended to complete the :ref:`Configuring Fast-DDS QoS via XML profiles <tutorials_xml_profiles_intro>` tutorial to learn how to configure ROS 2 via XML configuration files.


Set up the discovery server networks
------------------------------------

Set a specific interface pool for this implementation by running the following command.

.. code-block:: bash

    docker network create --subnet=113.11.3.0/16 vulcanexus_tutorial

Then, open five terminals, and run the Vulcanexus Humble image in each one with the following commands with the specific IP addresses:

.. tabs::

    .. tab:: Discovery Server Network A

        .. tabs::

            .. tab:: Server A

                .. code-block:: bash

                    docker run -it --rm --name server_A --net vulcanexus_tutorial --ip 113.11.3.0 ubuntu-vulcanexus:humble-desktop

            .. tab:: Client talker A

                .. code-block:: bash

                    docker run -it --rm --name talker_A --net vulcanexus_tutorial --ip 113.11.3.2 ubuntu-vulcanexus:humble-desktop

            .. tab:: Client listener

                .. code-block:: bash

                    docker run -it --rm --name listener --net vulcanexus_tutorial --ip 113.11.3.3 ubuntu-vulcanexus:humble-desktop

    .. tab:: Discovery Server Network B

        .. tabs::

            .. tab:: Server B

                .. code-block:: bash

                    docker run -it --rm --name server_B --net vulcanexus_tutorial --ip 113.11.3.1 ubuntu-vulcanexus:humble-desktop

            .. tab:: Client talker B

                .. code-block:: bash

                    docker run -it --rm --name talker_B --net vulcanexus_tutorial --ip 113.11.3.4 ubuntu-vulcanexus:humble-desktop



Configure the discovery entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The *servers* configuration is really simple: by using the `Fast DDS Discovery CLI <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`_, the *server* ``discovery id`` would be set as ``0`` in the ``Discovery Server Network A``, and ``1`` in the ``Discovery Server Network B``.

.. tabs::

    .. tab:: Discovery Server Network A

        .. tabs::

            .. tab:: Server A

                .. code-block:: bash

                    fastdds discovery --server-id 0

    .. tab:: Discovery Server Network B

        .. tabs::

            .. tab:: Server B

                .. code-block:: bash

                    fastdds discovery --server-id 1

The outputs should look similar to the followings:

.. tabs::

    .. tab:: Discovery Server Network A

        .. tabs::

            .. tab:: Server A

                .. code-block:: bash

                    ### Server is running ###
                    Participant Type:   SERVER
                    Server ID:          0
                    Server GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
                    Server Addresses:   UDPv4:[0.0.0.0]:11811

    .. tab:: Discovery Server Network B

        .. tabs::

            .. tab:: Server B

                .. code-block:: bash

                    ### Server is running ###
                    Participant Type:   SERVER
                    Server ID:          1
                    Server GUID prefix: 44.53.01.5f.45.50.52.4f.53.49.4d.41
                    Server Addresses:   UDPv4:[0.0.0.0]:11811

The output displays the ``server ID`` set, followed by the server GUID prefix.
Server address ``0.0.0.0`` tells *Fast DDS* to listen on all available interfaces.
Finally, the ``11811`` default port would be necessary for further configuration.

``FASTDDS_ENVIRONMENT_FILE``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each *client* needs to know to which *server* it should connect.
In order to update that information on run-time, the environment variable ``FASTDDS_ENVIRONMENT_FILE`` must be set to an existing ``json`` file (see `Fast DDS Environment variables <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#fastdds-environment-file>`_).
In that way, the environment variable that sets the *server* information would be loaded from a file instead of from the environment.
This allows to change the discovery servers information by simply modifying the ``json`` file.

Create a ``discovery_servers.json`` ``json`` file for each *client* and introduce the *server* IP address and port.

.. note::
   As long as the default port has been used in the tutorial, it could be omitted in the ``json`` configuration.


.. tabs::

    .. tab:: Discovery Server Network A

        .. tabs::

            .. tab:: Client talker A

                .. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/discovery_servers_networkA.json
                    :language: xml

            .. tab:: Client listener

                .. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/discovery_servers_networkA.json
                    :language: xml

    .. tab:: Discovery Server Network B

        .. tabs::

            .. tab:: Client talker B

                .. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/discovery_servers_networkB.json
                    :language: xml

The environment value can be either an absolute or relative path.
Set up the environment file:

.. code-block:: bash

    export FASTDDS_ENVIRONMENT_FILE="discovery_servers.json"


Run the example
---------------

After all the configuration has been set, run the *discovery servers*, and the ``talker`` and ``listener`` *client* nodes:

.. tabs::

    .. tab:: Discovery Server Network A

        .. tabs::

            .. tab:: Client talker A

                .. code-block:: bash

                    export FASTDDS_ENVIRONMENT_FILE="discovery_servers.json"
                    ros2 run demo_nodes_cpp talker

            .. tab:: Client listener

                .. code-block:: bash

                    export FASTDDS_ENVIRONMENT_FILE="discovery_servers.json"
                    ros2 run demo_nodes_cpp listener

    .. tab:: Discovery Server Network B

        .. tabs::

            .. tab:: Client talker B

                .. code-block:: bash

                    export FASTDDS_ENVIRONMENT_FILE="discovery_servers.json"
                    ros2 run demo_nodes_cpp talker

The expected output is that the ``Client listener`` node would only receive the publications from the ``Client talker A`` node.
Now, let's add the ``Server B`` as a discovery server in the ``Client listener`` on run-time to receive the ``Client talker B`` publications.

Discovery Server on run-time
----------------------------

During execution, modify the ``Client listener`` node ``json`` file to include the ``Server B`` data, as follows:

.. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/discovery_servers_both_networks.json
    :language: xml

After saving the file, the ``Client listener`` would discover the ``Client talker B`` through the discovery server just set.
Then, the ``Client listener`` would start receiving ``Client talker B`` publications.

.. uml::
    :align: center

    hide empty members

    package de0 as "Discovery Server Network A"{
        cloud "Server A" as s0
        (Client talker A) as p0
        (Client listener) as s
        s <-[dashed]right- p0
        s0 <-down- s
        s0 <-down- p0
        p0 -[hidden]left- s0
    }

    package de1 as "Discovery Server Network B"{
        cloud "Server B" as s1
        (Client talker B) as p1
        p1 -up-> s1
    }

    de1 -[hidden]right- de0
    s1 -[hidden]right- s0
    p1 -[hidden]right- p0
    p1 -[hidden]-> s0
    s -[dotted]-> s1
    s <-[dashed]- p1


.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/deployment/ds_locators_runtime/ds_runtime_video.webm">
        Your browser does not support the video tag.
    </video>

