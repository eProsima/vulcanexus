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

The `Discovery Server <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html>`_ mechanism is based on a client-server discovery paradigm, i.e. the metatraffic (message exchange among domain participants to identify each other) is managed by one or several server DomainParticipants (left figure), as opposed to simple discovery (right figure), where metatraffic is exchanged using a message broadcast mechanism like an IP multicast protocol.
A `Discovery-Server tool <https://eprosima-discovery-server.readthedocs.io/en/latest/index.html>`_ is available to ease Discovery Server setup and testing.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

.. figure:: /rst/figures/intro/discovery-server.svg
    :align: center
    :width: 50%

    Comparison of Discovery Server and Simple discovery mechanisms

A *server* is a participant to which the *clients* (and maybe other *servers*) send their discovery information.
The role of the *server* is to re-distribute the *clients* (and *servers*) discovery information to their known *clients* and *servers*.

A *client* is a participant that connects to one or more *servers* from which it receives only the discovery information they require to establish communication with matching endpoints.

Overview
--------

This tutorial will use ROS 2 ``demo_nodes_cpp`` ``talker`` and ``listener`` applications to establish the communication between *clients* through the *servers*.
Two different discovery server networks would be set.
Each of them would have at least one discovery server, and one discovery client, which would be running a ``talker`` node.
The aim of the tutorial is to add a discovery client running a ``listener`` node that receives the ``talker`` node publications coming from the same discovery server network, and on run-time, update the discovery server list to make it receive also the ``talker`` node publications coming from the other discovery server network.

.. uml::
    :align: center

    hide empty members

    package "Discovery Server Network 0" as de0{
        (Server 0) as s0
        (Client pub 0) as p0
        (Client sub) as s
        s <-[dashed]right- p0
        s0 <-down- s
        s0 <-down- p0
        p0 -[hidden]left- s0
    }

    package "Discovery Server Network 1" as de1{
        (Server 1) as s1
        (Client pub 1) as p1
        p1 -up-> s1
    }

    de1 -[hidden]right- de0
    s1 -[hidden]right- s0
    p1 -[hidden]right- p0
    p1 -[hidden]-> s0
    p0 -[dotted]-> s1
    s -[dotted]-> s1

To do so, the isolated ``talker`` client node discovery servers list would be updated on run-time, so the ``listener`` would be able to receive the new publications.

Prerequisites
-------------

First of all, make sure that Vulcanexus Humble is installed.
The docker installation is required for this tutorial (see :ref:`Docker installation <docker_installation>`).

.. note::

    It is highly recommended to complete the :ref:`Configuring Fast-DDS QoS via XML profiles <tutorials_xml_profiles_intro>` tutorial to learn how to configure ROS 2 via XML configuration files.


Set up the discovery server networks
------------------------------------

Open five terminals, and run the Vulcanexus Humble image in each one with the following commands:

.. tabs::

    .. tab:: Discovery Server Network 0

        .. tabs::

            .. tab:: Server_0

                .. code-block:: bash

                    docker run -it --rm --name server_0 ubuntu-vulcanexus:humble-desktop

            .. tab:: Client_talker_0

                .. code-block:: bash

                    docker run -it --rm --name client_pub_0 ubuntu-vulcanexus:humble-desktop

            .. tab:: Client_listener

                .. code-block:: bash

                    docker run -it --rm --name client_sub ubuntu-vulcanexus:humble-desktop

    .. tab:: Discovery Server Network 1

        .. tabs::

            .. tab:: Server_1

                .. code-block:: bash

                    docker run -it --rm --name server_1 ubuntu-vulcanexus:humble-desktop

            .. tab:: Client_talker_1

                .. code-block:: bash

                    docker run -it --rm --name client_pub_1 ubuntu-vulcanexus:humble-desktop



Configure the discovery entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The *servers* configuration is really simple, the *server* ``discovery id`` would be set as ``0`` in the first discovery server network, and ``1`` in the other one.

.. tabs::

    .. tab:: Server 0

        .. code-block:: bash

            fastdds discovery --server-id 0

    .. tab:: Server 1

        .. code-block:: bash

            fastdds discovery --server-id 1

The outputs should look similar to the followings:

.. tabs::

    .. tab:: Server 0

        .. code-block:: bash

            ### Server is running ###
            Participant Type:   SERVER
            Server ID:          0
            Server GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
            Server Addresses:   UDPv4:[0.0.0.0]:11811

    .. tab:: Server 1

        .. code-block:: bash

            ### Server is running ###
            Participant Type:   SERVER
            Server ID:          1
            Server GUID prefix: 44.53.01.5f.45.50.52.4f.53.49.4d.41
            Server Addresses:   UDPv4:[0.0.0.0]:11811

The output displays the ``server ID`` set, followed by the server GUID prefix.
Server address ``0.0.0.0`` tells *Fast DDS* to listen on all available interfaces.
Finally, the ``11811`` default port would be necessary for further configuration.

In order to set-up the *clients* to discover each other through the *server*, it is needed to set the *server* data (IP address, port and prefix) in all *clients*.
The port and prefix have just been obtained in the previous step, and each specific *server* IP address can be obtained introducing the *servers* container name in the following command:

.. tabs::

    .. tab:: Discovery Server Network 0

        .. code-block:: bash

            docker inspect --format '{{ .NetworkSettings.IPAddress }}' server_0

    .. tab:: Discovery Server Network 1

        .. code-block:: bash

            docker inspect --format '{{ .NetworkSettings.IPAddress }}' server_1

.. note::

    For this example, these are the used IP addresses for each entity.

    +----------------+------------+
    | CONTAINER NAME | IP ADDRESS |
    +================+============+
    | client_sub     | 172.17.0.2 |
    +----------------+------------+
    | client_pub_0   | 172.17.0.3 |
    +----------------+------------+
    | client_pub_1   | 172.17.0.4 |
    +----------------+------------+
    | server_0       | 172.17.0.5 |
    +----------------+------------+
    | server_1       | 172.17.0.6 |
    +----------------+------------+

Create a XML configuration file in each *client* container, and complete it with the following XML profile, including the *server* data obtained (IP address, port and GUID prefix).

.. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/client_configuration.xml
    :language: xml

The nodes would load the created profile automatically by setting the XML configuration file name as ``DEFAULT_FASTRTPS_PROFILES.xml``.

``FASTDDS_ENVIRONMENT_FILE``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Finally, each *client* needs to start a discovery client.
To do so, and this is the key of the tutorial, the environment variable ``FASTDDS_ENVIRONMENT_FILE`` must be set.

Setting this environment variable to an existing ``json`` file allows to load the environment variables from a file instead of from the environment.
This allows to change the value of some environment variables at run time with just modifying and saving the changes to the file.

Create a json file for each *client* and introduce the *server* IP address and port.
Make sure the talker in the discovery server network 1 only contains its discovery server data:

.. tabs::

    .. tab:: Discovery Server Network 0

        .. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/environment_file_complete.json
            :language: xml

    .. tab:: Discovery Server Network 1

        .. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/environment_file.json
            :language: xml

The environment value can be either an absolute or relative path.
Set up the environment file:

.. code-block:: bash

    export FASTDDS_ENVIRONMENT_FILE="environment_file.json"


Run the example
---------------

After all the configuration has been set, run the discovery servers, and the talker and listener *clients*:

.. tabs::

    .. tab:: Discovery Server Network 0

        .. tabs::

            .. tab:: Server_0

                .. code-block:: bash

                    fastdds discovery --server-id 0

            .. tab:: Client_talker_0

                .. code-block:: bash

                    ros2 run demo_nodes_cpp talker

            .. tab:: Client_listener

                .. code-block:: bash

                    ros2 run demo_nodes_cpp listener

    .. tab:: Discovery Server Network 1

        .. tabs::

            .. tab:: Server_1

                .. code-block:: bash

                    fastdds discovery --server-id 1

            .. tab:: Client_talker_1

                .. code-block:: bash

                    ros2 run demo_nodes_cpp talker

The expect output is that the listener would only receive the publications from the talker in the same discovery server network.
Now, let's add the other *server* as a discovery server in the ``client_pub_1`` talker on run-time to let the listener receive its publications.

Discovery Server on run-time
----------------------------

During execution, modify the listener ``json`` file to include the other *server* data, as follows:

.. literalinclude:: /resources/tutorials/core/deployment/ds_locators_runtime/environment_file_complete.json
    :language: xml

After saving the file, the listener would discover the remain talker through the discovery server just set.

.. uml::
    :align: center

    hide empty members

    package "Discovery Server Network 0" as de0{
        (Server 0) as s0
        (Client pub 0) as p0
        (Client sub) as s
        s <-[dashed]right- p0
        s0 <-down- s
        s0 <-down- p0
        p0 -[hidden]left- s0
    }

    package "Discovery Server Network 1" as de1{
        (Server 1) as s1
        (Client pub 1) as p1
        p1 -up-> s1
    }

    de1 -[hidden]right- de0
    s1 -[hidden]right- s0
    p1 -[hidden]right- p0
    p0 -[dotted]-> s1
    s -[dotted]-> s1
    p1 -[dotted]-> s0
    p1 -[dashed]-> s


.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/deployment/ds_locators_runtime/ds_runtime_video.webm">
        Your browser does not support the video tag.
    </video>

