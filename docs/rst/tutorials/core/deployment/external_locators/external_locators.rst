.. _tutorials_deployment_external_locators_external_locators:

Connecting Nodes over an External Network
==========================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Many robotic applications involve complex network topologies (mesh, nested, etc.) in which nodes, processes or entities from one host may need to interact with both, nodes from outside and inside its network.

Consider the following nested network scenario: one talker node inside one host's private LAN sub-network needs to communicate with two listener nodes; the first one belonging to the same LAN as the talker, and the second one deployed within an external host with its own private sub-network.
Also consider both hosts externally connected to the same network.
The following diagram depicts the aforementioned scheme.

.. figure:: /rst/figures/tutorials/core/external_locators/network_setup.svg
   :align: center
   :scale: 100%

This example case consists on two hosts, the first one maintaining two docker containers running the well-known ROS 2 talker-listener example nodes, and a second one with a ROS 2 listener inside another container.
In Vulcanexus, the communication among the different nodes can be achieved by means of Fast DDS' *External Locators* feature defined in  `WireProtocolConfigQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/core/policy/wireprotocolconfigqos.html>`_.


Prerequisites
--------------

For accomplishing this tutorial, two available hosts with Docker and a Vulcanexus image are required.
Please refer to the installation steps detailed in :ref:`docker_installation`.
In addition, host's ports 11200, 11201 need to be available (these ports have been selected as an example for this tutorial, but it is up to the user to make a different choice).


*Fast DDS* domain participants will require to announce themselves into their host's external network(s).
Net-tools, Network Manager or similar packages need to be installed in the system in order to retrieve the corresponding IPs addresses.

Understanding External Locators
--------------------------------

External locators should be seen as an extra feature over the default communication mechanisms (unicast, multicast).
In that sense, it is possible not just to discover peers via multicast within the same LAN, but also peers in remote sub-networks over a shared external network at some higher level (WAN, WLAN, etc.).
It is for this reason that External Locators relies on the the concept of levels of externality, which effectively map to the different nesting levels of the network setup, resulting in a sequence of external IP addresses exposed by the different nested sub-network interfaces.

As described in the next section, it is possible to configure external IP locators with an associated externality index, cost, and sub-network mask within the DomainParticipant configuration.


Enable External Locators via XML configuration files
-----------------------------------------------------

In order to define the desired External Locators configuration, an XML profile needs to be provided (see `Fast DDS XML profiles <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_).
External Locators announcement for the different `Communication phases: <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html>`_ participant and endpoint discovery phase (metatraffic, initial peers tags), and user data communication phase (user traffic tag), should be defined.

Following with the example above, two XML configuration profiles should be provided.
The power of External Locators is the ability to connect to nodes within external networks while still being discovered in the local network.
Due to this reasoning, multicast discovery will be used for the second container on the first host (consequently, there is no need to provide any additional XML profile).
The two resultant XML configurations are detailed below:

.. note::

    Note that the container network itself does not create another level of externality in this case, as it is bridged with the host network.

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: CONTAINER 1

                .. literalinclude:: /resources/tutorials/core/deployment/external_locators/host1_container1.xml
                    :language: xml



    .. tab:: HOST 2

        .. tabs::

            .. tab:: CONTAINER 1

                .. literalinclude:: /resources/tutorials/core/deployment/external_locators/host2_container1.xml
                    :language: xml

Run the example
----------------

This section provides with step-by-step instructions for setting up the example scenario described in previous sections.
On both hosts, open a shell and run:

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: TERMINAL 1

                .. code-block:: bash

                    docker run --rm -it `# Cleanup, interactive terminal` \
                        -p 11200-11201:7412-7413/udp `# Expose default internal ports to host` \
                        ubuntu-vulcanexus:humble-desktop `# Image name`

            .. tab:: TERMINAL 2

                .. code-block:: bash

                    docker run --rm -it `# Cleanup, interactive terminal` \
                        ubuntu-vulcanexus:humble-desktop `# Image name`


    .. tab:: HOST 2

        .. tabs::

            .. tab:: TERMINAL 1

                .. code-block:: bash

                    docker run --rm -it `# Cleanup, interactive terminal` \
                        -p 11200-11201:7412-7413/udp `# Expose default internal ports to host` \
                        ubuntu-vulcanexus:humble-desktop `# Image name`


.. note::

    It is important to specify the port mapping argument so as to expose docker internal ports to the host. See `Docker Networking <https://docs.docker.com/config/containers/container-networking/>`_ for further information.

The next step is the creation of the XML profiles.
Inside each one of the three containers, create a Profiles.xml file and paste the contents of the corresponding XML profile configuration, according to the previous section.

Finally, export the environment variable pointing to the Profiles.xml file, source Vulcanexus environment and run the ros2 example nodes.

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: CONTAINER 1

                .. code-block:: bash

                    source /vulcanexus_entrypoint.sh
                    export FASTRTPS_DEFAULT_PROFILES_FILE=/Profiles.xml # Or the Profiles.xml file location
                    ros2 run demo_nodes_cpp talker

            .. tab:: CONTAINER 2

                .. code-block:: bash

                    source /vulcanexus_entrypoint.sh
                    ros2 run demo_nodes_cpp listener


    .. tab:: HOST 2

        .. tabs::

            .. tab:: CONTAINER 1

                    .. code-block:: bash

                        source /vulcanexus_entrypoint.sh
                        export FASTRTPS_DEFAULT_PROFILES_FILE=/Profiles.xml # Or the Profiles.xml file location
                        ros2 run demo_nodes_cpp listener


At this point, nodes should be communicating with each other as expected.
A message `Hello World: [count]` should start printing in the talker's container terminal while both listeners keep receiving it, in their respective container consoles, as follows:

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/deployment/external_locators/external_locators.mp4">
        Your browser does not support the video tag.
    </video>
