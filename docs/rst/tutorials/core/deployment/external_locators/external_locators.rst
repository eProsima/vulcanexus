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
   :scale: 75%

This example case consists on two hosts, the first one maintaining two docker containers running the well-known ROS 2 talker-listener example nodes, and a second one with a ROS 2 listener inside another container.
In Vulcanexus, the communication among the different nodes can be achieved by means of the use of *External Locators* defined in  `WireProtocolConfigQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/core/policy/wireprotocolconfigqos.html>`_.


Prerequisites
--------------

For accomplishing this tutorial, two available hosts with Docker and a Vulcanexus image are required.
Please refer to the installation steps detailed in :ref:`docker_installation`.

/*Extend smth*/

Understanding External Locators
--------------------------------

External locators should be seen as an extra feature over the default communication mechanisms (unicast, multicast).
In that sense, it is possible not just to discover peers via multicast within the same LAN, but also peers in remote sub-networks over a shared external network at some higher level (WAN, WLAN, etc.).
It is for this reason that External Locators relies on the the concept of levels of externality, which effectively map to the different nesting levels of the network setup, resulting in a sequence of external IP addresses exposed by the different nested sub-network interfaces.

As described in the next section, it is possible to configure external IP locators with an associated externality index, cost, and sub-network mask within the DomainParticipant configuration.


Enable External Locators via XML configuration files
-----------------------------------------------------

In order to define the desired External Locators configuration, an XML profile needs to be provided (see `Fast DDS XML profiles <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_).
External Locators announcement for the different `Communication establishment Phases: <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html>`_ Participant Discovery Phase (metatraffic, initial peers tags) and Endpoint Discovery Phase (user traffic tag), should be defined.

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

                .. code-block:: xml

                    <?xml version="1.0" encoding="UTF-8" ?>
                    <dds>
                        <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
                            <participant profile_name="container0" is_default_profile="true">
                                <rtps>
                                    <!-- External locators for user traffic -->
                                    <default_external_unicast_locators>
                                        <udpv4 externality="1" cost="0" mask="24">
                                            <!-- Host 1 external IP -->
                                            <address>192.168.1.40</address>
                                            <port>11201</port>
                                        </udpv4>
                                    </default_external_unicast_locators>
                                    <builtin>
                                        <!-- External locators for discovery traffic -->
                                        <metatraffic_external_unicast_locators>
                                            <udpv4 externality="1" cost="0" mask="24">
                                                <!-- Host 1 external IP -->
                                                <address>192.168.1.40</address>
                                                <port>11200</port>
                                            </udpv4>
                                        </metatraffic_external_unicast_locators>
                                        <!-- Locators of remote participants (discovery traffic)-->
                                        <initialPeersList>
                                            <!--container 1 peer-->
                                            <locator>
                                                <udpv4>
                                                    <!-- Host 2 external IP -->
                                                    <address>192.168.1.56</address>
                                                    <port>11200</port>
                                                </udpv4>
                                            </locator>
                                            <!-- local network multicast. Discover
                                            other participants in the same LAN,
                                            using External Locators, or not -->
                                            <locator>
                                                <udpv4>
                                                    <address>239.255.0.1</address>
                                                    <port>7400</port>
                                                </udpv4>
                                            </locator>
                                        </initialPeersList>
                                    </builtin>
                                </rtps>
                            </participant>
                        </profiles>
                    </dds>

    .. tab:: HOST 2

        .. tabs::

            .. tab:: CONTAINER 1

                .. code-block:: xml

                    <?xml version="1.0" encoding="UTF-8" ?>
                    <dds>
                        <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
                            <participant profile_name="container0" is_default_profile="true">
                                <rtps>
                                    <!-- External locators for user traffic -->
                                    <default_external_unicast_locators>
                                        <udpv4 externality="1" cost="0" mask="24">
                                            <!-- Host 2 external IP -->
                                            <address>192.168.1.56</address>
                                            <port>11201</port>
                                        </udpv4>
                                    </default_external_unicast_locators>
                                    <builtin>
                                        <!-- External locators for discovery traffic -->
                                        <metatraffic_external_unicast_locators>
                                            <udpv4 externality="1" cost="0" mask="24">
                                                <!-- Host 2 external IP -->
                                                <address>192.168.1.56</address>
                                                <port>11200</port>
                                            </udpv4>
                                        </metatraffic_external_unicast_locators>
                                        <!-- Locators of remote participants (discovery traffic)-->
                                        <initialPeersList>
                                            <!-- Container 1 peer-->
                                            <locator>
                                                <udpv4>
                                                    <!-- Host 1 external IP -->
                                                    <address>192.168.1.40</address>
                                                    <port>11200</port>
                                                </udpv4>
                                            </locator>
                                            <!-- local network multicast. Discover
                                            other participants in the same LAN,
                                            using External Locators, or not -->
                                            <locator>
                                                <udpv4>
                                                    <address>239.255.0.1</address>
                                                    <port>7400</port>
                                                </udpv4>
                                            </locator>
                                        </initialPeersList>
                                    </builtin>
                                </rtps>
                            </participant>
                        </profiles>
                    </dds>


Run the example
----------------

This section provides with step-by-step instructions for setting up the example scenario described in the previous Background section.
On both hosts, open a shell and run:

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: TERMINAL 1

                .. code-block:: bash

                    xhost local:root
                    docker run --rm -it --privileged `# Cleanup, interactive terminal` \
                        -p 11200-11201:7412-7413/udp `# Expose default internal ports to host` \
                        -e DISPLAY=$DISPLAY `# Set DISPLAY environment variable` \
                        -v /tmp/.X11-unix:/tmp/.X11-unix `# Bind to tmp volume` \
                        ubuntu-vulcanexus:humble-desktop `# Image name`

            .. tab:: TERMINAL 2

                .. code-block:: bash

                    docker run --rm -it --privileged `# Cleanup, interactive terminal` \
                        -e DISPLAY=$DISPLAY `# Set DISPLAY environment variable` \
                        -v /tmp/.X11-unix:/tmp/.X11-unix `# Bind to tmp volume` \
                        ubuntu-vulcanexus:humble-desktop `# Image name`


    .. tab:: HOST 2

        .. tabs::

            .. tab:: TERMINAL 1

                .. code-block:: bash

                    xhost local:root
                    docker run --rm -it --privileged `# Cleanup, interactive terminal` \
                        -p 11200-11201:7412-7413/udp `# Expose default internal ports to host` \
                        -e DISPLAY=$DISPLAY `# Set DISPLAY environment variable` \
                        -v /tmp/.X11-unix:/tmp/.X11-unix `# Bind to tmp volume` \
                        ubuntu-vulcanexus:humble-desktop `# Image name`


.. note::

    It is important to specify the port mapping argument so as to expose docker internal ports to the host. See `Docker Networking <https://docs.docker.com/config/containers/container-networking/>`_ for further information.

The next step is the creation of the XML profiles.
Inside each one of the three containers, create a Profiles.xml file and paste the contents of the corresponding XML profile configuration, according to the previous section.

Finally, export the environment variable pointing to the Profiles file, source Vulcanexus environment and run the ros2 example nodes.

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: CONTAINER 1

                .. code-block:: bash

                    source vulcanexus_entrypoint.sh
                    export FASTRTPS_DEFAULT_PROFILES_FILE=/Profiles.xml #Or the Profiles.xml file location
                    ros2 run demo_nodes_cpp talker

            .. tab:: CONTAINER 2

                .. code-block:: bash

                    source vulcanexus_entrypoint.sh
                    ros2 run demo_nodes_cpp listener


    .. tab:: HOST 2

        .. tabs::

            .. tab:: CONTAINER 1

                    .. code-block:: bash

                        source vulcanexus_entrypoint.sh
                        export FASTRTPS_DEFAULT_PROFILES_FILE=/Profiles.xml #Or the Profiles.xml file location
                        ros2 run demo_nodes_cpp listener


At this point, nodes should be communicating with each other as expected.
A message `Hellow World: [count]` should start printing in the talker's container terminal while both listeners keep receiving it, in their respective container consoles, as follows:

.. raw:: html

    <video width=100% height=auto autoplay loop controls>
        <source src="../../../../../_static/resources/tutorials/core/deployment/external_locators/external_locators.mp4">
        Your browser does not support the video tag.
    </video>
