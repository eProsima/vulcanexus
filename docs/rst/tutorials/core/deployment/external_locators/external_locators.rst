.. _tutorials_deployment_external_locators_external_locators:

Connecting Nodes over an External Network
==========================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Many robotic applications involve complex network topologies (mesh, nested,...) in which nodes, processes or entities from one peer may need to interact with both, nodes from outside and inside its network.
This feature can be enabled in Vulcanexus by means of the use of *external locators* defined in  `WireProtocolConfigQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/core/policy/wireprotocolconfigqos.html>`_.

Consider the following mesh-like network scenario: one talker node inside one host private LAN sub-network needs to communicate with two listener nodes, the first one belongs to the same LAN as the first, whereas the second one is deployed inside an external host with its own private sub-network.
Both hosts are externally connected to the same network. The following diagram depicts the aforementioned scheme.

.. figure:: /rst/figures/tutorials/core/external_locators/network_setup.png
   :align: center
   :scale: 75%

This example case consists on two hosts, the first one maintaining two docker containers running the well-known ROS 2 talker-listener example nodes, and a second one with a ROS 2 listener inside another container.


Prerequisites
--------------

For accomplishing this tutorial, two available hosts with Docker and a vulcanexus image are required.
Please refer to the installation steps detailed in :ref:`docker_installation`.

Understanding External Locators
--------------------------------

External locators should be seen as an extra communication feature over multicast.
In that sense, it is possible not just to discover peers via multicast within the same LAN, but also peers in remote sub-networks over a shared external network at some higher level (WAN, WLAN,...).
It is for that reason that external locators relies on the the concept of levels of externality, which ubiquitously maps to the different nesting levels of the network setup.
That is the sequence of external IP addresses exposed by the different nested sub-network interfaces.

As described in the next section, user will be able to configure external IP locators with an associated externality index, cost, and sub-network mask within the DomainParticipant configuration.


Profiling XML with External Locators
-------------------------------------

In order to define the desired external locators configuration, a XML profile needs to be provided (see `Fast DDS XML profiles <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_).
External locators should be defined for the different `Communication stablishment Phases <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html>`_.
The following XML tags should be specified in order to correctly configure external locators:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <dds>
        <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
            <participant profile_name="example">
                <rtps>
                    <!-- External locators <ip/port> for user traffic -->
                    <default_external_unicast_locators>
                        <!-- List of locators -->
                    </default_external_unicast_locators>

                    <builtin>
                        <!-- External locators <ip/port> for discovery traffic -->
                        <metatraffic_external_unicast_locators>
                            <!-- List of locators -->
                        </metatraffic_external_unicast_locators>

                        <!-- Locators of remote participants (discovery traffic)-->
                        <initialPeersList>
                            <!-- list of peers locators-->
                        </initialPeersList>
                    </builtin>
                </rtps>
            </participant>
        </profiles>
    </dds>

Following with the above described example, three XML configuration profiles should be provided, which are detailed below:

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: CONTAINER 1

                .. code-block:: xml

                    <?xml version="1.0" encoding="UTF-8" ?>
                    <dds>
                        <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
                        <transport_descriptors>
                                <transport_descriptor>
                                    <transport_id>MyTransport</transport_id>
                                    <type>UDPv4</type>
                                </transport_descriptor>
                            </transport_descriptors>
                            <participant profile_name="container0" is_default_profile="true">
                                <rtps>
                                    <useBuiltinTransports>false</useBuiltinTransports>
                                    <!-- Link the Transport Layer to the Participant -->
                                    <userTransports>
                                        <transport_id>MyTransport</transport_id>
                                    </userTransports>
                                    <ignore_non_matching_locators>true</ignore_non_matching_locators>
                                    <!-- External locators for user traffic -->
                                    <default_external_unicast_locators>
                                        <udpv4 externality="1" cost="0" mask="24">
                                            <address>192.168.1.11</address>
                                            <port>11201</port>
                                        </udpv4>
                                    </default_external_unicast_locators>

                                    <builtin>
                                        <!-- External locators for discovery traffic -->
                                        <metatraffic_external_unicast_locators>
                                            <udpv4 externality="1" cost="0" mask="24">
                                                <address>192.168.1.11</address>
                                                <port>11200</port>
                                            </udpv4>
                                        </metatraffic_external_unicast_locators>
                                        <!-- Locators of remote participants (discovery traffic)-->
                                        <initialPeersList>
                                            <!--container 1 peer-->
                                            <locator>
                                                <udpv4>
                                                    <address>192.168.1.60</address>
                                                    <port>11200</port>
                                                </udpv4>
                                            </locator>
                                            <!-- local container network multicast-->
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

            .. tab:: CONTAINER 2

                .. code-block:: xml

                    <?xml version="1.0" encoding="UTF-8" ?>
                    <dds>
                        <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
                            <participant profile_name="container0" is_default_profile="true">
                                <rtps>
                                    <!-- External locators for user traffic -->
                                    <default_external_unicast_locators>
                                        <udpv4 externality="1" cost="0" mask="24">
                                            <address>192.168.1.11</address>
                                            <port>11201</port>
                                        </udpv4>
                                    </default_external_unicast_locators>

                                    <builtin>
                                        <!-- External locators for discovery traffic -->
                                        <metatraffic_external_unicast_locators>
                                            <udpv4 externality="1" cost="0" mask="24">
                                                <address>192.168.1.11</address>
                                                <port>11200</port>
                                            </udpv4>
                                        </metatraffic_external_unicast_locators>
                                        <!-- Locators of remote participants (discovery traffic)-->
                                        <initialPeersList>
                                            <!--container 1 peer-->
                                            <locator>
                                                <udpv4>
                                                    <address>192.168.1.60</address>
                                                    <port>11200</port>
                                                </udpv4>
                                            </locator>
                                            <!-- local container network multicast-->
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
                                        <ignore_non_matching_locators>true</ignore_non_matching_locators>
                                        <!-- External locators for user traffic -->
                                        <default_external_unicast_locators>
                                            <udpv4 externality="1" cost="0" mask="24">
                                                <address>192.168.1.11</address>
                                                <port>11201</port>
                                            </udpv4>
                                        </default_external_unicast_locators>

                                        <builtin>
                                            <!-- External locators for discovery traffic -->
                                            <metatraffic_external_unicast_locators>
                                                <udpv4 externality="1" cost="0" mask="24">
                                                    <address>192.168.1.11</address>
                                                    <port>11200</port>
                                                </udpv4>
                                            </metatraffic_external_unicast_locators>

                                            <!-- Locators of remote participants (discovery traffic)-->
                                            <initialPeersList>
                                                <!--container 1 peer-->
                                                <locator>
                                                    <udpv4>
                                                        <address>192.168.1.60</address>
                                                        <port>11200</port>
                                                    </udpv4>
                                                </locator>
                                                <!-- local container network multicast-->
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

.. note::

    Note that the docker network itself does not create another level of externality in this case, as the bridge driver to the host network is used when launching the different containers.

Connecting ROS 2 demo nodes through an external network
--------------------------------------------------------

This section provides the step-by-step instructions for setting up the example scenario described in the previous Background section.
Using a terminal, enter the following command in a terminal, on both hosts.

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: CONTAINER 1

                .. code-block:: bash

                    docker run --rm -it --privileged -p 11200-11201:7412-7413/udp -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubuntu-vulcanexus:humbe-desktop

            .. tab:: CONTAINER 2

                .. code-block:: bash

                    docker run --rm -it --privileged -p 11202-11203:7412-7413/udp -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubuntu-vulcanexus:humbe-desktop


    .. tab:: HOST 2

        .. tabs::

            .. tab:: CONTAINER 1

                .. code-block:: bash

                    docker run --rm -it --privileged -p 11200-11201:7412-7413/udp -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubuntu-vulcanexus:humbe-desktop


.. note::

    It is important to specify the port mapping argument for exposing docker internal ports to the host. See `Docker Networking <https://docs.docker.com/config/containers/container-networking/>`_ for further information.

The next step is the creation of the XML profiles.
Inside each one of the three containers, run the following commands and paste the contents of the corresponding XML profile configuration, according to the previous section.

.. code-block:: bash

    sudo apt update && sudo apt install gedit
    gedit /Profiles.xml
    # Paste the corresponding XML Profile configuration

Finally, export the environment variable pointing to the Profiles file, source Vulcanexus environment and run the ros2 example nodes.

.. tabs::

    .. tab:: HOST 1

        .. tabs::

            .. tab:: CONTAINER 1

                .. code-block:: bash

                    source vulcanexus_entrypoint.sh
                    export FASTRTPS_DEFAULT_PROFILES_FILE=/Profiles.xml
                    ros2 run demo_nodes_cpp talker

            .. tab:: CONTAINER 2

                .. code-block:: bash

                    source vulcanexus_entrypoint.sh
                    export FASTRTPS_DEFAULT_PROFILES_FILE=/Profiles.xml
                    ros2 run demo_nodes_cpp listener


    .. tab:: HOST 2

        .. tabs::

            .. tab:: CONTAINER 1

                    .. code-block:: bash

                        source vulcanexus_entrypoint.sh
                        export FASTRTPS_DEFAULT_PROFILES_FILE=/Profiles.xml
                        ros2 run demo_nodes_cpp listener


At this point, nodes should be communicating with each other as expected.
A message containing `Hellow World` string alongside with a counter should be printed in the talker node.
Both listeners with the same count number, receiving and printing it, in the other two containers.
