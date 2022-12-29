.. include:: ../../../../exports/alias.include

.. _tutorials_initial_peers_intro:

Adding initial peers QoS Policy
===============================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Vulcanexus offers the possibility of fully configuring Fast DDS' QoS policies through XML profile definition (see :ref:`ROS 2 QoS policies <concepts_about_qos>`).
For more information regarding XML profiles in Vulcanexus please refer to the tutorial :ref:`Configuring Fast-DDS QoS via XML profiles <tutorials_xml_profiles_intro>`.

This section describes how to specify this extended policies, in particular the initial peers configuration.

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), each |RTPSParticipant-api| must listen for incoming Participant Discovery Protocol (PDP) discovery metatraffic in two different ports, one linked with a multicast address, and another one linked to a unicast address.
Vulcanexus, together with *Fast-DDS*,  allows for the configuration of an initial peers list, which contains one or more such IP-port address pairs corresponding to remote DomainParticipants PDP discovery listening resources, so that the local DomainParticipant will not only send its PDP traffic to the default multicast address-port specified by its domain, but also to all the IP-port address pairs specified in the initial peers list.

A complete description of the initial peers list and its configuration can be found in Initial peers |InitialPeersFastDdsDocs|.

.. _initial_peers_prerequisites:

Prerequisites
-------------

First of all, make sure that Vulcanexus Humble is installed.
The docker installation is required for this tutorial (see :ref:`Docker installation <docker_installation>`).


Open two terminals, and run the Vulcanexus Humble image in each one with the following command:

.. code-block:: bash

      docker run \
        -it \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        ubuntu-vulcanexus:humble-desktop


.. _initial_peers_add_qos:

Adding initial peers QoS Policy
-------------------------------

In one of the docker instances, create a XML file, and complete it with the following example:

.. literalinclude:: /resources/tutorials/core/qos/initial_peers/initial_peers_config.xml
    :language: xml

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the RTPSParticipants' discovery traffic unicast listening ports are calculated using the following equation: 7400 + 250 * `domainID` + 10 + 2 *  participantID`.
Thus, if for example a RTPSParticipant operates in Domain 0 (default domain) and its ID is 1, its discovery traffic unicast  listening port would be: 7400 + 250 * 0 + 10 + 2 * 1 = 7412.
By default *eProsima Fast DDS* uses as initial peers the Metatraffic Multicast Locators.

.. important::

    .. Repeat the same procedure in the other opened Vulcanexus docker instance

    It is required to include the XML file in at least one of the Vulcanexus docker instances.
    As long as they are using multicast to discover each other, the participant with the initial peer set would let the other participant know the address and port through which they will communicate.

    .. note ::

        In the last part of the example it is described how to avoid multicast, using unicast and initial peers simultaneously.


.. important::

    The IP address included in the document must be updated with the Vulcanexus docker instance assigned IP address.

.. _initial_peers_ip_address:

Obtain Vulcanexus docker instance IP address
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a terminal, and list the Vulcanexus docker instances.

.. code-block:: bash

      docker container list

The output should look like the following table:

+--------------+----------------------------------+------------------------+---------------+--------------+-------+------------+
| CONTAINER ID | IMAGE                            | COMMAND                | CREATED       | STATUS       | PORTS | NAMES      |
+==============+==================================+========================+===============+==============+=======+============+
| eb81bc8d3a20 | ubuntu-vulcanexus:humble-desktop | "/bin/bash /vulcanex…" | 2 minutes ago | Up 2 minutes |       | keen_jones |
+--------------+----------------------------------+------------------------+---------------+--------------+-------+------------+
| 69ceb36bbacf | ubuntu-vulcanexus:humble-desktop | "/bin/bash /vulcanex…" | 2 minutes ago | Up 2 minutes |       | eager_pike |
+--------------+----------------------------------+------------------------+---------------+--------------+-------+------------+

.. important::

    Check the container ID of the docker instance where the XML was created, and obtain the **other** docker instance IP address.


Using the provided auto-generated instance name (last column content in the output table), check its detailed information.
The following command also filters the output to obtain only the instance IP address:

.. code-block:: bash

      docker inspect <instance_autogenerated_name(keen_jones)> | grep IPAddress


Edit the previously created XML file, and include the IP address of the **other** instance.

.. important::

    Make sure the IP address set in the xml document is NOT the instance's own IP address.

.. _initial_peers_xml_location:

XML configuration file location
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to use the profiles, it is required to let the application know it.
There are two possibilities for providing *Fast DDS* with XML configuration files:

* **Recommended**: Setting the location with environment variable ``FASTRTPS_DEFAULT_PROFILES_FILE`` to contain the path to the XML configuration file (see `Environment Variables <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#env-vars>`_).

.. code-block:: bash

      export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>

* **Alternative**: Placing the XML file in the running application directory under the name *DEFAULT_FASTRTPS_PROFILES.xml*.

For example:

.. code-block:: bash

    cp <path_to_xml_file> DEFAULT_FASTRTPS_PROFILES.xml
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1

.. important::

    Repeat the same procedure in the other opened Vulcanexus docker instance.

.. _initial_peers_example:

Example
-------

Once both Vulcanexus docker environments have been configured, run a demo application in each instance:

.. code-block:: bash

      # Talker
      ros2 run demo_nodes_cpp talker

      # Listener
      ros2 run demo_nodes_cpp listener

The Vulcanexus docker instances would discover each other via multicast, then one of them would let the other know its address and port through it would establish any communication, and they will connect.

.. _initial_peers_unicast_example:

Unicast Example
^^^^^^^^^^^^^^^

If unicast is required instead of multicast, the following profile should be updated in the existing XML profile:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
        <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
            <participant profile_name="initial_peers_example_profile" is_default_profile="true">
                <rtps>
                    <builtin>
                        <initialPeersList>
                            <locator>
                                <udpv4>
                                    <address>172.17.0.3</address> <!-- "keen_jones" IP address -->
                                    <port>7777</port>             <!-- "keen_jones" port       -->
                                </udpv4>
                            </locator>
                        </initialPeersList>
                        <metatrafficUnicastLocatorList>
                            <locator>
                                <udpv4>
                                    <address>172.17.0.2</address> <!-- "eager_pike" IP address -->
                                    <port>7666</port>             <!-- "eager_pike" port       -->
                                </udpv4>
                            </locator>
                        </metatrafficUnicastLocatorList>
                    </builtin>
                </rtps>
            </participant>
        </profiles>

In this case, the port `7777` is not longer the default port, it is a new custom port to reach the peer address.
Furthermore, the meta-traffic unicast locator has been set with the Vulcanexus docker instance IP address (own address), and the port it would be communicating through.

Create a new XML file and complete it with the following example in the non-existing xml file docker instance:

.. literalinclude:: /resources/tutorials/core/qos/initial_peers/initial_peers_unicast_config.xml
    :language: xml

Finally, the addresses and ports have been swapped compared to the previous XML file, in order to match IP addresses and ports in which they are set to use.

.. important::

    Make sure the XML profile has been configured in both docker instances:

    .. code-block:: bash

      export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>

Run the application example again in both Vulcanexus docker instances:

.. code-block:: bash

      # Talker
      ros2 run demo_nodes_cpp talker

      # Listener
      ros2 run demo_nodes_cpp listener
