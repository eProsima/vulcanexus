.. include:: ../../../../exports/alias.include

.. _tutorials_initial_peers_intro:

Static discovery between ROS 2 nodes with Initial Peers
=======================================================

Background
----------

As most ROS 2 developers already know, Fast DDS is the default middleware of ROS 2, so this tutorial focuses on applying a direct configuration to the Fast DDS middleware so that you can modify its settings without the need to do it through the exposed API of ROS 2.

Vulcanexus offers the possibility of fully configuring Fast DDS' QoS policies through XML profile definition (see :ref:`ROS 2 QoS policies <concepts_about_qos>`).
For more information on how to configure ROS 2 through XML profiles please refer to :ref:`Configuring Fast-DDS QoS via XML profiles <tutorials_xml_profiles_intro>` tutorial.

This section describes how to specify this extended policies, in particular the initial peers configuration, which applies to the DDS DomainParticipant QoS.
For this purpose, it is important to take into consideration the mapping between ROS 2 entities and DDS DomainParticipants (see `Node to Participant mapping <https://design.ros2.org/articles/Node_to_Participant_mapping.html>`_).
In short, a ROS 2 Context is equivalent to a DDS DomainParticipant, so a DDS DomainParticipant can contain multiple ROS 2 Nodes (see :ref:`Creating ROS contexts and nodes <vulcanexus_configure_xml_creatingROScontext_nodes>`).
Therefore, this tutorial only applies to deployments with more than one ROS 2 Context.

According to the DDS standard, each DomainParticipant must listen for incoming DomainParticipant discovery metatraffic in two different ports, one linked with a multicast address, and another one linked to a unicast address.
Vulcanexus, together with *Fast DDS*, allows configuring an initial peers list, which contains one or more such IP-port address pairs corresponding to remote DomainParticipants discovery listening resources, so that the local DomainParticipant will not only send its PDP traffic to the default multicast network address, but also to all the IP-port pairs specified in the initial peers list.

A complete description of the initial peers list and its configuration can be found in |InitialPeersFastDdsDocs| in *Fast DDS* documentation.

.. _initial_peers_prerequisites:

Prerequisites
-------------

First of all, make sure that Vulcanexus kilted is installed.
The docker installation is required for this tutorial (see :ref:`Docker installation <docker_installation>`).

Open two terminals, and run the Vulcanexus kilted image in each one with the following command:

*   Terminal 1:

    .. code-block:: bash

        docker run -it --name ros2_context_1 ubuntu-vulcanexus:kilted-desktop


*   Terminal 2:

    .. code-block:: bash

        docker run -it --name ros2_context_2 ubuntu-vulcanexus:kilted-desktop

.. note::

    It is highly recommended to complete the :ref:`Configuring Fast-DDS QoS via XML profiles <tutorials_xml_profiles_intro>` tutorial to learn how to configure ROS 2 via XML configuration files.

.. _initial_peers_add_qos:

Adding initial peers QoS Policy
-------------------------------

In the docker container named ``ros2_context_1``, create a XML file, and complete it with the following example:

.. literalinclude:: /resources/tutorials/core/qos/initial_peers/initial_peers_config.xml
    :language: xml

.. note::

    (Advanced users) According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the participant discovery traffic unicast listening ports are defined in the `Well Known Ports <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/listening_locators.html?highlight=well%20known#well-known-ports>`_ section of the Fast DDS documentation.
    Thus, in this example, the Participant operates in Domain 0 (default domain) and its ID is 1, so its discovery traffic unicast listening port is 7412.
    By default *eProsima Fast DDS* uses as initial peers the metatraffic multicast locators (network addresses).

It is required to include the XML file in at least one of the Vulcanexus docker containers, as the participant with the initial peer set would let the other participant know the address and port through which they will communicate.

.. _initial_peers_ip_address:

Obtain Vulcanexus docker container IP address
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The IP address included in the document must be updated with the Vulcanexus docker container assigned IP address.

Open a terminal, and list the Vulcanexus docker containers.

.. code-block:: bash

      docker container list

The output should look like the following table:

+-----------------+---------------------------------+----------------+---------------+--------------+-------+----------------+
| CONTAINER ID    | IMAGE                           | COMMAND        | CREATED       | STATUS       | PORTS | NAMES          |
+=================+=================================+================+===============+==============+=======+================+
| <container ID1> | ubuntu-vulcanexus:kilted-desktop | "…vulcanexus…" | 2 minutes ago | Up 2 minutes |       | ros2_context_1 |
+-----------------+---------------------------------+----------------+---------------+--------------+-------+----------------+
| <container ID2> | ubuntu-vulcanexus:kilted-desktop | "…vulcanexus…" | 2 minutes ago | Up 2 minutes |       | ros2_context_2 |
+-----------------+---------------------------------+----------------+---------------+--------------+-------+----------------+

Using the container name, check its detailed information.
The following command also filters the output to obtain only the container IP address:

.. code-block:: bash

      docker inspect ros2_context_2 | grep IPAddress


Edit the previously XML file created in the ``ros2_context_1`` container, and include the IP address of the ``ros2_context_2`` container.
Make sure the IP address set in the xml file is NOT the container's own IP address.

.. _initial_peers_xml_location:

XML configuration file location
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to use the profiles loaded from XML configuration files, it is required to let the application know it.
There are two possibilities for providing ROS 2 with XML configuration files:

*   **Recommended**: Setting the location in ``FASTRTPS_DEFAULT_PROFILES_FILE`` environment variable, which may contain the path to the XML configuration file (see `Environment Variables <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#env-vars>`_).

    .. code-block:: bash

        export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>

*   **Alternative**: Renaming the XML file to *DEFAULT_FASTRTPS_PROFILES.xml* and placing it in the running application directory.
    For example:

    .. code-block:: bash

        cp <path_to_xml_file> DEFAULT_FASTRTPS_PROFILES.xml
        export RMW_FASTRTPS_USE_QOS_FROM_XML=1


.. _initial_peers_example:

Example
-------

Once the ROS 2 context running in ``ros2_context_1`` Vulcanexus container has been configured, run a demo application in each container.

*   ``ros2_context_1`` container (terminal 1):

    .. code-block:: bash

        ros2 run demo_nodes_cpp talker

*   ``ros2_context_2`` container (terminal 2):

    .. code-block:: bash

        ros2 run demo_nodes_cpp listener

The listener would receive the messages sent by the talker.

.. _initial_peers_unicast_example:

Unicast Example
^^^^^^^^^^^^^^^

In the previous example, both multicast and unicast communication were used for entity discovery.
Next, it is shown how to fully disable multicast communication for discovery, so that the only means of discovery is through unicast communication.
In this case, one of the participants will know in advance the listening address of the other participant and will send its discovery information via unicast communication.
The second participant will then reply to the first participant using the network address contained in the information received.
After the discovery, both will start the transmission of user data.

Replace the content of the existing XML configuration file in the ``ros2_context_1`` container with the one shown below.

.. literalinclude:: /resources/tutorials/core/qos/initial_peers/initial_peers_unicast_config_context1.xml
    :language: xml

In this case, by defining a meta-traffic unicast locator, the Participant creates a unicast meta-traffic receiving resource (communication status and discovery protocol data) for each address-port pair specified (see `Disabling all Multicast Traffic <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/disabling_multicast.html?highlight=meta%20traffic#disabling-all-multicast-traffic>`_).
In that way, and by setting a custom port as initial peer, both ROS 2 context would communicate exclusively by each specified network address and ports (avoiding multicast and the usage of any default ports).


Then, create a new XML file and complete it with the following example in the ``ros2_context_2`` container:

.. literalinclude:: /resources/tutorials/core/qos/initial_peers/initial_peers_unicast_config_context2.xml
    :language: xml

After comparing the two files, it is worth noting that the network address that the ``ros2_context_1`` container is trying to connect (as a initial peer) is now the listening network address in the ``ros2_context_2`` container (as a meta-traffic unicast), and vice versa.

Make sure the XML profile has been configured in both docker containers:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>

Run the application example again in both Vulcanexus docker containers:

* ``ros2_context_1`` container (terminal 1):

    .. code-block:: bash

        ros2 run demo_nodes_cpp talker

* ``ros2_context_2`` container (terminal 2):

    .. code-block:: bash

        ros2 run demo_nodes_cpp listener

The listener would receive the messages sent by the talker.
