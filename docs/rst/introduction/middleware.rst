.. _vulcanexus_middleware:

Vulcanexus Middleware
=====================

Vulcanexus supports *eProsima Fast DDS* as the official middleware.
*Fast DDS* provides many features of interest to the development of robotics applications, providing also custom solutions to well-known problems as, for instance, discovery issues for large deployments.
This section provides an introduction to *Fast DDS* and its main advantages.
More information can be found in `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/latest/>`_.

.. _vulcanexus_fastdds:

Fast DDS
--------

*eProsima Fast DDS* is a C++ implementation of the `DDS (Data Distribution Service) Specification <https://www.omg.org/spec/DDS/About-DDS/>`__, a protocol defined by the `Object Management Group (OMG) <https://www.omg.org/>`__.
The *eProsima Fast DDS* library provides both an Application Programming Interface (API) and a communication protocol that deploy a Data-Centric Publisher-Subscriber (DCPS) model, with the purpose of establishing efficient and reliable information distribution among Real-Time Systems.
*eProsima Fast DDS* is predictable, scalable, flexible, and efficient in resource handling.
For meeting these requirements, it makes use of typed interfaces and hinges on a many-to-many distributed network paradigm that neatly allows separation of the publisher and subscriber sides of the communication.

Main Features
^^^^^^^^^^^^^

* **Real-Time behaviour:** *eProsima Fast DDS* can be configured to offer real-time features, guaranteeing responses within specified time constrains.

* **Built-in Discovery Server:** *eProsima Fast DDS* is based on the dynamical discovery of existing publishers and subscribers, and performs this task continuously without the need to contacting or setting any servers.
  However, a Client-Server discovery as well as other discovery paradigms can also be configured.
  See :ref:`vulcanexus_discovery_server` for more information.
  Several tutorials are available showcasing different discovery mechanisms and strategies: :ref:`discovery server <ros2-advanced-tutorial-fastdds-discovery-server>`, :ref:`initial peers <tutorials_initial_peers_intro>`, and :ref:`static EDP discovery <tutorials_deployment_static_edp>`.

* **Sync and Async publication modes:** *eProsima Fast DDS* supports both synchronous and asynchronous data publication.
  See :ref:`vulcanexus_configure_publication_mode` and :ref:`ros2_tutorials_fastdds_sync_async` tutorials for more information.

* **Best effort and reliable communication:** *eProsima Fast DDS* supports an optional reliable communication paradigm over *Best Effort* communications protocols such as UDP.
  Furthermore, another way of setting a reliable communication is to use the also supported TCP transport.

* **Transport layers:** *eProsima Fast DDS* implements an architecture of pluggable transports: UDPv4, UDPv6, TCPv4, TCPv6 and SHM (shared memory).
  See :ref:`tutorials_core_deployment_custom_transports` tutorial for more information.

* **Security:** *eProsima Fast DDS* can be configured to provide secure communications.
  For this purpose, it implements pluggable security at three levels: authentication of remote participants, access control of entities and encryption of data.
  *Fast DDS* also supports hardware secure modules implementing PKCS#11 as it is showcased in :ref:`tutorials_security_pkcs11_pkcs11` tutorial.

* **Statistics Module:** *eProsima Fast DDS* can be configured to gather and provide information about the data being exchanged by the user application.
  Several tutorials are available showcasing different options to obtain the statistics within Vulcanexus ecosystem: :ref:`ROS 2 Monitor <tutorials_tools_fastdds_monitor>`, :ref:`Fast DDS Statistics Backend <tutorials_tools_monitor_with_backend>`, :ref:`SQL <tutorials_tools_monitor_sql>`, and :ref:`Prometheus <tutorials_tools_prometheus>`.

* **Plug-and-play Connectivity:** New applications and services are automatically discovered, and can join and leave the network at any time without the need for reconfiguration.

* **Scalability and Flexibility:** DDS builds on the concept of a global data space.
  The middleware is in charge of propagating the information between publishers and subscribers.
  This guarantees that the distributed network is adaptable to reconfigurations and scalable to a large number of entities.

* **Application Portability:** The DDS specification includes a platform specific mapping to IDL, allowing an application using DDS to switch among DDS implementations with only a re-compile.

* **Extensibility:** *eProsima Fast DDS* allows the protocol to be extended and enhanced with new services without breaking backwards compatibility and interoperability.

* **Configurability and Modularity:** *eProsima Fast DDS* provides an intuitive way to be configured, either through code or XML profiles.
  Modularity allows simple devices to implement a subset of the protocol and still participate in the network.
  See :ref:`tutorials_xml_profiles_intro` tutorial for more information.

* **High performance:** *eProsima Fast DDS* uses a static low-level serialization library, `Fast CDR <https://github.com/eProsima/Fast-CDR>`__, a C++ library that serializes according to the standard CDR serialization mechanism defined in the `RTPS Specification <https://www.omg.org/spec/DDSI-RTPS/>`__ (see the Data Encapsulation chapter as a reference).

* **Easy to use:** The interactive demo :ref:`vulcanexus_shapes_demo` is available for the user to dive into the DDS world.
  See :ref:`tutorials_tools_shapes_demo` tutorial for more information.

* **Low resources consumption:** *eProsima Fast DDS*:

  * Allows to preallocate resources, to minimize dynamic resource allocation.
  * Avoids the use of unbounded resources.
  * Minimizes the need to copy data.

* **Multi-platform:** The OS dependencies are treated as pluggable modules.
  Users may easily implement platform modules using the *eProsima Fast DDS* library on their target platforms.
  By default, the project can run over Linux, Windows and MacOS.

* **Free and Open Source:** Both *eProsima Fast DDS*, its internal dependencies (such as *eProsima Fast CDR*), and its external ones (such as the *foonathan* library) are free and open source.

.. _vulcanexus_discovery_server:

ROS 2 Discovery Server
----------------------

:ref:`ROS 2 Discovery Server <ros2-advanced-tutorial-fastdds-discovery-server>` is a Fast DDS enabled feature that procures an alternative discovery mechanism to the default ROS 2 discovery mechanism, `Simple Discovery Protocol (SDP) <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#simple-disc-settings>`_, which is served by the DDS implementations according to the DDS specification.
Whereas SDP provides automatic out-of-the-box discovery by leveraging multicast, ROS 2 Discovery Server provides a centralized hub for managing discovery which drastically reduces network bandwidth utilization when compared to SDP, since the nodes, publishers, and subscribers, only discovered those remote ROS 2 entities with which they need to communication (as opposed to the SDP model where everyone knows about each other).
Furthermore, it does not relay on multicast, which makes this mechanism more robust over WiFi, and simplifies ROS 2 deployments in managed networks, where the use of multicast is often restricted.
Its main features are:

* **Ease of use**: Vulcanexus (through Fast DDS) provides a CLI to instantiate Discovery Servers with one command.
  To connect a node (Client) to the Discovery Server, a simple environment variable is used (much like in ROS 1).

* **Scalability**: The discovery related network traffic can be reduced by more than an 85 % margin when compared to SDP.

* **Robustness**: ROS 2 Discovery Server supports redundant servers, effectively removing the single point of failure that its ROS 1 counterpart (ROS Master) entailed.

* **Run-time mutability**: It is possible to change the Server to which a node (Client) connects on run-time.

* **Ease of deployment**: All that is necessary to get rid of all ROS 2 discovery related problems during deployment (WiFi, multicast, bandwidth exhaustion, etc.) is a process to run the Discovery Server, and an environment variable to configure the Clients.

.. figure:: /rst/figures/intro/discovery-server.svg
    :align: center
    :width: 50%

.. note::

    Please refer to the `Discovery Server documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html>`_ for more information on all possible Discovery Server configuration options and use-cases.
