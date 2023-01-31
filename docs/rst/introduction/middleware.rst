.. _vulcanexus_middleware:

Vulcanexus MiddleWare
=====================

Vulcanexus supports *eProsima Fast DDS* as the official middleware.
*Fast DDS* provides many features of interest to the development of robotics applications, providing also custom solutions to well-known problems as, for instance, discovery issues for large deployments.
This section provides an introduction to *Fast DDS* and its main advantages.

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

* **Sync and Async publication modes:** *eProsima Fast DDS* supports both synchronous and asynchronous data publication.

* **Best effort and reliable communication:** *eProsima Fast DDS* supports an optional reliable communication paradigm over *Best Effort* communications protocols such as UDP.
  Furthermore, another way of setting a reliable communication is to use the also supported TCP transport.

* **Transport layers:** *eProsima Fast DDS* implements an architecture of pluggable transports: UDPv4, UDPv6, TCPv4, TCPv6 and SHM (shared memory).

* **Security:** *eProsima Fast DDS* can be configured to provide secure communications.
  For this purpose, it implements pluggable security at three levels: authentication of remote participants, access control of entities and encryption of data.

* **Statistics Module:** *eProsima Fast DDS* can be configured to gather and provide information about the data being exchanged by the user application.

* **Plug-and-play Connectivity:** New applications and services are automatically discovered, and can join and leave the network at any time without the need for reconfiguration.

* **Scalability and Flexibility:** DDS builds on the concept of a global data space.
  The middleware is in charge of propagating the information between publishers and subscribers.
  This guarantees that the distributed network is adaptable to reconfigurations and scalable to a large number of entities.

* **Application Portability:** The DDS specification includes a platform specific mapping to IDL, allowing an application using DDS to switch among DDS implementations with only a re-compile.

* **Extensibility:** *eProsima Fast DDS* allows the protocol to be extended and enhanced with new services without breaking backwards compatibility and interoperability.

* **Configurability and Modularity:** *eProsima Fast DDS* provides an intuitive way to be configured, either through code or XML profiles.
  Modularity allows simple devices to implement a subset of the protocol and still participate in the network.

* **High performance:** *eProsima Fast DDS* uses a static low-level serialization library, `Fast CDR <https://github.com/eProsima/Fast-CDR>`__, a C++ library that serializes according to the standard CDR serialization mechanism defined in the `RTPS Specification <https://www.omg.org/spec/DDSI-RTPS/>`__ (see the Data Encapsulation chapter as a reference).

* **Easy to use:** The interactive demo :ref:`vulcanexus_shapes_demo` is available for the user to dive into the DDS world.

* **Low resources consumption:** *eProsima Fast DDS*:

  * Allows to preallocate resources, to minimize dynamic resource allocation.
  * Avoids the use of unbounded resources.
  * Minimizes the need to copy data.

* **Multi-platform:** The OS dependencies are treated as pluggable modules.
  Users may easily implement platform modules using the *eProsima Fast DDS* library on their target platforms.
  By default, the project can run over Linux, Windows and MacOS.

* **Free and Open Source:** *eProsima Fast DDS*, the internal dependencies (such as *eProsima Fast CDR*) and the external ones (such as the *foonathan* library) are free and open source.

.. _vulcanexus_discovery_server:

ROS 2 Discovery Server
----------------------

.. note::
    This section is under maintenance and will be updated soon.

