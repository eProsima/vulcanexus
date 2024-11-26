.. include:: ../../../../exports/alias.include

.. _tutorials_router_fully_connected:

How to avoid Fully-Connected Graph Networks in ROS 2
====================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Facing Scalability Challenges in Your ROS 2 System?
---------------------------------------------------

Have you ever struggled with scaling your ROS 2 network as it grows in size?
As more nodes, topics, and services are added to a ROS 2 system, the underlying DDS (Data Distribution Service) quickly forms a fully-connected graph, meaning that every node discovers and communicates with every other node.
While this setup guarantees robust connectivity, it can also lead to network congestion and high discovery traffic as the system scales, impacting performance and reliability.

**Quick Solution Overview**

Fortunately, there are tools designed to address these scalability challenges, such as the `DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/>`__.
The |ddsrouter| Can encapsulate and route communications, allowing nodes in different domains or even separate LANs to exchange information without every node being directly connected.
This selective routing minimizes traffic, making it possible to maintain high performance even in complex distributed systems.
Basically, instead of communicating all the ROS 2 nodes in the same domain, separe them in different domains, enrouting the data through a |ddsrouter| with a a participant listening to the topics and another one transmiting them.
The following configuration file illustrate an example of |ddsrouter| setup:

.. code-block:: yaml

    version: v4.0

    participants:
      - name: listening_participant
        kind: simple
        domain: 0                          # Publishers domain

      - name: transmiting_participant
        kind: simple
        domain: 1                          # Subscribers domain

After creating the configuration file, to start the |ddsrouter| just run:

.. code:: bash

    ddsrouter -c <path_to_configuration_file>

Overview
--------

In this tutorial, we will explore how the DDS Router can be used to reduce unnecessary connections and streamline communication in a ROS 2 network with multiple hosts connected over the same WiFi network.
On one host, we will run two video `publishers` of the ROS2 package ``image_tools`` node ``cam2image``, while on the other, we will set up two `subscribers` of the ``showimage`` node to receive the video streams.
Instead of directly connecting the `publishers` and `subscribers` across the hosts (resulting in four total connections), we will streamline the communication by introducing a |ddsrouter|  on each host.

By using |ddsrouter|, we can limit the communication to only two participants, one on each host, enhancing both control and efficiency.
Each publisher will communicate solely with a local participant within the |ddsrouter|  on its host, rather than establishing connections with all remote subscribers.
The local |ddsrouter|  participant then links to an XML-configured participant, which manages communication with the corresponding XML participant of the remote |ddsrouter|  on the other host.

This approach minimizes direct cross-host connections, allowing for more scalable and controlled communication between publishers and subscribers across the network.

.. figure:: /rst/figures/tutorials/cloud/fully_connected.svg
   :align: center

.. warning::

    A brief theoretical overview will be provided for additional context.
    However, readers may skip directly to the :ref:`tutorials_router_fully_connected_prerequisites` section if they prefer to start with the setup steps.

Background
----------

In ROS 2, communication between nodes, including publishers, subscribers, service clients, and servers, is managed by DDS (Data Distribution Service).
This results in a fully-connected graph where every node in the network must discover and communicate with every other node.
While this approach ensures robustness and flexibility, it also means that as the number of participants grows, so does the network traffic associated with discovery and communication.

Each participant in the network must be aware of all other participants, leading to an n² growth in discovery traffic as the system expands.
This can make large networks more complex to manage, especially when many topics and services are involved.
The discovery process can generate significant traffic, and when new nodes join, the network can experience a surge in communication, affecting performance.

To improve the efficiency of discovery in large-scale ROS 2 networks, tools like the `Discovery Server <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html#discovery-server>`__ can be used.
The Discovery Server centralizes the discovery process, enabling nodes to connect to a central point rather than independently discovering all other nodes.
This reduces network traffic and simplifies node management.
There are `tutorials <https://docs.vulcanexus.org/en/latest/rst/tutorials/core/discoveryserver/discoveryserver_tutorials.html>`__ available that explain how to integrate the Discovery Server into your ROS 2 network, providing practical steps to optimize discovery and scaling in larger deployments.

DDS Router
^^^^^^^^^^

In addition to the Discovery Server, another powerful tool is the `DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/>`__.
The DDS Router allows users to create a communication bridge that connects two otherwise isolated DDS networks.
It enables **encapsulation and routing** of DDS communications across physically or virtually separated LANs, allowing participants from different networks to publish and subscribe to both local and remote topics seamlessly.

The *DDS Router* works by internally running **Participants**, which act as communication interfaces to various DDS networks.
Each Participant connects to a different DDS network, and when a message is received on one network, the DDS Router forwards it to the others.
This architecture allows systems with different transport protocols, discovery mechanisms, or domain IDs to communicate with each other without manual intervention.
The DDS Router also supports a **zero-copy communication** mechanism, ensuring efficient data transmission by sharing memory pointers between Participants without copying data.

The *DDS Router* can significantly alleviate the challenges posed by the fully-connected graph of participants in DDS, which makes it difficult to scale large networks.
By routing communication between different DDS networks, the DDS Router reduces the need for all participants to directly discover and connect with each other.
This selective encapsulation helps control and minimize the network traffic caused by the full discovery process, making it easier to manage systems with many nodes, topics, and participants.
As a result, the *DDS Router* provides a scalable solution for large ROS 2 networks, optimizing resource usage and improving overall performance.

.. _tutorials_router_fully_connected_prerequisites:

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`__
* `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`__
* `Docker installation <https://docs.vulcanexus.org/en/latest/rst/installation/docker.html>`__

Run this tutorial
-----------------

To run this tutorial, start by publishing images on **Host A**.
You’ll set up two publishers: one streaming images from the webcam and another streaming a predefined image source.
This will publish to the topic ``image``:

.. code:: bash

    ROS_DOMAIN_ID=0 ros2 run image_tools cam2image --ros-args -p reliability:=best_effort

And this will publish on the topic ``burger``:

.. code:: bash

    ROS_DOMAIN_ID=0 ros2 run image_tools cam2image --ros-args -r image:=burger -p burger_mode:=True -p reliability:=best_effort

Next, run the subscriber nodes on **Host B** for each topic.
Assign these nodes to a different domain to prevent direct communication between the publishers and subscribers:

.. code:: bash

    ROS_DOMAIN_ID=2 ros2 run image_tools showimage --ros-args -p reliability:=best_effort

.. code:: bash

    ROS_DOMAIN_ID=2 ros2 run image_tools showimage --ros-args -r image:=burger -p reliability:=best_effort

Finally, set up a |ddsrouter| in each domain to manage and filter communication between the hosts.
This setup will ensure that messages are only passed between two participants, reducing cross-host traffic while maintaining a stable connection.

Each |ddsrouter| will contain a local participant assigned to the same domain as the ROS 2 nodes running on its host, as well as an XML-configured participant optimized for efficient large data communication (see `this tutorial <https://docs.vulcanexus.org/en/latest/rst/tutorials/core/wifi/large_data/large_data.html>`__ for more details).
In this configuration, publishers and subscribers will exchange data with the local participant within their host’s router.
This local participant will then handle forwarding the data to the XML participant, enabling seamless communication across hosts.

To configure communication on **Host A**, we create the following YAML configuration file.
This file defines the participants, their types, and specifies an XML file for additional configuration details:

.. code-block:: yaml

    version: v4.0

    xml:
      files:
        - "<path_to_xml>"

    participants:
      - name: ROS2_Domain_0
        kind: simple
        domain: 0

      - name: ROS2_large_data_A
        kind: xml
        profile: large_data_participant

In this configuration, the participant named ``ROS2_large_data_A`` uses an XML profile called ``large_data_participant``, which is tailored to handle large data transfers.
The following XML file specifies the settings for the `large data <https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/tcp/tcp_large_data_with_options.html>`_ participant:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>

    <dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
      <profiles>
        <participant profile_name="large_data_participant">
          <domainId>1</domainId>
          <rtps>
            <builtinTransports max_msg_size="1MB" sockets_size="1MB" non_blocking="true" tcp_negotiation_timeout="50">LARGE_DATA</builtinTransports>
          </rtps>
        </participant>
      </profiles>
    </dds>

Now, let’s configure **Host B** with a similar YAML setup.
This configuration enables Host B to communicate effectively within the same large-data framework as Host A:

.. code-block:: yaml

    version: v4.0

    xml:
    files:
        - "<path_to_xml>"

    participants:
      - name: ROS2_Domain_2
        kind: simple
        domain: 2

      - name: ROS2_large_data_B
        kind: xml
        profile: large_data_participant

With the following XML file:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>

    <dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
      <profiles>
        <participant profile_name="large_data_participant">
          <domainId>1</domainId>
          <rtps>
            <builtinTransports max_msg_size="1MB" sockets_size="1MB" non_blocking="true" tcp_negotiation_timeout="50">LARGE_DATA</builtinTransports>
          </rtps>
        </participant>
      </profiles>
    </dds>

Once the configuration files are ready, run the |ddsrouter| in each host with:

.. code:: bash

    ddsrouter -c <path_to_yaml_configuration_file>

Now, each publisher and subscriber communicates solely with its local router, which then manages and optimizes data transfer between the hosts.
This configuration ensures efficient, controlled data flow, minimizing network load while providing reliable large data communication across the WiFi network.

Conclusions
-----------

By setting up a |ddsrouter| on each host, this tutorial demonstrates a more efficient and scalable approach to managing ROS 2 communications over a network.
With only two participants handling cross-host communication, the system avoids the typical fully-connected graph structure, which can cause network traffic and complexity to grow as nodes are added.

This selective routing solution reduces unnecessary data transmission between publishers and subscribers, ensuring that only essential information passes between hosts.
As a result, we achieve a stable, optimized network structure that maintains data integrity while minimizing bandwidth usage, making it suitable for larger or bandwidth-sensitive deployments.
The setup provides both robustness and scalability, offering a solid foundation for complex, distributed ROS 2 applications.
