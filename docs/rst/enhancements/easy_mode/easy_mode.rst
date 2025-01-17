.. _easy_mode:

Easy Mode
=========

The new ``Vulcanexus Easy Mode`` aims to simplify, enhance and optimize the deployment of any ROS 2 application reinforcing the overall out-of-the-box user experience.
Moreover, it claims to be the default discovery behavior within :ref:`Vulcanexus <vulcanexus_introduction>`.
This section reveals its significance, operating mode and impact.

Quick Overview
^^^^^^^^^^^^^^

The ``Vulcanexus Easy Mode`` is a builtin discovery mode that simplifies to the most the deployment of ROS 2 applications with the use of `Discovery Server <https://eprosima-discovery-server.readthedocs.io/en/latest/>`_.
To enable this feature the user only needs to set the environment variable ``EASY_MODE=<ip>``.
This ``<ip>`` can be either the IP address of the current host or the ``<ip>`` of an external host in the same LAN acting as the central point of discovery (``master``).
A new discovery server will be spawned in background in the domain specified by the ``ROS_DOMAIN_ID`` environment variable.
The following diagram illustrates the concept:

.. image:: ../../figures/enhancements/easy_mode/easy_mode_quick_intro.png
    :align: center
    :width: 55%

The advantages of the ``Vulcanexus Easy Mode`` can be explored in :ref:`easy_mode_benefits`.

Feature Insights
^^^^^^^^^^^^^^^^

This sub-section provides a detailed explanation of the ``Vulcanexus Easy Mode``.

Motivation
----------

ROS 2 users often expect ROS 2 applications to run out-of-the-box without caring about middleware configuration or network setup.
As a result, experience has revealed that the default `DDS Simple discovery mechanism <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html>`_ with multicast is not always the best choice for the majority of use cases.
However, `eProsima's Discovery Server <https://eprosima-discovery-server.readthedocs.io/en/latest/>`_ mechanism effectively removes the need for multicast discovery traffic and presents a flexible solution for different network topologies.

.. list-table::
   :width: 100%
   :class: borderless

   * - .. image:: ../../figures/enhancements/easy_mode/simple_discovery_multiple_hosts.png
          :width: 100%

     - .. image:: ../../figures/enhancements/easy_mode/easy_mode_multiple_hosts.png
          :width: 100%

This method proposes a centralized discovery approach by means of a Discovery Server entity.
`Investigation and comparison of both methods <https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/ros2/discovery_server/ros2_discovery_server.html#discovery-server-v2>`_ have shown that the number of discovery packets is drastically reduced and the system scalability improves when using the Discovery Server.
The previous image shows how Discovery Server simplifies the discovery graph compared with the default DDS Simple discovery for a localhost deployment of four ROS 2 nodes.

Background
----------

*Vulcanexus* uses :ref:`Fast DDS as middleware <vulcanexus_middleware>`.
DDS entities require those to have information about each other to communicate.
Hence, before ROS 2 nodes (publishers, subscriptions, services,...) start exchanging messages, they must first discover each other.
The *Simple Discovery Protocol* is the standard protocol defined in the DDS standard.
However, it has known disadvantages in some scenarios:

* Scalability, as the number of exchanged packets increases significantly as new nodes are added.
* Requires multicast capabilities that may not work reliably in some scenarios, e.g. WiFi.

The *Fast DDS Discovery Server* offers a Client-Server framework that enables nodes to connect through an intermediary server.
Each node acts as a discovery client, transmitting its information to one or more discovery servers and obtaining discovery data from them.
This approach minimizes network traffic associated with discovery and eliminates the need for multicast functionality.

Another relevant concept in ROS 2 is the `ROS_DOMAIN_ID <https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html>`_.
The *ROS_DOMAIN_ID* is a unique identifier that allows multiple ROS 2 applications to run on the same network without interfering with each other.
It is a way of isolating different ROS 2 application groups that do not need to communicate with each other.
Internally, it is directly related to the DDS Domain ID, which, in turn, selects different network port ranges for each domain.
See the `ROS 2 documentation <https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html>`_ for further explanation.

Understanding Easy Mode
-----------------------

The new ``Vulcanexus Easy Mode`` can be enabled by simply setting the already known environment variable ``EASY_MODE`` to an IP (later explained).
The transports configured in this new mode include ``TCP`` for discovery and user data and ``Shared Memory`` for user data (in case it is `possible to use <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html>`_).

When the first ROS 2 node is launched, it will automatically spawn a Discovery Server instance in the given domain i.e the one specified in the ``ROS_DOMAIN_ID`` (0 by default) and make the ROS 2 node a client pointing to it.
If a Discovery Server is already running in the domain, the node will simply connect to it as a client, and no additional servers will be spawned.
Therefore, only one Discovery Server will be present in each domain.
The following diagram illustrates this concept:

.. image:: ../../figures/enhancements/easy_mode/multiple_ds_domain.png
    :align: center
    :width: 45%

Servers connection
------------------

A direct consequence of the ``Easy Mode`` is that every ROS 2 node has a well defined discovery scope depending on the ``<ip>``.
This allows it to discover other ROS 2 nodes either on the same host only, or also the ones on a remote host.

The next image shows an example of this latter case:

.. image:: ../../figures/enhancements/easy_mode/easy_mode_connecting_servers.png
    :align: center
    :width: 55%

The solid red arrow represents that the discovery server in Host B points to the master in Host A.
Then, the dashed arrow represents that both servers will discovery each other and finally, the green arrow represents the data exchange between the ROS 2 nodes in purple (meaning that they share the same topic).

The act of connecting Discovery Servers can be also done in other fashions, such as:

* Setting the environment variable ``ROS_STATIC_PEERS`` with the pair ``<ip_address:domain_id>`` of the server to connect to.
* Using the ``fastdds discovery`` cli tool with the sub commands ``add`` or ``set`` followed by the domain and the ``<ip_address:domain_id>`` pair.

Please, refer to the `Fast DDS documentation <TODO:Insert LINK>`_ for further information in this regard.

The figure below shows a general case where multiple Discovery Servers are connected to each other:

.. image:: ../../figures/enhancements/easy_mode/easy_mode_general.png
    :align: center
    :width: 80%

On the left part of the figure, when the discovery servers in Hosts' B and C are connected to an external master server in Host A (i.e setting ``EASY_MODE=<host_a_ip>``), servers in Host B and C will automatically connect to each other.
This suggests that connecting to a Discovery Server is equivalent to connecting to all the servers that this server is connected to, because connected servers automatically conform to a `mesh topology <https://www.bbc.co.uk/bitesize/guides/z7mxh39/revision/6>`_.
See `Discovery Server documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html>`_.

At the same time, the right part of the figure illustrates the case of a different domain in which Hosts B and C are operating in localhost i.e having the ``EASY_MODE`` to their own host's IP.
Which is perfectly valid and can coexist with the other domain.

For a practical example demo, please refer to the :ref:`easy_mode_tutorial`.

.. _easy_mode_benefits:

Benefits
^^^^^^^^

The use of ``Vulcanexus Easy Mode`` brings the following benefits:

* **Simplicity**: The user does not need to worry about configuring the middleware, enhancing the out-of-the-box experience.
  The user easily manages connections between domains and hosts only when needed.
* **Scalability**: The number of discovery packets is drastically reduced, and the system scalability improves.
* **Reliability**: The new discovery mode is more reliable than Simple discovery with multicast, especially in WiFi scenarios, as it uses peer to peer TCP.
