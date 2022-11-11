.. _vulcanexus_router:

ROS 2 Router
============

*ROS 2 Router* (a.k.a. *DDS Router*) is an end-user software application shipped with *Vulcanexus Cloud* that enables the connection of distributed *ROS 2* networks.
That is, *ROS 2* nodes deployed in one geographic location and using a dedicated local network will be able to communicate with other nodes deployed in different geographic areas on their own dedicated local networks as if they were all on the same network through the use of *ROS 2 Router*.
This is achieved by deploying a *ROS 2 Router* on an edge device of each local network so that the *ROS 2 Router* routes DDS traffic from one network to the other through WAN communication.

Furthermore, *ROS 2 Router* is a software designed for various forms of distributed networks, such as mesh networks in which nodes are deployed in different private local networks that are auto-discovered without any centralized network node, or cloud-based networks where there is a data processing cloud and multiple geographically distributed edge devices.

########
Overview
########

Following are some of the key features of *ROS 2 Router*:

  1. **WAN communication over TCP**: it supports WAN over TCP communication to establish communications over the Internet.
  2. **Distributed nature**: the user may deploy intermediate *ROS 2 Router* nodes to discover new entities that enter and leave the network dynamically.
  3. **Efficient data routing**: *ROS 2 Router* avoids data introspection achieving a zero-copy system in data forwarding.
  4. **Easy deployment**: it is based on an easily configurable modular system for users with no knowledge of computer networks.
  5. **Topic allowlisting**: it is possible to configure a *ROS 2 Router* to forward just the user data belonging to a topic specified by the user.
  6. **Dynamic topic discovery**: the user does not need to fully specify over which topics to communicate (i.e. provide concrete topic names and types).
     The discovery of topics matching the allowlisting rules automatically triggers the creation of all entities required for communication.
  7. **Quality of Service preservation**: *ROS 2 Router* uses the QoS set in the user's *ROS 2* network and keeps the reliability and durability of the communication for each topic.
     These QoS are also manually configurable.

.. figure:: /rst/figures/intro/tools/router/ddsrouter_cloud.png

##########
Next Steps
##########

Visit `eProsima DDS Router Documentation <https://eprosima-dds-router.readthedocs.io/en/latest>`_ for more information on how to configure and deploy a *ROS 2 Router* instance.
Also feel free to review :ref:`Vulcanexus Cloud Tutorials <tutorials_cloud_cloud_tutorials>` to discover the possibilities that this tool has to offer in different scenarios.
