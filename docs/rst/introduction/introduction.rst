.. _vulcanexus_global_introduction:

Vulcanexus
==========

*Vulcanexus* is a *ROS 2* (Robot Operating System) all-in-one tool set.
It allows users to build robotics applications combining the unique *Vulcanexus* elements with the *ROS 2* libraries, having *Fast DDS* as its fixed middleware implementation.

These open source elements include numerous features and tools, providing *Vulcanexus* users customizable solutions while improving overall system performance.
With *Vulcanexus*, users have fast access to constantly improving functionalities, such as the latest *Fast DDS* version along with its new features.
Vulcanexus being always at the bleeding edge of eProsima's set of tools and libraries allows users to quickly test new features and fixes, gather metrics on your ROS deployment using the Statistics Backend or even deploy against XRCEs with the Micro-ROS Agent.

Vulcanexus Basics
-----------------

As said before, Vulcanexus is a ROS 2 all-in-one tool set which implies that Vulcanexus shares the underlying concepts, principles and architecture with ROS.

*ROS* (Robot Operating System) is an open-source suite for robot development.
Not an Operating System in the traditional sense, it is a framework that provides a huge range of libraries and tools intended to ease development for a wide variety of devices using several different technologies by providing a unified set of APIs and conventions.

ROS 2 is the latest iteration of this philosophy, modernizing ROS's code base and adding support for Real Time programming.
Some of it's most important concepts are listed below.

Graph Model
^^^^^^^^^^^
One of the main strengths of ROS 1 (and of its successor, ROS 2) is its Graph Model. ROS deployments, no mater the size, can be represented by a graph structure, with Nodes as the vertexes of the graph and the Topics connecting them as the edges.
This architecture provides several benefits to the system. For instance, a faulty node could crash without compromising the whole system providing improved reliability over more monolithic deployments. It also allows for faster prototyping, nodes can be started, stopped and swapped as long as their Topics stay the same.

Nodes
^^^^^
As explained earlier, Nodes are one of the main entities in the ROS 2 architecture. Most of ROS client code is in the form of a ROS node. Nodes can house other kinds of entities depending on their purpose. They can define Publishers to provide data for the rest of the nodes, they can use Subscribers to subscriber to topics and receive data on the network and so on and so forth.
Multiple nodes can be grouped inside the same process to leverage the lower overhead and higher speed of interprocess communication or they can be spread over different processes to provide redundancy and fault tolerance to your architecture.

Topics
^^^^^^
Topics are the means by which the different Nodes communicate with each other. Each Topic has a uniquely identifying name inside their namespace, and a user-defined type that specifies the kind and layout of messages that are being transmitted through it.
Topics follow a publication/subscription model. Nodes require a Publisher on a Topic to send data and must have an active subscription on a Topic to receive it.

Services
^^^^^^^^
Services are in Remote Procedure Calls between Nodes. They are akin to Topics in the sense that they are also a means for Nodes to exchange information. Nodes advertise services that are actions that can be requested and will provide a result produced by their registered callback.
These are provided by Nodes via a different set of entities, Servers and Clients.

Parameters
^^^^^^^^^^
ROS 1 had a database used to store Node information called the parameter server. This database provided a centralized location to store and access Node information that did not change frequently (such as configuration values).
In ROS 2 this approach was discarded in favor of a distributed one. Each Node would now publish its own parameters for the other Nodes to see (and interact with them).
