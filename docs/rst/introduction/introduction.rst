.. _vulcanexus_global_introduction:

Vulcanexus Overview
===================

*Vulcanexus*, the *ROS 2* all-in-one tool set, expands the *ROS 2* environment improving the developer experience with *ROS 2* providing open source features and tools not available with the current `ROS 2 release <https://www.ros.org/reps/rep-2000.html#release-schedule>`_.
Consequently, *Vulcanexus* shares the underlying concepts, principles and architecture of :doc:`ROS 2 <../../ros2_documentation/source/index>`.
For more information about *ROS 2* please refer to the 2022 Science Robotics vol. 7 (May 2022) paper `Robot Operating System 2: Design, architecture, and uses in the wild <https://www.science.org/doi/10.1126/scirobotics.abm6074>`_ by S. Macenski, T. Foote, B. Gerkey, C. Lalancette and W. Woodall.
This section will follow and summarize the most important points contained in this paper, pointing out the differences and improvements that *Vulcanexus* provides.

Scope
-----

*ROS 2* provides a software ecosystem to develop robotics applications (or also known as a Software Development Kit or SDK).
It is not an Operating System (OS) in the traditional sense, but a framework that provides a huge range of libraries and tools intended to ease development for a wide variety of devices using several different technologies by providing a unified set of APIs and conventions.
The main categories within this ecosystem are:

Middleware
^^^^^^^^^^

The middleware layer is charged with the communication between components.
After considering several communication protocols the `Object Management Group (OMG) <https://www.omg.org/>`_ `DDS (Data Distribution Service) <https://www.omg.org/spec/DDS/About-DDS/>`_ protocol was selected for being an open standard with a security extension, having a distributed dynamic discovery and being highly customizable for every kind of application.
Several DDS vendors, both open source and licensed, are available within the *ROS 2* environment.
However, *Vulcanexus* supports `eProsima's Fast DDS <https://fast-dds.docs.eprosima.com/en/latest/>`_ open source implementation.
The main advantage being that *Vulcanexus* releases are not tied to a specific *Fast DDS* version which is enforced by the `ROS 2 ROS Enhancement Proposal (REP) 2004 <https://ros.org/reps/rep-2004.html#package-requirements>`_ 1.vi. requirement:

    *Must have a policy that keeps API and ABI stability within a released ROS distribution.*

Consequently, only patch releases that fix bugs without modifying API and ABI are considered for updates in the *ROS 2* environment.
*Vulcanexus*, on the other hand, updates the middleware layer any time *eProsima Fast DDS* releases a new version in order to benefit from the latest features and fixes.
*Vulcanexus* takes charge of releasing a new binary stable distribution that is compatible with the potential ABI breaks so the *Vulcanexus* user has only to update its *Vulcanexus* distro to benefit from the latest *Fast DDS* release.
For instance, *Vulcanexus* provides :ref:`PKCS #11 security support <tutorials_security_pkcs11_pkcs11>` whereas *ROS 2* does not support it (as of September 2022).

Algorithms
^^^^^^^^^^

Another category includes robotic application's algorithms.
*Vulcanexus*, being a *ROS 2* all-in-one tool set, provides the algorithms included in *ROS 2* ecosystem.

Developer tools
^^^^^^^^^^^^^^^

Finally, *ROS 2* provides several development tools for debugging, logging, visualization, introspection, simulation, etc.
*Vulcanexus* adds to *ROS 2* toolset including several tools:

- `eProsima Fast DDS Monitor <https://fast-dds-monitor.readthedocs.io/en/latest/>`_: graphical desktop application to monitor DDS communications when using *eProsima Fast DDS*.
- `eProsima Shapes Demo <https://eprosima-shapes-demo.readthedocs.io/en/latest/index.html>`_: first demo application to understand the most used DDS Quality of Service (QoS) and test DDS and *ROS 2* communication.
- :ref:`micro-ROS <tutorials_micro_introduction_microros>`: default embedded tookit for *ROS 2* and *Vulcanexus*.
- `eProsima DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/index.html>`_: end-user software application enabling the connection of distributed DDS networks.
- `Webots <https://cyberbotics.com/doc/guide/foreword>`_: open-source three-dimensional mobile robot simulator.

.. note::

    This documentation provides several :ref:`tutorials <tutorials_core_ros2_tutorials>` showcasing the capabilities and advantages of *Vulcanexus*

ROS 2 Architecture
------------------

*ROS 2* architecture is based on several abstraction layers.
Developers usually interact exclusively with the client libraries, which expose the core communication APIs.
Each one developed in a specific programming language to leave users freedom to choose the one that best applies to its application.
Below, a common interface, ``rcl``, connects with the ROS MiddleWare (RMW) layer where the esential communication APIs are defined.
Each DDS vendor provides its specific RMW implementation using their own DDS library.
*Vulcanexus*, supporting *eProsima Fast DDS*, constantly updates the ``rmw_fastrtps`` library (Fast DDS previously was know as Fast-RTPS) in order to provide the latest features included in Fast DDS library to the *Vulcanexus* community, instead of having to wait to the next *ROS 2* release.

.. figure:: /rst/figures/intro/vulcanexus_architecture.svg
    :align: center

For more information about *ROS 2*, the user is encouraged to read *ROS 2* documentation which is included within *Vulcanexus* documentation in the following section, especially the :doc:`Concepts <../../ros2_documentation/source/Concepts>` chapter.
