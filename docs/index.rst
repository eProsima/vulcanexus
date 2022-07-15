.. _vulcanexus_introduction:

Vulcanexus Documentation
========================

.. image:: /rst/figures/logo.png
  :height: 100px
  :width: 100px
  :align: left
  :alt: Vulcanexus
  :target: http://www.vulcanexus.com/


*Vulcanexus* is a *ROS 2* (Robot Operating System) all-in-one tool set.
It allows users to build robotics applications combining the unique *Vulcanexus* elements with the *ROS 2* libraries, having *Fast DDS* as its fixed middleware implementation.

These open source elements include numerous features and tools, providing *Vulcanexus* users customizable solutions while improving overall system performance.
With *Vulcanexus*, users have fast access to constantly improving functionalities, such as the latest *Fast DDS* version along with its new features.

*Vulcanexus* combinable elements are:

1. **VULCANEXUS-CORE**: a set of software libraries that enables users to build the most comprehensive and straightforward robotics application.
   It consists of eProsima *Fast DDS* and *ROS 2*.
2. **VULCANEXUS-TOOLS**: a set of features and applications which allows users to test, improve and configure the performance of *Vulcanexus* in their systems.
3. **VULCANEXUS-MICRO**: provides access for resource constrained devices (micro-controllers) to the DDS world, bridging the gap between them and *ROS 2*.
4. **VULCANEXUS-CLOUD**: scales and integrates *ROS 2* networks located in geographically spaced environments, and enables the deployment of DDS entities in the cloud in a quick and easy way.
5. **VULCANEXUS-SIMULATION**: enables users to design robotic simulations, providing an end-to-end development environment to model, program, and simulate robots.

*Vulcanexus* created a collection of downloadable packages that include useful combinations of the previously described elements with *ROS 2*:

.. figure:: /rst/figures/intro/vulcanexus_packages_table.svg

The following documentation includes instructions for installing each *Vulcanexus* packages, some tutorials help users to get started, and the supported platforms and releases.

.. toctree::
   :caption: Installation Manual
   :maxdepth: 2
   :hidden:
   :numbered: 5

   /rst/installation/linux_binary_installation.rst
   /rst/installation/linux_source_installation.rst
   /rst/installation/docker.rst

.. toctree::
   :caption: Tutorials
   :maxdepth: 2
   :hidden:
   :numbered: 5

   /rst/tutorials/tools_tutorial.rst
   /rst/tutorials/cloud/cloud_tutorial.rst
   /rst/tutorials/cloud/wan_tcp/wan_tcp.rst
   /rst/tutorials/microros_tutorial.rst

.. toctree::
   :caption: Support
   :maxdepth: 2
   :hidden:
   :numbered: 5

   /rst/platforms/platforms
   /rst/notes/notes
