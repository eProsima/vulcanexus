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

Getting started
^^^^^^^^^^^^^^^

* :ref:`Installation Manual <linux_binary_installation>`

  - Instructions to set up ROS 2 with Vulcanexus for the first time.

* :ref:`Overview <vulcanexus_global_introduction>`

  - *Vulcanexus* concepts including *ROS 2* Documentation.

* :ref:`Tutorials <tutorials_core_ros2_tutorials>`

  - Collection of step-by-step instructions that help users to get started.

* :ref:`Support <platforms>`

  - Vulcanexus officially supported platforms and releases.

* :ref:`Appendixes <glossary>`

  - Glossary of terms used throughout this documentation.


Vulcanexus project
^^^^^^^^^^^^^^^^^^

*Vulcanexus* is composed of a collection of downloadable packages:

1. **Vulcanexus Core**: a set of software libraries that enables users to build the most comprehensive and straightforward robotics application.
   It consists of *eProsima Fast DDS* latest version as the *ROS 2* middleware.
2. **Vulcanexus Tools**: a set of features and applications which allows users to test, improve and configure the performance of *Vulcanexus* in their systems.
3. **Vulcanexus Micro**: provides access for resource constrained devices (micro-controllers) to the DDS world, bridging the gap between them and *ROS 2*.
4. **Vulcanexus Cloud**: scales and integrates *ROS 2* networks located in geographically spaced environments, and enables the deployment of DDS entities in the cloud in a quick and easy way.
5. **Vulcanexus Simulation**: enables users to design robotic simulations, providing an end-to-end development environment to model, program, and simulate robots.
6. **Vulcanexus Base**: includes within the same installation *Vulcanexus* tools, micro and cloud features.
7. **Vulcanexus Desktop**: includes *ROS 2* desktop installation as well as every available *Vulcanexus* package.
   It is the more complete *Vulcanexus* installation.

The table below shows the components included in every one of the previously explained packages:

.. figure:: /rst/figures/intro/vulcanexus_packages_table.png

Contacts and Commercial support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Find more about us at `Vulcanexus webpage <https://vulcanexus.org/>`__.

Support available at:

* Email: support@eprosima.com
* Phone: +34 91 804 34 48

Contributing to the documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Vulcanexus Docs* is an open source project, and as such all contributions, both in the form of feedback and content
generation, are most welcomed.
To make such contributions, please refer to the
`Contribution Guidelines <https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md>`_ hosted in our GitHub repository.

.. toctree::
   :caption: Installation Manual
   :maxdepth: 2
   :hidden:
   :numbered: 5

   /rst/installation/linux_binary_installation.rst
   /rst/installation/linux_source_installation.rst
   /rst/installation/docker.rst

.. toctree::
   :caption: Overview
   :maxdepth: 2
   :hidden:
   :numbered: 5

   /rst/introduction/introduction.rst
   /rst/introduction/developer_tools.rst
   /rst/introduction/metapackages.rst
   /ros2_documentation/source/index.rst
   /rst/microros_documentation/index.rst

.. toctree::
   :caption: Tutorials
   :maxdepth: 2
   :hidden:
   :numbered: 5

   /rst/tutorials/core/ros2_tutorials
   /rst/tutorials/tools/tools_tutorials
   /rst/tutorials/cloud/cloud_tutorials
   /rst/tutorials/micro/micro_tutorials

.. toctree::
   :caption: Support
   :maxdepth: 2
   :hidden:
   :numbered: 5

   /rst/platforms/platforms
   /rst/notes/notes


.. _index_appendix:

.. toctree::
   :caption: Appendixes
   :maxdepth: 2
   :hidden:

   /rst/appendixes/glossary
