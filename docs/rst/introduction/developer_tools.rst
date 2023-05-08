.. _vulcanexus_developer_tools:

Developer Tools
===============

*ROS 2* provides several development tools for debugging, logging, visualization, introspection, simulation, etc.
*Vulcanexus* adds to *ROS 2* toolset the following tools:

.. toctree::
   :titlesonly:
   :maxdepth: 1
   :hidden:

   tools/monitor.rst
   tools/backend.rst
   tools/shapes_demo.rst
   tools/router.rst
   tools/webots.rst

- :ref:`ROS 2 QoS Profiles Manager <vulcanexus_qos_profiles_manager>`: tool suite to generate XML configuration files for Vulcanexus middleware Fast DDS.
- :ref:`ROS 2 Monitor <vulcanexus_monitor>`: graphical desktop application to monitor *ROS 2* communications.
- :ref:`Fast DDS Statistics Backend <fastdds_statistics_backend>`: *ROS 2 Monitor*'s backend. Can be leveraged with other monitoring frontend applications (e.g. :ref:`Prometheus <tutorials_tools_prometheus>`).
- :ref:`ROS 2 Shapes Demo <vulcanexus_shapes_demo>`: first demo application to understand the most used *ROS 2* Quality of Service (QoS) and test DDS and *ROS 2* communication.
- :ref:`ROS 2 Router <vulcanexus_router>`: end-user software application enabling the connection of distributed *ROS 2* environments.
- :ref:`Webots <webots>`: open-source three-dimensional mobile robot simulator.

.. note::

    This documentation provides several :ref:`tutorials <tutorials_core_ros2_tutorials>` showcasing the capabilities and advantages of *Vulcanexus*
