.. _vulcanexus_microros:

.. figure:: /rst/figures/micro/biglogo.png
    :width: 500px
    :align: center

micro-ROS Documentation
=======================

**micro-ROS** is the default embedded toolkit for ROS 2 and Vulcanexus.
It provides a complete set of tools, frameworks and APIs for deploying applications in *microcontrollers (MCU)* with full support for ROS 2 communications paradigms: Nodes, Publication/Subscription, Services, etc.

micro-ROS targets MCUs with 32 bit architectures and at least 32 kB of RAM, providing support for most popular **RTOSes** and **bare metal** systems.
With this in mind, micro-ROS aims to be as **lightweight** and **versatile** as possible, with a wide set of configurable parameters to tune the application memory usage and timing requirements.

micro-ROS also targets **hard real-time** environments, giving the user control over:

* Memory usage, achieving **zero dynamic memory usage** at runtime.
* **Timing** of each operation to achieve **deterministic** behavior.
* **Execution order** of the operations.

micro-ROS is **transport agnostic**.
With its default middleware (`eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_) it is possible to run micro-ROS on top of almost any transport layer that supports a package or stream communication paradigm: TCP/IP or UDP/IP stacks, UART devices, CAN/FD buses, SPI, radio links, etc.


This documentation is structured as follows:

.. toctree::
    :maxdepth: 1

    getting_started/getting_started.rst
    build_system_components/build_system_components.rst
    user_api/user_api.rst

Tutorials about micro-ROS can be found at :ref:`micro-ROS tutorials <tutorials_micro_micro_tutorials>`.

.. raw:: html

   <h2>Architecture</h2>


micro-ROS follows an architecture that is based on ROS 2, but also has significant differences that allows the embedded integration.
In order to ease the use, it also provides tools for handling the integration in embedded platforms.

micro-ROS **layered architecture** abstracts the OS/RTOS with a middleware layer.
On top of the middleware, micro-ROS uses the same ROS 2 core layers (RMW, RCL and RCUTILS).
With respect to user APIs, micro-ROS provides a **C99 API named RCLC** that allows using all the concepts available on ROS 2 C++ or Python APIs (RCLCPP or RCLPY).

**Compatibility with ROS 2 type support** is also granted, meaning that any message definition used in ROS 2 can be used seamlessly in micro-ROS.
By means of the micro-ROS build systems, any `.msg`, `.srv` or `.action` definition file can be integrated in the micro-ROS build in order to achieve a type interoperability with ROS 2.

.. raw:: html

   <h2>Middleware</h2>

In the case of micro-ROS, the default middleware layer is `eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_, a **lightweight**, **portable**,  with **minimal overhead** middleware based on the `OMG DDS-XRCE <https://www.omg.org/spec/DDS-XRCE/1.0/About-DDS-XRCE/>`_ standard.

The micro-ROS default middleware is compound of two parts: the **Micro XRCE-DDS Client** that runs in the MCU side, and the **Micro XRCE-DDS Agent** that runs in the ROS 2 side and communicates the embedded side with the ROS 2 dataspace.
The `eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_ is the base package for the `micro-ROS Agent <https://github.com/micro-ROS/micro-ROS-Agent>`_, a package that extends the former with specific ROS 2 features such as graph management.

.. note::

    `micro-ROS Agent <https://github.com/micro-ROS/micro-ROS-Agent>`_ is distributed along with `Vulcanexus Micro <https://docs.vulcanexus.org/en/latest/>`_.
    Check the `installation tutorial <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_.

In the same way as ROS 2, micro-ROS has a **pluggable middleware interface** so other middleware implementations can be used at the bottom layer of the stack.


.. figure:: /rst/figures/micro/microros_stack.png
    :align: center

.. raw:: html

   <h2>Build Systems</h2>

micro-ROS provides a set of tools for integrating it in many build systems, toolchains and embedded development frameworks.
Those packages are heterogeneous due to the diversity of tools used by different vendors.

In general, micro-ROS provides standalone solutions for most used RTOSes and environments such as: :ref:`Zephyr RTOS <micro_ros_zephyr_rtos>`, :ref:`Espressif ESP-IDF <micro_ros_esp_idf>`, FreeRTOS, :ref:`Microsoft Azure RTOS <micro_ros_azure_rtos>`, etc.
It also provides packages for integrating it in major vendor tools: :ref:`Renesas e2 Studio<micro_ros_renesas_e2_studio>` or :ref:`STM32 Cube IDE/MX <micro_ros_stm32cube>`.

Finally ``micro_ros_setup`` is a package that provides simple scripts for navigating the micro-ROS support packages.
``micro_ros_setup`` is distributed along with `Vulcanexus Micro <https://docs.vulcanexus.org/en/latest/>`_.
Check the `installation tutorial <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_.

.. note::

    For the full understanding of this build system approach please refer to the :ref:`Build System Components reference<micro_build_system_components>`.

