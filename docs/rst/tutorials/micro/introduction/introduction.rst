.. _tutorials_micro_introduction:

.. figure:: /rst/figures/tutorials/micro/biglogo.png
    :width: 500px
    :align: center

Introduction to micro-ROS
==========================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none

Overview
--------

**micro-ROS** is the default embedded toolkit for ROS 2 and Vulcanexus.

It provides a complete set of tools, frameworks and APIs for deploying application in *microcontrollers (MCU)* with ROS 2 concepts: nodes, publisher, subscribers and much more.
The micro-ROS target MCUs are 32 bit architectures with more than 32 kB of RAM. It can be run on top of an **RTOS** or even in a **bare metal** approach.

micro-ROS aims to be as **lightweight** as possible, while keeping it **configurable**. This way, it is possible to tune tons of parameters in terms of memory or timing to fit user application requirements.

micro-ROS targets **hard real-time** environments, so:

* user has control of the the memory usage, achieving **zero dynamic memory usage** at runtime
* user has control of the **timing** of each operation so they are **deterministic**
* user has control of the **execution order** of the operations

micro-ROS is **transport agnostic**. With its default middleware (`eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_) it is possible to run micro-ROS on top of almost any transport layer that supports a package or stream communication paradigm: TCP/IP or UDP/IP stacks, UART devices, CAN/FD buses, SPI, radio links, etc.

Architecture
^^^^^^^^^^^^

micro-ROS follows an architecture that is based on ROS 2, but also has significant differences that allows the embedded integration. In order to ease the use, it also provide tools for handling the integration in embedded platforms.

micro-ROS **layered architecture** abstracts the OS/RTOS with a middleware layer. On top of the middleware, micro-ROS uses the same ROS 2 core layers (RMW, RCL and RCUTILS). With respect to user APIs, micro-ROS provides a **C99 API named RCLC** that provides the concepts available on ROS 2 C++ or Python APIs (RCLCPP or RCLPY).

micro-ROS is **compatible with the ROS 2 type support**, that means that any message definition used in ROS 2 can be used seamlessly in micro-ROS. By means of the micro-ROS build systems, any `.msg`, `.srv` or `.action` definition file can be integrated in the micro-ROS build in order to achieve a type interoperability with ROS 2.

Middleware
^^^^^^^^^^

In the case of micro-ROS the default middleware layer is `eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_, a **lightweight**, **extremely portable**,  with **minimal overhead** and `OMG <https://www.omg.org/spec/DDS-XRCE/1.0/About-DDS-XRCE/>`_ standard-based.

The micro-ROS default middleware is compound of two parts: the **Micro XRCE-DDS Client** that runs in the MCU side and the **Micro XRCE-DDS Agent** that runs in the ROS 2 side and communicates the embedded side with the ROS 2 dataspace. The `eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_ is the base package for the `micro-ROS Agent <https://github.com/micro-ROS/micro-ROS-Agent>`_, a package that extends the former with specific ROS 2 features such as graph management.

`micro-ROS Agent <https://github.com/micro-ROS/micro-ROS-Agent>`_ is distributed along with `Vulcanexus Micro <https://docs.vulcanexus.org/en/latest/>`_. Check the `installation instruction <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_.

In the same way as ROS 2, micro-ROS has a **pluggable middleware interface** so other middleware implementations can be used at the bottom layer of the stack.


.. figure:: /rst/figures/tutorials/micro/microros_stack.png
    :align: center


Build Systems
^^^^^^^^^^^^^

micro-ROS provides a set of tools for integrating it in many build systems, toolchains and embedded development frameworks. Those packages are heterogeneous due to the diversity of tools used by different vendors.

In general, micro-ROS provides standalone solutions for most used RTOSes and environments such as: :ref:`Zephyr RTOS <micro_ros_zephyr_rtos>`, :ref:`Espressif ESP-IDF <micro_ros_esp_idf>`, FreeRTOS, :ref:`Microsoft Azure RTOS <micro_ros_azure_rtos>`, etc. But also provides packages for integrating it in major vendor tools: :ref:`Renesas e2 Studio<micro_ros_renesas_e2_studio>` or :ref:`STM32 Cube IDE/MX <micro_ros_stm32cube>`.

Finally ``micro_ros_setup`` is a package that provides simple scripts for navigating the micro-ROS support packages. ``micro_ros_setup`` is distributed along with `Vulcanexus Micro <https://docs.vulcanexus.org/en/latest/>`_. Check the `installation instruction <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_.

.. note::

    For the full understanding of this build system approach visit :ref:`Build System Components reference<tutorials_micro_build_system_components>`.
