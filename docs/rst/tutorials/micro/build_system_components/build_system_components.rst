.. _tutorials_micro_build_system_components:


Build System Components
=======================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


.. figure:: /rst/figures/tutorials/micro/platforms_logos/renesas_e2_studio.png
    :align: center

.. _micro_ros_renesas_e2_studio:

Renesas e\ :sup:`2` Studio
^^^^^^^^^^^^^^^^^^^^^^^^^^

`Renesas RA Family <https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus>`_ is the **official supported hardware** of micro-ROS.
This family of MCU features a wide range of features based on Arm® Cortex®-M33, M23, and M4 processor cores.

Renesas e\ :sup:`2` Studio provides a whole integrated development environment that allows for professional development, test and profiling of embedded applications. The micro-ROS component for Renesas provides a set of tools and instructions to integrate the build and configuration of micro-ROS inside of a Renesas e\ :sup:`2` Studio project.

This packages supports multiple RTOSes (FreeRTOS and Azure RTOS), as well as a bare-metal approach. Also, a wide range of transport layer are available: UART, UDP/IP, TCP/IP, USB-CDC and CAN/FD.

.. note::

    For detailed instructions on how to use micro-ROS with this platform, please refer to `micro-ROS for Renesas e2 Studio <https://github.com/micro-ROS/micro_ros_renesas2estudio_component>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component renesas_e2_studio

.. figure:: /rst/figures/tutorials/micro/platforms_logos/esp_idf.png
    :align: center

.. _micro_ros_esp_idf:

Espressif ESP-IDF
^^^^^^^^^^^^^^^^^
micro-ROS component for `ESP-IDF <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/>`_ allows the integration of the micro-ROS stack in one of the most famous **WiFI-enabled MCUs**: the ESP32 family.

By means of this component, it is easy integrate, configure and deploy a micro-ROS application that can communicate with the ROS 2 dataspace over WiFi, UART or USB-CDC. By default, this toolchain integrates FreeRTOS as framework and allows the user to run micro-ROS tasks simultaneously along with other user process.

Most of the ESP32 versions are supported: ESP32, ESP32-S2, ESP32-S3 and even the RISC-V based MCU of Espressif, the ESP32-C3.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS component for ESP-IDF <https://github.com/micro-ROS/micro_ros_espidf_component>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component esp_idf

.. figure:: /rst/figures/tutorials/micro/platforms_logos/zephyr_rtos.png
    :align: center

.. _micro_ros_zephyr_rtos:

Zephyr RTOS
^^^^^^^^^^^

`Zephyr RTOS <https://docs.zephyrproject.org/>`_ is an RTOS for embedded systems **supported by the Linux Foundation**. It provides a full featured and layered architectured RTOS ready for deploy production-ready applications. Thanks to the huge amount of ready-to-use drivers, it will be easy to integrate connectivity solutions such as Bluetooth BLE, WiFi, USB or CAN.

With the micro-ROS module for Zephyr RTOS, ROS 2 users will find easy to integrate an embedded micro-ROS application in the Zephyr ecosystem. This module support a considerable part of the huge amount of `supported boards by Zephyr <https://docs.zephyrproject.org/latest/boards/index.html>`_.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS module for Zephyr <https://github.com/micro-ROS/micro_ros_zephyr_module>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component zephyr_rtos

.. figure:: /rst/figures/tutorials/micro/platforms_logos/mbed_rtos.png
    :align: center

.. _micro_ros_mbed_rtos:

ARM MBed RTOS
^^^^^^^^^^^^^

micro-ROS provides a module for integration ROS 2 embedded application in `ARM MBed RTOS <https://os.mbed.com/>`_. A basic UART transport is provided for version v6.8 and beyond.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS module for Mbed RTOS <https://github.com/micro-ROS/micro_ros_mbed>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component mbed_rtos

.. figure:: /rst/figures/tutorials/micro/platforms_logos/nuttx_rtos.png
    :align: center

.. _micro_ros_nuttx_rtos:

NuttX RTOS
^^^^^^^^^^

`NuttX <https://nuttx.apache.org/>`_ is one of the most complete and mature RTOS available. It is **supported by Apache Foundation** and provides a piece of software with emphasis on standard compliance and small footprint. It is a powerful and flexible RTOS that can be used for embedded applications. Also, it is compliant with POSIX and ANSI standards.

micro-ROS app for NuttX provides an example application environment where the micro-ROS stack is integrated in the NuttX 10 build system and can be ran as a NuttX application with an UART transport.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS app for Nuttx RTOS <https://github.com/micro-ROS/micro_ros_nuttx_app>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component nuttx_rtos

.. figure:: /rst/figures/tutorials/micro/platforms_logos/azure_rtos.png
    :align: center

.. _micro_ros_azure_rtos:

Microsoft Azure RTOS
^^^^^^^^^^^^^^^^^^^^

`Azure RTOS <https://azure.microsoft.com/en-us/services/rtos/>`_ is the **embedded IoT development environment of Microsoft**. It provides a huge amount of solutions and ready-to-use libraries with focus in cloud applications and connectivity.

By means of this module, micro-ROS is integrated in ThreadX (the Azure RTOS scheduler), using NetX (the Azure RTOS network stack) to provide a UDP/IP transport.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS app for Microsoft Azure RTOS <https://github.com/micro-ROS/micro_ros_azure_rtos_app>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component azure_rtos

.. figure:: /rst/figures/tutorials/micro/platforms_logos/tiva_c_series.png
    :align: center

.. _micro_ros_tiva_c_series:

TI Tiva C Series
^^^^^^^^^^^^^^^^

Texas Instruments Tiva C Series is a family of MCU based on ARM Cortex-M4F. micro-ROS provides support for this platform and a basic USB-CDC transport.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS app for TI Tiva™ C Series TM4C123G <https://github.com/micro-ROS/micro_ros_tivac_launchpad_app>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component tiva_c_series

.. figure:: /rst/figures/tutorials/micro/platforms_logos/stm32cube.png
    :align: center

.. _micro_ros_stm32cube:

ST Micro ST Cube IDE/MX
^^^^^^^^^^^^^^^^^^^^^^^

`STM32Cube <https://www.st.com/en/ecosystems/stm32cube.html>`_ is one of the **preferred options for using the ST Micro STM32 family**. By means of this module, the micro-ROS user will find easy to integrate the micro-ROS stack in a STM32Cube project.

All the STM32 MCUs are supported by this IDE and most of them should be able to run the micro-ROS stack. By now, multiple Nucleo boards featuring STM32F4 and STM42F7 devices has been tested a proven to work. By using this module the micro-ROS user will find it easy to port micro-ROS to its own STM32 with FreeRTOS and the provided serial based transport.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS for STM32CubeMX/IDE <https://github.com/micro-ROS/micro_ros_stm32cubemx_utils>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component stm32cube

.. figure:: /rst/figures/tutorials/micro/platforms_logos/platformio.png
    :align: center

.. _micro_ros_platformio:

Platform.IO
^^^^^^^^^^^

`Platform.IO <https://platformio.org/>`_ is one of the **most popular collaborative platform for embedded development**. It provides a large set of supported platforms, frameworks and libraries for embedded development. All of this based with an intuitive configuration procedure and an automated toolchain installation.

By using this module, micro-ROS user will be able to integrate micro-ROS in their Platform.IO projects just by including a couple of configuration lines. Some of the out-of-the-box supported transport are: UART, WiFi and Ethernet. Also, the micro-ROS user will find easy to port micro-ROS to the large list of `supported platforms <https://registry.platformio.org/search?t=platform>`_.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS for PlatformIO <https://github.com/micro-ROS/micro_ros_platformio>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component platformio

.. figure:: /rst/figures/tutorials/micro/platforms_logos/arduino.png
    :align: center

.. _micro_ros_arduino:

Arduino
^^^^^^^

`Arduino <https://www.arduino.cc/>`_ is the reference development framework to introduce new users to the embedded world. Using their **user-friendly IDE and tools**, Arduino users can easily go from their first blinky LED to the their own micro-ROS applications.

micro-ROS for Arduino provides a set of prebuilt libraries for reference platforms so the user do not have to handle the micro-ROS build procedure. Also, for advanced users, instructions for customizing the installation and recompiling the library are provided.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS for Arduino <https://github.com/micro-ROS/micro_ros_arduino>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component arduino


.. figure:: /rst/figures/tutorials/micro/platforms_logos/raspberry_pi_pico.png
    :align: center

.. _micro_ros_raspberry_pi_pico:

Raspberry Pi Pico
^^^^^^^^^^^^^^^^^

`Raspberry Pi Pico <https://www.raspberrypi.com/products/raspberry-pi-pico/>`_ is the versatile embedded and super low cost solution of **Raspberry Pi for microcontroller environments**. It features a dual core Cortex-M0+ based silicon named RP2040, which include a wide variety of peripherals such as I2C, SPI, UART or GPIO, and even a hardware programmable module named PIO. Also a full featured API for hardware abstraction are provided within the SDK.

micro-ROS module for Raspberry Pi Pico SDK provides a set of prebuilt libraries so the user do not have to handle the micro-ROS build procedure. Also, for advanced users, instructions for customizing the installation and recompiling the library are provided.

.. note::

    For detailed instructions on how to use micro-ROS with this platform visit `micro-ROS for Raspberry Pi Pico <https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk>`_ Github repository or use your Vulcanexus distribution to download this module:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        ros2 run micro_ros_setup component raspberry_pi_pico
