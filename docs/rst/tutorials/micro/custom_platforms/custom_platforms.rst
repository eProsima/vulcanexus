.. _tutorials_micro_custom_platforms:

Integrating micro-ROS
=====================

Integrating micro-ROS in a platform is a process that highly depends on the target platform.
In general, micro-ROS provides ready-to-use solutions for integrating the micro-ROS Client library in multiple platforms and build systems, more information about this micro-ROS modules can be found in :ref:`Build System Components <micro_build_system_components>` section.

This tutorial aims to go beyond those modules and provide some ideas on how to integrate micro-ROS in a new platform.
This task can be divided in two main parts: generating the micro-ROS Client library and header directory and linking it against the target embedded application.

It is important to note that micro-ROS Client library is designed to be platform independent.
This means that the library can be built as a standalone library with the only requirement of using the toolchain and ``libc`` implementation of the target platform.

This tutorial will cover the former topics along these sections:

.. contents:: Table of Contents
    :depth: 3
    :local:
    :backlinks: none


Generating a micro-ROS Client library
-------------------------------------

The micro-ROS Client library, in most cases, is compound of:

- A static library built with an specific toolchain. Normally a ``.a`` file.
- A include folder where all the required headers are located. Normally a ``include`` folder.

Given that, most of common build system tools such as CMake or Make will be able to link against the static library and use the include folder to compile the application.

In order to generate those two components, two approaches are provided: using a micro-ROS tool for generating them or creating a custom script for handling this build.

.. _tutorials_micro_generating_lib_script:

micro-ROS generate_lib script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

    This sections starts from a previously created micro-ROS environment. Check the first steps of `Getting started micro-ROS <https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/getting_started/getting_started.html>`_ for instructions on how to create a micro-ROS environment for embedded platforms.

The ``micro_ros_setup`` tool provides a script for generating and building the micro-ROS Client library according to a specific configuration and build parameters.

The following command will download all the required packages:

.. code-block:: bash

    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

For configuring both the micro-ROS library and the build process a **colcon.meta** file and a **CMake toolchain** are required.

An example of a **colcon.meta** file can be:

.. code-block:: json

  {
      "names": {
          "tracetools": {
              "cmake-args": [
                  "-DTRACETOOLS_DISABLED=ON",
                  "-DTRACETOOLS_STATUS_CHECKING_TOOL=OFF"
              ]
          },
          "rosidl_typesupport": {
              "cmake-args": [
                  "-DROSIDL_TYPESUPPORT_SINGLE_TYPESUPPORT=ON"
              ]
          },
          "rcl": {
              "cmake-args": [
                  "-DBUILD_TESTING=OFF",
                  "-DRCL_COMMAND_LINE_ENABLED=OFF",
                  "-DRCL_LOGGING_ENABLED=OFF"
              ]
          },
          "rcutils": {
              "cmake-args": [
                  "-DENABLE_TESTING=OFF",
                  "-DRCUTILS_NO_FILESYSTEM=ON",
                  "-DRCUTILS_NO_THREAD_SUPPORT=ON",
                  "-DRCUTILS_NO_64_ATOMIC=ON",
                  "-DRCUTILS_AVOID_DYNAMIC_ALLOCATION=ON"
              ]
          },
          "microxrcedds_client": {
              "cmake-args": [
                  "-DUCLIENT_PIC=OFF",
                  "-DUCLIENT_PROFILE_UDP=OFF",
                  "-DUCLIENT_PROFILE_TCP=OFF",
                  "-DUCLIENT_PROFILE_DISCOVERY=OFF",
                  "-DUCLIENT_PROFILE_SERIAL=OFF",
                  "-UCLIENT_PROFILE_STREAM_FRAMING=ON",
                  "-DUCLIENT_PROFILE_CUSTOM_TRANSPORT=ON"
              ]
          },
          "rmw_microxrcedds": {
              "cmake-args": [
                  "-DRMW_UXRCE_MAX_NODES=1",
                  "-DRMW_UXRCE_MAX_PUBLISHERS=5",
                  "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
                  "-DRMW_UXRCE_MAX_SERVICES=1",
                  "-DRMW_UXRCE_MAX_CLIENTS=1",
                  "-DRMW_UXRCE_MAX_HISTORY=4",
                  "-DRMW_UXRCE_TRANSPORT=custom"
              ]
          }
      }
  }

An example of a **CMake toolchain** for a Cortex-M3 platform can be:

.. code-block::

  set(CMAKE_SYSTEM_NAME Generic)
  set(CMAKE_CROSSCOMPILING 1)
  set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

  # SET HERE THE PATH TO YOUR C99 AND C++ COMPILERS
  set(CMAKE_C_COMPILER arm-none-eabi-gcc)
  set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

  set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
  set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

  # SET HERE YOUR BUILDING FLAGS
  set(FLAGS "-O2 -ffunction-sections -fdata-sections -fno-exceptions -mcpu=cortex-m3 -nostdlib -mthumb --param max-inline-insns-single=500 -DF_CPU=84000000L" CACHE STRING "" FORCE)

  set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS_INIT "-std=c++11 ${FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

Once both files are ready, the micro-ROS library can be generated and built using the following command:

.. code-block:: bash

  ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_custom_toolchain.cmake $(pwd)/my_custom_colcon.meta

Creating a custom build script
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A basic understanding on how to proceed can be extracted analyzing the code of the `custom library generation script <https://github.com/micro-ROS/micro_ros_setup/blob/humble/config/generate_lib/generic/build.sh>`_ explained above.

But in general the following points shall be taken into account:

.. note::

    When following this instructions sourcing a ROS 2 environment shall be avoided in order to avoid mixing the ROS 2 build system with the micro-ROS build system.

1. Create a micro-ROS development environment with the following packages in the correct branch:

  - ``ament_cmake`` (`https://github.com/ament/ament_cmake <https://github.com/ament/ament_cmake>`_)
  - ``ament_lint`` (`https://github.com/ament/ament_lint <https://github.com/ament/ament_lint>`_)
  - ``ament_package`` (`https://github.com/ament/ament_package <https://github.com/ament/ament_package>`_)
  - ``googletest`` (`https://github.com/ament/googletest <https://github.com/ament/googletest>`_)
  - ``ament_cmake_ros`` (`https://github.com/ros2/ament_cmake_ros <https://github.com/ros2/ament_cmake_ros>`_)
  - ``ament_index`` (`https://github.com/ament/ament_index <https://github.com/ament/ament_index>`_)


2. Build this environment locally using ``colcon build --cmake-args -DBUILD_TESTING=OFF``
3. Source the local environment using ``source install/local_setup.bash``
4. Create a new micro-ROS workspace and clone the micro-ROS Client packages inside it

  - ``micro-CDR`` (`https://github.com/eProsima/micro-CDR <https://github.com/eProsima/micro-CDR>`_)
  - ``Micro-XRCE-DDS-Client`` (`https://github.com/eProsima/Micro-XRCE-DDS-Client <https://github.com/eProsima/Micro-XRCE-DDS-Client>`_)
  - ``rcl`` (`https://github.com/micro-ROS/rcl <https://github.com/micro-ROS/rcl>`_)
  - ``rclc`` (`https://github.com/ros2/rclc <https://github.com/ros2/rclc>`_)
  - ``rcutils`` (`https://github.com/micro-ROS/rcutils <https://github.com/micro-ROS/rcutils>`_)
  - ``micro_ros_msgs`` (`https://github.com/micro-ROS/micro_ros_msgs <https://github.com/micro-ROS/micro_ros_msgs>`_)
  - ``rmw-microxrcedds`` (`https://github.com/micro-ROS/rmw-microxrcedds <https://github.com/micro-ROS/rmw-microxrcedds>`_)
  - ``rosidl_typesupport`` (`https://github.com/micro-ROS/rosidl_typesupport <https://github.com/micro-ROS/rosidl_typesupport>`_)
  - ``rosidl_typesupport_microxrcedds`` (`https://github.com/micro-ROS/rosidl_typesupport_microxrcedds <https://github.com/micro-ROS/rosidl_typesupport_microxrcedds>`_)
  - ``rosidl`` (`https://github.com/ros2/rosidl <https://github.com/ros2/rosidl>`_)
  - ``rmw`` (`https://github.com/ros2/rmw <https://github.com/ros2/rmw>`_)
  - ``rcl_interfaces`` (`https://github.com/ros2/rcl_interfaces <https://github.com/ros2/rcl_interfaces>`_)
  - ``rosidl_defaults`` (`https://github.com/ros2/rosidl_defaults <https://github.com/ros2/rosidl_defaults>`_)
  - ``unique_identifier_msgs`` (`https://github.com/ros2/unique_identifier_msgs <https://github.com/ros2/unique_identifier_msgs>`_)
  - ``common_interfaces`` (`https://github.com/ros2/common_interfaces <https://github.com/ros2/common_interfaces>`_)
  - ``test_interface_files`` (`https://github.com/ros2/test_interface_files <https://github.com/ros2/test_interface_files>`_)
  - ``rmw_implementation`` (`https://github.com/ros2/rmw_implementation <https://github.com/ros2/rmw_implementation>`_)
  - ``rcl_logging`` (`https://github.com/ros2/rcl_logging <https://github.com/ros2/rcl_logging>`_)
  - ``ros2_tracing`` (`https://gitlab.com/ros_tracing/ros2_tracing <https://gitlab.com/ros_tracing/ros2_tracing>`_)
  - ``micro_ros_utilities`` (`https://github.com/micro-ROS/micro_ros_utilities <https://github.com/micro-ROS/micro_ros_utilities>`_)

5. Make sure that the following packages are removed or ignored:

  - ``rosidl/rosidl_typesupport_introspection_cpp``
  - ``rcl_logging/rcl_logging_log4cxx``
  - ``rcl_logging/rcl_logging_spdlog``
  - ``rclc/rclc_examples``
  - ``rcl/rcl_yaml_param_parser``

6. Build the micro-ROS workspace using colcon, your required ``colcon.meta`` and your custom compiler flags using a CMake toolchain file ``my_toolchain.cmake``:

.. code-block::

  colcon build \
      --merge-install \
      --packages-ignore-regex=.*_cpp \
      --metas my_colcon.meta \
      --cmake-force-configure \
      --cmake-clean-cache \
      --cmake-args \
      "--no-warn-unused-cli" \
      --log-level=ERROR \
      -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
      -DTHIRDPARTY=ON \
      -DBUILD_SHARED_LIBS=OFF \
      -DBUILD_TESTING=OFF \
      -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
      -DCMAKE_TOOLCHAIN_FILE=my_toolchain.cmake \
      -DCMAKE_VERBOSE_MAKEFILE=ON;

7. (Optional) Merge the generated ``.a`` libraries using `ar <https://man7.org/linux/man-pages/man1/ar.1.html>`_ utility.

Integrating a custom build system
---------------------------------

At this point, the generated ``.a`` and ``include`` folder generated in the previous step shall be linked against a micro-ROS application.

Some approaches for integrating micro-ROS Client library on a platform build system can be:

Bare gcc approach
^^^^^^^^^^^^^^^^^

Using a common ``gcc`` command line, the following steps can be followed:

.. code-block::

    [TRIPLET PREFIX]-gcc -o microros_app.elf [COMPILER FLAGS] -I[MICROROS INCLUDE FOLDER] main.c libmicroros.a

.. note::

    Note that ``[COMPILER FLAGS]`` shall be the same when generating the micro-ROS Client library and when building the micro-ROS application.

Makefile
^^^^^^^^

An example on how to integrate micro-ROS Client library in a Make build system can be found in the `micro-ROS app for TI Tiva C Series <https://github.com/micro-ROS/micro_ros_tivac_launchpad_app/blob/humble/Makefile>`_.

CMake
^^^^^

An example on how to integrate micro-ROS Client library in a CMake build system can be found in the `micro-ROS example for Mbed RTOS <https://github.com/micro-ROS/micro_ros_mbed/blob/humble/CMakeLists.txt>`_.


micro-ROS system dependencies
-----------------------------

There are three points where micro-ROS Client library needs to use functionality of the target platform beyond the ``libc`` implementation:

- Obtaining a time reference
- Configuring the transport layer
- Dealing with memory allocation

Time reference
^^^^^^^^^^^^^^

In order to operate in a time-based approach, the micro-ROS library will need at link time an implementation of the function ``int clock_gettime(clockid_t, struct timespec *)`` from the `POSIX specification <https://man7.org/linux/man-pages/man3/clock_gettime.3.html>`_ .

This function will need to fill ``struct timespec *`` input argument implement with a monotonic time reference.
In the case that the target platform does not provide this function, it is possible to implement it at application level and let the linker to resolve the symbol when linking the micro-ROS Client library.

A reference implementation can be:

.. code-block:: c

  #include <sys/time.h>

  #define USEC_IN_SEC 1000000
  #define NSEC_IN_USEC 1000

  int clock_gettime(clockid_t clock_id, struct timespec *tp)
  {
    (void) clock_id;

    uint64_t microseconds_elapsed = my_platform_get_microseconds();

    // Handle here possible rollovers of your platform timers if required.

    tp->tv_sec = microseconds_elapsed / USEC_IN_SEC;
    tp->tv_nsec = (microseconds_elapsed % USEC_IN_SEC) * NSEC_IN_USEC;

    return 0;
  }

Transport layer
^^^^^^^^^^^^^^^

micro-ROS Client will need a transport implementation for communication with the micro-ROS Agent.

Details on how to implement this transports can be found in :ref:`Custom Transports tutorial <tutorials_micro_custom_transports>`.

.. note::

    This tutorial explains how to implement custom transports in both micro-ROS Client and Agent side.
    In the most common use case both parts are communicated using serial ports or UDP sockets.
    That means that only Client side transport shall be implemented and the Agent side transport can be used as it is provided.

.. _tutorials_micro_custom_platforms_allocators:

Allocators
^^^^^^^^^^

More details about micro-ROS allocators are provided at :ref:`Memory management allocators tutorial <tutorials_micro_memory_management_allocators>`.

