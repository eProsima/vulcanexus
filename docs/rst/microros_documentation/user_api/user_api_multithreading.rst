.. _tutorials_micro_multithreading:

Multithreading
==============

.. contents:: Table of Contents
    :depth: 1
    :local:
    :backlinks: none

Multithreading support can be enabled at compilation time, which enables the micro-ROS default middleware (Micro XRCE-DDS) to be thread safe under specific circumstances.

With this feature, micro-ROS entities such as publishers, subscribers or services can run on different RTOS execution threads, with the improvements on performance, usability and resource optimization that this implies.

Ready to use code related to this concept can be found on micro-ROS demos repository `multithread_publisher_subscriber <https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/multithread_publisher_subscriber/main.c>`_ example.

.. TODO(acuadros95): Add explanation on mutex protection levels (Stream, transport, session?)

.. note::

   This feature is not supported on micro-ROS experimental middleware `embeddedRTPS <https://discourse.ros.org/t/embeddedrtps-the-new-experimental-middleware-for-micro-ros/22741>`_.

Configuration
-------------

This option is disabled by default and needs to be enabled at compile time via ``UCLIENT_PROFILE_MULTITHREAD`` CMake argument on the middleware package (``microxrcedds_client``).

An example ``colcon.meta`` is:

.. code-block:: json

  {
      "names": {
          "microxrcedds_client": {
              "cmake-args": [
                  "-DUCLIENT_PROFILE_MULTITHREAD=ON"
              ]
          }
      }
  }

Supported platforms
^^^^^^^^^^^^^^^^^^^

This functionality is not available on all platforms, as specific Mutex implementations are provided by each system. Support for the following platforms is provided:

.. TODO(acuadros95): Add some explanation, link to build system or whatever here

- FreeRTOS
- Zephyr
- Posix

For other platforms, a compilation error will be triggered with the following message: ``XRCE multithreading not supported for this platform.``

API usage
---------

This section will explain the limitations of this approach, offering design tips to overcome them.

Restrictions
^^^^^^^^^^^^

1. **Entity creation and destruction methods are not thread safe**: Any other micro-ROS related execution shall be halted until this steps are completed.
2. The :ref:`parameter server API <micro_user_api_parameter_server>` is not thread safe, this include modifications coming from a triggered callback.
3. The executor callbacks will run within the same thread where the executor is spinned.

Architecture tips
^^^^^^^^^^^^^^^^^

1. Initialization and destruction of entities:

  - Have a initial micro-ROS thread with initialization and destruction steps for all entities.
  - Use the approach shown on the :ref:`Ping API <tutorials_micro_handle_reconnections_ping>` tutorial to block other threads until the micro-ROS entities are ready.

2. Callback dispatching:

  - Executor callbacks can be distributed on multiple threads by using a unique executor instance per thread.
  - This means that its expected to have a executor instance for each thread where callbacks shall be processed.

3. Publishers are thread safe and can be called from multiple threads as long as the publisher object is not destroyed.
4. Parameter server local API shall be used within the same thread as its related executor is spinned.
5. Action servers goal execution is expected to run on its own thread, being ``rclc_action_send_result`` thread safe.
