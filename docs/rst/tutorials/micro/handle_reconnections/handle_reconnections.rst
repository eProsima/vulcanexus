.. _tutorials_micro_handle_reconnections:

Reconnections and liveliness
============================

.. contents:: Table of Contents
    :depth: 1
    :local:
    :backlinks: none

This tutorial aims at providing insight on mechanism for handling reconnections between the micro-ROS agent and the client and assert the connection liveliness.

This includes temporal transport faults, agent or client restarts, client plug and play, etc.

Client side: Ping API
---------------------

Reconnections can be handle manually on the client side using the agent ping functionality with the following sequence:

1. Wait until agent is reachable.
2. Create required micro-ROS entities.
3. Use the micro-ROS API as usual: spin entities, publish, etc.
4. When a failure is detected, destroy the created entities and go back to the first step.

The following code shows an example of this sequence:

.. code-block:: c

    // micro-ROS loop
    while (true)
    {
        // Timeout for each ping attempt
        const int timeout_ms = 100;

        // Number of ping attempts
        const uint8_t attempts = 1;

        // Wait for agent connection
        while(RMW_RET_OK != rmw_uros_ping_agent(timeout_ms, attempts))
        {
            // sleep, handle other task, etc.
        }

        // Create micro-ROS entities
        create_entities();

        // Spin period
        const unsigned int spin_timeout = RCL_MS_TO_NS(100);

        // Spin until connection fails
        while(RMW_RET_OK != rclc_executor_spin_some(&executor, spin_timeout))
        {
            // sleep, handle other task, etc.
        }

        // Connection is lost, destroy entities and go back to first step
        destroy_entities();
    }

This approach allows a micro-ROS client to handle agent restarts or to follow a plug and play approach, where the micro-ROS app will only run when the agent connection is available.

A full example can be found on micro-ROS for Arduino repository `micro-ros_reconnection <https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino>`_ example.

.. note::
    Details on the Ping API usage can be found in the :ref:`Middleware API: Ping agent <tutorials_micro_user_middleware_ping>` tutorial.

Agent side: Hard liveliness check
---------------------------------

The main problem with the previous section's method is that entity destruction always happens on micro-ROS client's request. This implies that other ROS 2 entities will not be aware of the micro-ROS client destruction.

The Hard liveliness check mechanism allows the micro-ROS agent to ping the client periodically. This way, the agent will take care of ensuring that the micro-ROS client is alive and will destroy the created entities if a certain timeout happens without any response from the client side. This means that the nodes, publishers, subscribers (and any other entity) created by the client will be removed from the ROS 2 graph.

Note that the client shall also be aware of the disconnection to create the micro-ROS entities again, this can be achieved by including the previous section approach.

.. note::
    The micro-ROS client shall spin an executor to give a response to the agent liveliness check messages, a empty executor can be used for this purpose.

Configuration
^^^^^^^^^^^^^

This feature is enabled by default in the micro-ROS Agent and **must be enabled** by means of `colcon.meta` parameters in the micro-ROS client:

    - ``UCLIENT_HARD_LIVELINESS_CHECK``: Enable hard liveliness check
    - ``UCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT``: Configure connection timeout in milliseconds (Default value: 10000).

Example configuration on `colcon.meta` file:

.. code-block:: python

    # colcon.meta example with Hard Liveliness Check configuration
    {
        "names": {
            "microxrcedds_client": {
                "cmake-args": [
                    "-DUCLIENT_HARD_LIVELINESS_CHECK=ON",
                    "-DUCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT=5000"
                ]
            }
        }
    }
