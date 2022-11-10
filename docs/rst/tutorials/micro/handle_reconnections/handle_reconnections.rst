.. _tutorials_micro_handle_reconnections:

Reconnections and liveliness
============================

.. contents:: Table of Contents
    :depth: 1
    :local:
    :backlinks: none

This tutorial aims at providing insight on mechanism for handling reconnections between the micro-ROS Agent and Client and assert the connection liveliness.

This features can be useful in multiple situations when the Agent or the Client are no longer available or the communication channel between them is broken. Those situations can be, among others:

- Transport faults: Detect transport fault or disconnections and act accordingly.
- Agent or Client restarts: Make the system resilient to a possible restart on the MCU or Agent side.
- Client plug and play: Useful on systems with multiple MCU boards that are attached on demand.

Client side: Ping API
---------------------

micro-ROS Client default rmw layer `rmw_microxrcedds_c <https://github.com/micro-ROS/rmw_microxrcedds>`_ offers a ping utility to test the connection with the Agent. This ping utility can be used at any stage of the micro-ROS application, allowing users to check the Agent availability before attempting to initialize a micro-ROS session or create any entities.

Usage details on this utility can be found in the :ref:`Middleware API: Ping Agent <tutorials_micro_user_middleware_ping>` tutorial.

State machine approach
^^^^^^^^^^^^^^^^^^^^^^

Reconnections can be handle manually on the micro-ROS Client side using the Agent ping functionality with the following sequence:

1. Wait until Agent is reachable.
2. Create required micro-ROS entities.
3. Use the micro-ROS API as usual: spin entities, publish, etc.
4. When a failure is detected in steps 2 or 3, destroy the created entities and go back to the first step.

This approach allows a micro-ROS Client to handle Agent restarts or to follow a plug and play approach, where the micro-ROS app will only run when the Agent connection is available. The following code shows an example of this sequence:

.. code-block:: c

    // Timeout for each ping attempt
    const int timeout_ms = 100;

    // Number of ping attempts
    const uint8_t attempts = 1;

    // Spin period
    const unsigned int spin_timeout = RCL_MS_TO_NS(100);

    // Enum with connection status
    enum states {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } state;

    while (true)
    {
        switch (state)
        {
            case WAITING_AGENT:
                // Check for agent connection
                state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
                break;

            case AGENT_AVAILABLE:
                // Create micro-ROS entities
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

                if (state == WAITING_AGENT)
                {
                    // Creation failed, release allocated resources
                    destroy_entities();
                };
                break;

            case AGENT_CONNECTED:
                // Check connection and spin on success
                state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (state == AGENT_CONNECTED)
                {
                    rclc_executor_spin_some(&executor, spin_timeout);
                }
                break;

            case AGENT_DISCONNECTED:
                // Connection is lost, destroy entities and go back to first step
                destroy_entities();
                state = WAITING_AGENT;
                break;

            default:
                break;
        }
    }

A working example with this approach can be found on micro-ROS for Arduino repository `micro-ros_reconnection <https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino>`_ example.

Agent side: Hard liveliness check
---------------------------------

The main problem with the previous section's method is that entity destruction always happens on micro-ROS Client's request. This implies that other ROS 2 entities will not be aware of the micro-ROS Client destruction.

The **Hard Liveliness Check** mechanism allows the micro-ROS Agent to ping the Client periodically. This way, the Agent will take care of ensuring that the micro-ROS client is alive and will destroy the created entities if a certain timeout happens without any response from the Client side. This means that the nodes, publishers, subscribers (and any other entity) created by the Client will be removed from the ROS 2 graph.

This mechanism does not have a penalty on the application throughput, as it will avoid sending ping messages if the Agent is receiving data from the Client.

In other cases, the micro-ROS Client shall spin an executor to give a response to the Agent liveliness check messages, an empty executor can be used for this purpose.

.. note::

    Note that the Client shall also be aware of the disconnection to create the micro-ROS entities again, this can be achieved by including the previous section approach.

Configuration
^^^^^^^^^^^^^

This feature is enabled by default in the micro-ROS Agent and **must be enabled** by means of `colcon.meta` parameters in the micro-ROS Client:

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
