.. _micro_user_middleware:

Middleware API
==============

micro-ROS default rmw layer `rmw_microxrcedds_c <https://github.com/micro-ROS/rmw_microxrcedds>`_ offers a set of utilities to directly configure and interact with the underlying middleware.
The exposed API detailed on this documentation can be used with the ``rmw_microxrcedds_c`` public header: ``#include <rmw_microros/rmw_microros.h>``.

Transport configuration
-----------------------

Middleware provided transports can be configured at run-time, allowing the user to set the Agent address or the serial port identifier:

.. code-block:: c

    // Init RCL options and context
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_init_options_init(&init_options, rcl_get_default_allocator());

    // Take RMW options from RCL options
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // UDP transport configuration
    char* agent_ip = "127.0.0.1";
    char* agent_port = "8888";
    rmw_uros_options_set_udp_address(agent_ip, agent_port, rmw_options);

    // Serial transport configuration
    char* serial_device = "/dev/ttyAMA0";
    rmw_uros_options_set_serial_device(serial_device, rmw_options)

.. note::

   Transport must be enabled on build time using ``RMW_UXRCE_TRANSPORT_UDP`` or ``RMW_UXRCE_TRANSPORT_SERIAL`` options.

Client Key configuration
------------------------

The client key will identify a micro-ROS client on the Agent side, implying that all the connected clients must have a different key.
This value is set randomly on micro-ROS initialization, but it is also possible to set it manually:

.. code-block:: c

    // Init RCL options and context
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_init_options_init(&init_options, rcl_get_default_allocator());

    // Take RMW options from RCL options
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Set RMW client key
    uint32_t client_key = 0xBA5EBA11;
    rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options);

This feature can be useful for reusing DDS entities already created on the agent side, as explained on this `Micro XRCE-DDS Deployment example <https://micro-xrce-dds.docs.eprosima.com/en/latest/getting_started.html#deployment-example>`_.

Time configuration
------------------

Timeouts
^^^^^^^^

- Confirmation timeout: Reliable entities can increase blocking time on ``rcl_publish`` or executor spin calls as they will wait for the acknowledgement of each sent message.

  The default timeout value can be modified individually at run-time:

  .. code-block:: c

    // Confirmation timeout in milliseconds
    int ack_timeout = 1000;

    // Get RWM publisher handle and set reliable timeout
    rmw_publisher_t* rmw_publisher_handle = rcl_publisher_get_rmw_handle(&publisher);
    rmw_uros_set_publisher_session_timeout(&rmw_publisher_handle, ack_timeout);

    // Get RWM service handle and set reliable timeout
    rmw_service_t* rmw_service_handle = rcl_service_get_rmw_handle(&service);
    rmw_uros_set_service_session_timeout(rmw_service_handle, ack_timeout);

    // Get RWM service handle and set reliable timeout
    rmw_client_t* rmw_client_handle = rcl_client_get_rmw_handle(&client);
    rmw_uros_set_client_session_timeout(rmw_client_handle, ack_timeout);

- Entity timeouts: Creation and destruction of entities also include a timeout, as they will wait for the Agent confirmation on the operation. This timeout will affect all ``init`` and ``fini`` methods such as ``rclc_node_init_default``, ``rcl_publisher_fini``, etc.

  The default value can be modified at run-time for all entities:

  .. code-block:: c

      rclc_support_t support;
      support.context

      rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);

      if (NULL != rmw_context)
      {
          // Timeout in milliseconds
          int timeout = 1000;
          rmw_uros_set_context_entity_creation_session_timeout(rmw_context, timeout);
          rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, timeout);
      }


  .. note::

    To avoid waiting for agent confirmation ``timeout = 0`` can be used, allowing the release of local resources if the agent is not present.

.. _micro_ros_api_time_sync:

Time synchronization
^^^^^^^^^^^^^^^^^^^^

micro-ROS clients can synchronize their epoch time with the connected Agent, this can be very useful when working in embedded environments that do not provide any time synchronization mechanism.
This utility is based on the NTP protocol, taking into account delays caused by the transport layer.

An usage example can be found on `epoch_synchronization micro-ROS demo <https://github.com/micro-ROS/micro-ROS-demos/blob/kilted/rclc/epoch_synchronization/main.c>`_:

.. code-block:: c

    // Sync timeout
    const int timeout_ms = 1000;

    // Synchronize time with the agent
    rmw_uros_sync_session(timeout_ms);


After the session is synchronized, the adjusted timestamp can be retrieved with the following API:

.. code-block:: c

    // Check if session has been synchronized
    if (rmw_uros_epoch_synchronized())
    {
        // Get time in milliseconds or nanoseconds
        int64_t time_ms = rmw_uros_epoch_millis();
        int64_t time_ns = rmw_uros_epoch_nanos();
    }

.. note::

    micro-ROS shall be already initialized and connected to the agent to use this functionality.

.. _micro_user_middleware_ping:

Ping agent
----------

The client can test the connection with the Agent with the ping utility. This functionality can be used even when the micro-ROS context has not yet been initialized, which is useful to test the connection before trying to connect to the Agent. An example can be found on `ping_uros_agent micro-ROS demo <https://github.com/micro-ROS/micro-ROS-demos/blob/kilted/rclc/ping_uros_agent/main.c>`_.

.. code-block:: c

    // Timeout for each attempt
    const int timeout_ms = 1000;

    // Number of attemps
    const uint8_t attemps = 5;

    // Ping the agent
    rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms, attempts);

    if (RMW_RET_OK == ping_result)
    {
        // micro-ROS Agent is reachable
        ...
    }
    else
    {
        // micro-ROS Agent is not available
        ...
    }

A secondary API is provided to ping the Agent with a specific rmw configuration. This API allows the user to ping with a specific custom transport without interfering with the actual micro-ROS configuration:

.. code-block:: c

    // Initialize rcl options and retrieve the internal rmw options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Set custom transport
    rmw_uros_options_set_custom_transport(
        false,
        NULL,
        custom_transport_open,
        custom_transport_close,
        custom_transport_write,
        custom_transport_read,
        rmw_options);

    // Ping the agent with custom transport
    const int timeout_ms = 1000;
    const uint8_t attemps = 5;
    rmw_ret_t ping_result = rmw_uros_ping_agent_options(timeout_ms, attempts, rmw_options);

    if (RMW_RET_OK == ping_result)
    {
        // micro-ROS Agent is reachable
        ...
    }
    else
    {
        // micro-ROS Agent is not available
        ...
    }

Error handling
-------------------------

micro-ROS RMW can be configured to report middleware errors to user space using custom callbacks.
This option is disabled by default and needs to be enabled at compile time via ``RMW_UROS_ERROR_HANDLING`` CMake argument.

The behavior of this flag is:

 - ``RMW_UROS_ERROR_HANDLING=OFF``: Error handling is disabled. This is the default behavior. **Errors are not reported to user space**.
 - ``RMW_UROS_ERROR_HANDLING=ON`` and **callback not set**: Error handling is enabled. Default ROS 2 RMW macros are used to report errors.
 - ``RMW_UROS_ERROR_HANDLING=ON`` and **callback set**: Error handling is enabled. User callback and default ROS 2 RMW macros are used to report errors.

An example ``colcon.meta`` is:

.. code-block:: json

  {
      "names": {
          "rmw_microxrcedds": {
              "cmake-args": [
                  "-DRMW_UROS_ERROR_HANDLING=ON"
              ]
          }
      }
  }

Once enabled, the user can register a callback to be called when an error is detected, using the following API:

.. code-block:: c

    void rmw_uros_set_error_handling_callback(
        rmw_uros_error_handling error_cb);

An example callback of type ``rmw_uros_error_handling`` is:

.. code-block:: c

    void error_handler(
        const rmw_uros_error_entity_type_t entity,
        const rmw_uros_error_source_t source,
        const rmw_uros_error_context_t context,
        const char * file,
        const int line)
    {
        // Do something with the error
        ...
    }

``rmw_uros_error_entity_type_t`` represent with entity is triggering the error.
It can be one of the following:

 - ``RMW_UROS_ERROR_ON_UNKNOWN``: Generic entity.
 - ``RMW_UROS_ERROR_ON_NODE``: Node entity.
 - ``RMW_UROS_ERROR_ON_SERVICE``: Service server entity.
 - ``RMW_UROS_ERROR_ON_CLIENT``: Service client entity.
 - ``RMW_UROS_ERROR_ON_SUBSCRIPTION``: Subscription entity.
 - ``RMW_UROS_ERROR_ON_PUBLISHER``: Publisher entity.
 - ``RMW_UROS_ERROR_ON_GRAPH``: Graph manager.
 - ``RMW_UROS_ERROR_ON_GUARD_CONDITION``: Guard condition entity.
 - ``RMW_UROS_ERROR_ON_TOPIC``: Topic memory.

``rmw_uros_error_source_t`` represent the source of the error.
It can be one of the following:

 - ``RMW_UROS_ERROR_ENTITY_CREATION``: Error on entity creation.
 - ``RMW_UROS_ERROR_ENTITY_DESTRUCTION``: Error on entity destruction.
 - ``RMW_UROS_ERROR_CHECK``: Error on a check.
 - ``RMW_UROS_ERROR_NOT_IMPLEMENTED``: Feature not implemented.
 - ``RMW_UROS_ERROR_MIDDLEWARE_ALLOCATION``: Memory error.

``rmw_uros_error_context_t`` represent the context of the error and contains, one or more of the following members:

 - ``node``: Name of the node of type ``const char *``.
 - ``node_namespace``: Namespace of the node of type ``const char *``.
 - ``topic_name``: Name of the topic of type ``const char *``.
 - ``ucdr``: Pointer to the ``ucdrBuffer`` of type ``const ucdrBuffer *``.
 - ``size``: Size of the buffer of type ``size_t``.
 - ``type_support``: Pointer to the type support of type ``const message_type_support_callbacks_t *`` or ``const service_type_support_callbacks_t *``.
 - ``description``: Description of the error of type ``const char *``.

Also, this callback function gets the file name and line number where the error was detected.
This information can be accessed using:

 - ``file``: File name of type ``const char *``.
 - ``line``: Line number of type ``int``.
