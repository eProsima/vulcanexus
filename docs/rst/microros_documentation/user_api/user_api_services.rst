.. _micro_user_api_services:

Services
=======================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none

ROS 2 services are another communication mechanism between nodes. Services implement a client-server paradigm based on ROS 2 messages and types. Further information about ROS 2 services can be found on  :ref:`Understanding services <ROS2Services>`.

Ready to use code related to this concepts can be found on micro-ROS demos repository `addtwoints_server <https://github.com/micro-ROS/micro-ROS-demos/tree/jazzy/rclc/addtwoints_server/main.c>`_ and `addtwoints_client <https://github.com/micro-ROS/micro-ROS-demos/tree/jazzy/rclc/addtwoints_client/main.c>`_ examples.

Server
------

Initialization
^^^^^^^^^^^^^^

Services initialization follow the same pattern as publishers and subscribers:

.. code-block:: c

  // Service name
  const char * service_name = "test_service";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Create a reliable service server ('node' is available and already initialized)
  rcl_service_t reliable_service;
  rclc_service_init_default(
      &reliable_service, &node,
      type_support, service_name);

  // Create a best effort service server
  rcl_service_t best_effort_service;
  rclc_service_init_best_effort(
      &best_effort_service, &node,
      type_support, service_name);

  // Create a custom QoS service server
  rcl_service_t custom_service;
  rclc_service_init(
      &custom_service, &node, type_support,
      service_name, &rmw_qos_profile_services_default);

Callback
^^^^^^^^

Once a request arrives, the executor will call the configured callback with the request and response messages as arguments.
The request message ``request_msg`` contains the request sent by the client, while the ``response_msg`` argument should be modified with the server response.
The response message will be sent to the client after the callback returns.

Using ``AddTwoInts.srv`` type definition as an example:

.. code-block:: c

  int64 a
  int64 b
  ---
  int64 sum

The client request message will contain two integers ``a`` and ``b``, and expects the ``sum`` of those values as the response:

.. code-block:: c

  // Function prototype:
  void (* rclc_service_callback_t)(const void *, void *);

  // Implementation example:
  void service_callback(const void * request_msg, void * response_msg)
  {
      // Cast messages to expected types
      example_interfaces__srv__AddTwoInts_Request * req_in =
          (example_interfaces__srv__AddTwoInts_Request *) request_msg;
      example_interfaces__srv__AddTwoInts_Response * res_in =
          (example_interfaces__srv__AddTwoInts_Response *) response_msg;

      // Handle request message and set the response message values
      printf("Client requested sum of %d and %d.\n", (int) req_in->a, (int) req_in->b);
      res_in->sum = req_in->a + req_in->b;
  }

As in the subscriber, the service and its callback must be added to the executor in order to process incoming requests on a the executor spin:

.. code-block:: c

  // Service message objects
  example_interfaces__srv__AddTwoInts_Response response_msg;
  example_interfaces__srv__AddTwoInts_Request request_msg;

  // Add service server to the executor ('executor' and 'service' are available and already initialized)
  rclc_executor_add_service(
      &executor, &service, &request_msg,
      &response_msg, service_callback);

  // Spin executor to receive requests
  rclc_executor_spin(&executor);

A service server can also be removed from the executor after it is not longer needed:

.. code-block:: c

  // Remove a service server from an executor
  rclc_executor_remove_service(
    &executor, &service);

Client
------

Initialization
^^^^^^^^^^^^^^

.. code-block:: c

  // Service name
  const char * service_name = "test_service";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Create a reliable service client ('node' is available and already initialized)
  rcl_client_t reliable_client;
  rclc_client_init_default(
      &reliable_client, &node,
      type_support, service_name);

  // Create a best effort service client
  rcl_client_t best_effort_client;
  rclc_client_init_best_effort(
      &best_effort_client, &node,
      type_support, service_name);

  // Create a custom QoS service client
  rcl_client_t custom_client;
  rclc_client_init(
      &custom_client, &node, type_support,
      service_name, &rmw_qos_profile_services_default);

Callback
^^^^^^^^

The function will have the response message as its only argument, containing the response sent by the server:

.. code-block:: c

  // Function prototype:
  void (* rclc_client_callback_t)(const void *);

  // Implementation example:
  void client_callback(const void * response_msg)
  {
      // Cast response message to expected type
      example_interfaces__srv__AddTwoInts_Response * msgin =
          (example_interfaces__srv__AddTwoInts_Response * ) response_msg;

      // Handle response message
      printf("Received service response %ld + %ld = %ld\n", req.a, req.b, msgin->sum);
  }

Adding the service client to the executor:

.. code-block:: c

  // Response message object
  example_interfaces__srv__AddTwoInts_Response res;

  // Add service client to the executor ('executor' and 'client' are available and already initialized)
  rclc_executor_add_client(&executor, &client, &res, client_callback);

  // Spin executor to receive requests
  rclc_executor_spin(&executor);

A service client can also be removed from the executor after it is not longer needed:

.. code-block:: c

  // Remove a service client from an executor
  rclc_executor_remove_client(
    &executor, &client);

Send a request
^^^^^^^^^^^^^^

Once the service client and server are configured, the service client can send a request and spin the executor to get the reply.
Following the example on `AddTwoInts.srv`:

.. code-block:: c

  // Request message object (Must match initialized client type support)
  example_interfaces__srv__AddTwoInts_Request request_msg;

  // Initialize request message memory and set its values
  example_interfaces__srv__AddTwoInts_Request__init(&request_msg);
  request_msg.a = 24;
  request_msg.b = 42;

  // Sequence number of the request (Populated in 'rcl_send_request')
  int64_t sequence_number;

  // Send request
  rcl_send_request(&client, &request_msg, &sequence_number);

  // Spin the executor to get the response
  rclc_executor_spin(&executor);

Message initialization
----------------------

Before publishing or receiving a message, it may be necessary to initialize its memory for types with strings or sequences.

.. note::

    Check the :ref:`Handling messages memory tutorial <tutorials_micro_memory_management_message_memory>` for details.

Cleaning Up
-----------

To destroy an initialized service or client:

.. code-block:: c

  // Destroy service server and client
  rcl_service_fini(&service, &node);
  rcl_client_fini(&client, &node);

This will delete any automatically created infrastructure on the agent (if possible) and free used memory on the client side.
