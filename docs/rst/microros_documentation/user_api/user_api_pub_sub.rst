.. _micro_user_api_publishers_and_subscribers:

Publishers and subscribers
==========================

ROS 2 publishers and subscribers are the basic communication mechanism between nodes using topics. Further information about ROS 2 publish-subscribe pattern can be found on :ref:`Understanding topics <ROS2Topics>`.

.. TODO(acuadros95): Refer to getting started tutorial

Ready to use code related to this concepts can be found on micro-ROS demos repository `int32_publisher <https://github.com/micro-ROS/micro-ROS-demos/tree/kilted/rclc/int32_publisher/main.c>`_ and `int32_subscriber <https://github.com/micro-ROS/micro-ROS-demos/tree/kilted/rclc/int32_subscriber/main.c>`_ examples.

.. note::

  micro-ROS publishers and subscribers can be configured using quality-of-service. For a better understanding of DDS quality-of-service, check :ref:`About Quality of Service settings <concepts_about_qos>`.

Publisher
---------

Initialization
^^^^^^^^^^^^^^

Starting from a code where ``rclc_support_t`` is initialized and a micro-ROS node is created, there are tree ways to initialize a publisher depending on the desired quality-of-service configuration:

.. code-block:: c

  // Topic name
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Create a reliable publisher ('node' is available and already initialized)
  rcl_publisher_t reliable_publisher;
  rclc_publisher_init_default(
      &reliable_publisher, &node,
      &type_support, &topic_name);

  // Create a best effort publisher
  rcl_publisher_t best_effort_publisher;
  rclc_publisher_init_best_effort(
      &best_effort_publisher, &node,
      &type_support, &topic_name);

  // Create a custom QoS publisher
  rcl_publisher_t custom_publisher;
  rclc_publisher_init(
      &custom_publisher, &node,
      &type_support, &topic_name, &rmw_qos_profile_default);

Publish a message
^^^^^^^^^^^^^^^^^

To publish a message on an successfully initialized publisher:

.. code-block:: c

  // Int32 message object
  std_msgs__msg__Int32 msg;

  // Set message value
  msg.data = 0;

  // Publish message
  rcl_publish(&publisher, &msg, NULL);

For periodic publications, ``rcl_publish`` can be placed inside a timer callback. Check the :ref:`Executor and timers <micro_user_api_executors_and_timers>` section for details.

Subscription
------------

Initialization
^^^^^^^^^^^^^^

The subscription initialization is almost identical to the publisher one:

.. code-block:: c

  // Topic name
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Create a reliable subscriber ('node' is available and already initialized)
  rcl_subscription_t reliable_subscriber;
  rclc_subscription_init_default(
        &reliable_subscriber, &node,
        &type_support, &topic_name);

  // Create a best effort subscriber
  rcl_subscription_t best_effort_subscriber;
  rclc_subscription_init_best_effort(
        &best_effort_subscriber, &node,
        &type_support, &topic_name);

  // Create a custom QoS subscriber
  rcl_subscription_t custom_subscriber;
  rclc_subscription_init(
          &custom_subscriber, &node,
          &type_support, &topic_name, qos_profile);

Callback
^^^^^^^^

The executor is responsible to call the configured callback when a publication is received.
The callback will have a pointer to the received message as its only argument, containing the values received by the subscriber:

.. code-block:: c

  // Function prototype:
  void (* rclc_subscription_callback_t)(const void *);

  // Implementation example:
  void subscription_callback(const void * msgin)
  {
      // Cast message pointer to expected type
      const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;

      // Process message
      printf("Received: %d\n", msg->data);
  }

Once the subscriber and the executor are initialized, the subscriber and its callback must be added to the executor to receive incoming publications once its spinning:

.. code-block:: c

  // Message object to receive publisher data
  std_msgs__msg__Int32 msg;

  // Add subscriber to the executor ('executor' and `subscriber` are available and already initialized)
  rclc_executor_add_subscription(
    &executor, &subscriber, &msg,
    &subscription_callback, ON_NEW_DATA);

  // Spin executor to receive messages
  rclc_executor_spin(&executor);


A subscription can also be removed from the executor after it is not longer needed:

.. code-block:: c

  // Remove a subscription from an executor
  rclc_executor_remove_subscription(
    &executor, &subscription);

Message initialization
----------------------

Before publishing or receiving a message, it may be necessary to initialize its memory for types with strings or sequences.

.. note::

    Check the :ref:`Handling messages memory tutorial <tutorials_micro_memory_management_message_memory>` for details.

Cleaning Up
-----------

After finishing the publisher/subscriber, the node will no longer advertise that it is publishing/listening on the topic.
To destroy an initialized publisher or subscriber:

.. code-block:: c

  // Destroy publisher
  rcl_publisher_fini(&publisher, &node);

  // Destroy subscriber
  rcl_subscription_fini(&subscriber, &node);

This will delete any automatically created infrastructure on the agent (if possible) and free used memory on the client side.
