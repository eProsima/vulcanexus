.. _tutorials_micro_user_api_actions:

Actions
=======

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none


ROS 2 actions are a communication mechanism between two nodes. Actions implement a client-server paradigm with feedback based on ROS 2 messages and types. Further information about ROS 2 actions can be found on :ref:`Understanding actions <ROS2Actions>`.

Ready to use code related to this concepts can be found on micro-ROS demos repository `fibonacci_action_client <https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/fibonacci_action_client/main.c>`_ and `fibonacci_action_server <https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/fibonacci_action_server/main.c>`_ examples.

Action Server
-------------

micro-ROS action servers allows the user to accept or reject ROS 2 action clients goals, send feedback on their execution, and a final result.

Initialization
^^^^^^^^^^^^^^

Action server initialization can be done using the following API:

.. code-block:: c

  // Action name
  const char * action_name = "fibonacci";

  // Get action type support
  const rosidl_action_type_support_t * type_support =
      ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci);

  // Create a reliable action server ('node' and 'support' is available and already initialized)
  rclc_action_server_t action_server;
  rclc_action_server_init_default(
    &action_server,
    &node,
    &support,
    type_support,
    action_name
  );

Callbacks
^^^^^^^^^

Once an action goal request arrives, the executor will execute the configured `handle_goal` callback with the received goal request and the configured context as arguments.
Using the received ``goal_handle``, the action server can accept or reject the request returning ``RCL_RET_ACTION_GOAL_REJECTED`` or ``RCL_RET_ACTION_GOAL_ACCEPTED``.

Using ``Fibonacci.action`` type definition as an example:

.. code-block:: c

    # Goal
    int32 order
    ---
    # Result
    int32[] sequence
    ---
    # Feedback
    int32[] sequence

The client request message will contain an integers ``order``:

.. code-block:: c

  // Function prototype:
  void (* handle_goal_callback_t)(rclc_action_goal_handle_t *, void *);

  // Implementation example:
  rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle, void * context)
  {
    (void) context;

    example_interfaces__action__Fibonacci_SendGoal_Request * req =
      (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

    // Too big, rejecting
    if (req->goal.order > 200) {
      return RCL_RET_ACTION_GOAL_REJECTED;
    }

    // Activate here the goal processing task

    return RCL_RET_ACTION_GOAL_ACCEPTED;
  }

If the goal has been accepted, during the processing task the action server can send feedback to the client using the goal handle:

.. code-block:: c

  // Generate and fill feedback
  example_interfaces__action__Fibonacci_FeedbackMessage feedback = {};
  // Publish feedback message
  rclc_action_publish_feedback(goal_handle, &feedback);

When the the requested goal is completed, the action server can send the result to the client using the goal handle and its finished state: ``GOAL_STATE_SUCCEEDED``, ``GOAL_STATE_CANCELED`` or ``GOAL_STATE_ABORTED``.

.. code-block:: c

  // Generate and fill result message
  example_interfaces__action__Fibonacci_GetResult_Response response = {0};
  rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
  // Send result
  rclc_action_send_result(goal_handle, goal_state, &response);

Also, during the goal handling, the action client can request the cancellation of the goal. If the client requests the cancellation, the action server can decide if the goal can be cancelled using a callback:

.. code-block:: c

  // Function prototype:
  void (* cancel_goal_callback_t)(rclc_action_goal_handle_t *, void *);

  bool handle_cancel(rclc_action_goal_handle_t * goal_handle, void * context)
  {
    if( /* goal can be cancelled */) {
      return true;
    }

    return false;
  }

Finally, all the callbacks can be added to the executor when the action server is added:

.. code-block:: c

  #define NUMBER_OF_SIMULTANEOUS_HANDLES 10

  // Goal request storage
  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[NUMBER_OF_SIMULTANEOUS_HANDLES];

  // Add action server to the executor ('executor' and 'action_server' are available and already initialized)
  rclc_executor_add_action_server(
    &executor,
    &action_server,
    NUMBER_OF_SIMULTANEOUS_HANDLES,
    ros_goal_request,
    sizeof(example_interfaces__action__Fibonacci_SendGoal_Request),
    handle_goal,   // Goal request callback
    handle_cancel, // Goal cancel callback
    NULL           // Context
  );

  // Spin executor to receive requests
  rclc_executor_spin(&executor);

.. note::

  An action server cannot be removed from the executor.

Action Client
-------------

micro-ROS action client allows the user to send ROS 2 action goal requests to an action server, receive feedback and results or request a goal cancellation.


Initialization
^^^^^^^^^^^^^^

Action client initialization can be done using the following API:

.. code-block:: c

  // Action name
  const char * action_name = "fibonacci";

  // Get action type support
  const rosidl_action_type_support_t * type_support =
      ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci);

  // Create a reliable action client ('node' is available and already initialized)
  rclc_action_client_t action_client;
  rclc_action_client_init_default(
    &action_client,
    &node,
    type_support,
    "fibonacci"
  );

Send a request
^^^^^^^^^^^^^^

An action client can send a goal request to the server and spin the executor to get the response.

Using ``Fibonacci.action`` type definition as an example:

.. code-block:: c

    # Goal
    int32 order
    ---
    # Result
    int32[] sequence
    ---
    # Feedback
    int32[] sequence

The client request message will contain an integers ``order``:

.. code-block:: c

  // Fill goal request message
  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;
  ros_goal_request.goal.order = 10;

  // Send goal request
  rclc_action_send_goal_request(&action_client, &ros_goal_request, NULL);
  
  // Spin the executor to get the response
  rclc_executor_spin(&executor);

Callbacks
^^^^^^^^^

In order to receive the goal request response, the client needs to implement a callback:

.. code-block:: c

  // Function prototype:
  void (* goal_request_callback_t)(rclc_action_goal_handle_t *, bool, void *);

  void goal_request_callback(rclc_action_goal_handle_t * goal_handle, bool accepted, void * context)
  {
    example_interfaces__action__Fibonacci_SendGoal_Request * request =
      (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;
    printf(
      "Goal request (order: %d): %s\n",
      request->goal.order,
      accepted ? "Accepted" : "Rejected"
    );
  }

Also, the client needs to implement a callback to receive feedback:

.. code-block:: c

  // Function prototype:
  void (* feedback_callback_t)(rclc_action_goal_handle_t *, void *, void *);

  void feedback_callback(rclc_action_goal_handle_t * goal_handle, void * ros_feedback, void * context)
  {
    example_interfaces__action__Fibonacci_SendGoal_Request * request =
      (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

    example_interfaces__action__Fibonacci_FeedbackMessage * feedback =
      (example_interfaces__action__Fibonacci_FeedbackMessage *) ros_feedback;

    printf(
      "Goal Feedback (order: %d) [",
      request->goal.order
    );

    for (size_t i = 0; i < feedback->feedback.sequence.size; i++) {
      printf("%d ", feedback->feedback.sequence.data[i]);
    }

    printf("]\n");
  }

And finally, the client needs to implement a callback to receive the result:

.. code-block:: c

  // Function prototype:
  void (* result_request_callback_t)(rclc_action_goal_handle_t *, void *, void *);

  void result_request_callback(
    rclc_action_goal_handle_t * goal_handle,
    void * ros_result_response,
    void * context)
  {
    (void) context;

    example_interfaces__action__Fibonacci_SendGoal_Request * request =
      (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

    example_interfaces__action__Fibonacci_GetResult_Response * result =
      (example_interfaces__action__Fibonacci_GetResult_Response *) ros_result_response;

    printf(
      "Goal Result (order: %d) [ ",
      request->goal.order
    );

    if (result->status == GOAL_STATE_SUCCEEDED) {
      for (size_t i = 0; i < result->result.sequence.size; i++) {
        printf("%d ", result->result.sequence.data[i]);
      }
    } else if (result->status == GOAL_STATE_CANCELED) {
      printf("CANCELED ");
    } else {
      printf("ABORTED ");
    }

    printf("]\n");
  }

During the goal execution, the action client can request a goal cancellation:

.. code-block:: c

  // Send goal cancel request
  rclc_action_send_cancel_request(goal_handle);

And define a callback for the cancel request response:

.. code-block:: c

  // Function prototype:
  void (* cancel_request_callback_t)(rclc_action_goal_handle_t *, bool, void *);

  void cancel_request_callback(
    rclc_action_goal_handle_t * goal_handle,
    bool cancelled,
    void * context)
  {
    example_interfaces__action__Fibonacci_SendGoal_Request * request =
      (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

    printf(
      "Goal cancel request (order: %d): %s\n",
      request->goal.order,
      cancelled ? "Accepted" : "Rejected");
  }

Finally, all the callbacks shall be exposed to the executor when the action client is added:

.. code-block:: c

  #define MAX_FIBONACCI_ORDER 50

  // Action message objects
  example_interfaces__action__Fibonacci_FeedbackMessage ros_feedback;
  example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;

  // Init message memory with expected sizes
  ros_feedback.feedback.sequence.capacity = MAX_FIBONACCI_ORDER;
  ros_feedback.feedback.sequence.size = 0;
  ros_feedback.feedback.sequence.data = (int32_t *) malloc(
    ros_feedback.feedback.sequence.capacity * sizeof(int32_t));

  ros_result_response.result.sequence.capacity = MAX_FIBONACCI_ORDER;
  ros_result_response.result.sequence.size = 0;
  ros_result_response.result.sequence.data = (int32_t *) malloc(
    ros_result_response.result.sequence.capacity * sizeof(int32_t));

  // Add action client to the executor ('executor' and 'action_client' are available and already initialized)
  rclc_executor_add_action_client(
    &executor,
    &action_client,
    NUMBER_OF_SIMULTANEOUS_HANDLES,
    &ros_result_response,
    &ros_feedback,
    goal_request_callback,
    feedback_callback,
    result_request_callback,
    cancel_request_callback,
    NULL
  );

  // Spin executor to receive requests
  rclc_executor_spin(&executor);

.. note::

  An action client cannot be removed from the executor.

Message initialization
----------------------

Before using any of the types involved in goal request, cancel or feedback, it may be necessary to initialize its memory for types with strings or sequences.
Check the `Handling messages memory tutorial <https://micro.ros.org/docs/tutorials/advanced/handling_type_memory>`_ on micro-ROS webpage for details.

.. TODO(pgarrido): replace this tutorial with the one in vulcanexus when available

Cleaning Up
-----------

To destroy an initialized action service or client:

.. code-block:: c

  // Destroy service server and client
  rclc_action_server_fini(&action_server, &node);
  rclc_action_client_fini(&action_client, &node);

This will delete any automatically created infrastructure on the agent (if possible) and free used memory on the client side.
