.. _tutorials_micro_user_api_executors_and_timers:

Executor and timers
=======================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none


Timers
------

Timers can be created and added to the executor, which will periodically execute the timer callback once it is spinning.
Timers are usually used to handle periodic publications or events.

Initialization
^^^^^^^^^^^^^^

Starting from a code where ``rclc_support_t`` is initialized, a timer can be initialized given a callback and its trigger period in nanoseconds:

.. code-block:: c

  // Timer period on nanoseconds
  const unsigned int timer_period = RCL_MS_TO_NS(1000);

  // Create and initialize timer
  rcl_timer_t timer;
  rclc_timer_init_default(&timer, &support, timer_period, timer_callback);

  // Add to the executor
  rclc_executor_add_timer(&executor, &timer);

Callback
^^^^^^^^

The callback gives a pointer to the associated timer and the time elapsed since the previous call or since the timer was created on the first iteration.

.. code-block:: c

  // Function prototype
  void (* rcl_timer_callback_t)(rcl_timer_t *, int64_t);

  // Implementation example
  void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
  {
      // Check last triggered timestamp
      printf("Last callback time: %ld\n", last_call_time);

      if (timer != NULL)
      {
          // Handle periodic event
      }
  }

The timer can be canceled or have its period and/or callback modified using the passed pointer. Check `rcl/timer.h <https://github.com/ros2/rcl/blob/humble/rcl/include/rcl/timer.h>`_ for details.

Cleaning Up
^^^^^^^^^^^

To destroy an initialized timer:

.. code-block:: c

  // Destroy timer
  rcl_timer_fini(&timer);

This will free used memory and make the timer invalid

Executor
--------

The rclc executor allows to handle the execution of callbacks of entities such as subscriptions, timers, services and client taking into account the required priorities.

Initialization
^^^^^^^^^^^^^^

The maximum number of handles of an executor is configured on its initialization.
A handle can be defined as a single event or callback: subscriptions, timers, services, clients and guard conditions. Nodes and publishers are excluded as they do not trigger input events.

.. code-block:: c

  // Get configured allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Initialize support object
  rclc_support_t support;
  rclc_support_init(&support, argc, argv, &allocator);

  // Initialize executor
  rclc_executor_t executor;
  const size_t number_of_handles = 1;
  rclc_executor_init(&executor, &support.context, number_of_handles, &allocator);

.. note::

  Complex entities with underlying services or subscriptions define their expected number of handles, as for example the parameter server.

Add/Remove entities
^^^^^^^^^^^^^^^^^^^^^^^

As explained on their respective ``Callback`` sections, entities can be added and removed from an executor.

The executor will trigger the callbacks of the added entities, so periodic events or incoming messages can be handled.
Removed entities will free the used executor handles and their callback will be removed from the executor events.

.. note::

  Complex entities as the parameter server or action client/servers cannot be removed from an executor.

Spinning
^^^^^^^^

The executor implements the ROS2 spin mechanism. During a spin, periodic events and incoming messages will be handle, invoking the respective callbacks for each event.
The following spin methods are available:

- Spin: Endless spin, will block and check for events using a busy wait approach.

  .. code-block:: c

    // Spin endlessly
    rclc_executor_spin(&executor);

- Spin period: Endless periodic spin. This method will block and check for events using periodically.
  If an event is triggered, the method will sleep for the remaining period time.

  .. code-block:: c

    // Spin period on nanoseconds
    const unsigned int spin_period = RCL_MS_TO_NS(1000);

    // Spin endlessly
    rclc_executor_spin_period(&executor, spin_period);

- Spin some: Spin with timeout. Will check for events with a maximum timeout.
  If an event is triggered within the specified timeout, the method will exit.

  .. code-block:: c

    // Spin timeout on nanoseconds
    const unsigned int spin_timeout = RCL_MS_TO_NS(100);

    // Spin with timeout
    rclc_executor_spin_some(&executor, spin_timeout);


.. TODO(pgarrido): Reenable when multithreading section is ready

.. Multithreading
.. ^^^^^^^^^^^^^^

.. To use executors on multithreaded environments, a different executor shall be created for each running thread.
.. This implies that different entities will spin on each thread, which gives opportunities for setting different priorities for a set of entities.

.. .. warning::

..   Entities creation and destruction is not thread safe and shall not occur while the executor is spinning.

Cleaning Up
^^^^^^^^^^^

To destroy an initialized executor:

.. code-block:: c

  // Destroy executor
  rclc_executor_fini(&executor);

This will free used memory and make the executor invalid.
