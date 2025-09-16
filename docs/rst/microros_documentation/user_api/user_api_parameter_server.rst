.. _micro_user_api_parameter_server:

Parameter Server
=======================

ROS 2 parameters allow the user to create variables on a node and manipulate/read them with different ROS 2 commands. Further information about ROS 2 parameters can be found on :ref:`Understanding parameters <ROS2Params>`.

Ready to use code related to this concepts can be found on micro-ROS demos repository `parameter_server <https://github.com/micro-ROS/micro-ROS-demos/blob/kilted/rclc/parameter_server/main.c>`_ example.

Initialization
--------------
A micro-ROS parameter server can be initiated using the RCLC parameter server API:

.. code-block:: c

    // Parameter server object
    rclc_parameter_server_t param_server;

    // Initialize parameter server with default configuration
    rcl_ret_t rc = rclc_parameter_server_init_default(&param_server, &node);

    if (RCL_RET_OK != rc)
    {
        ... // Handle error
        return -1;
    }

Options
^^^^^^^
A parameter server can be configured at configuration time, the following options can be adjusted:

- ``notify_changed_over_dds``: Publish parameter events to other ROS 2 nodes as well.
- ``max_params``: Maximum number of parameters allowed on the ``rclc_parameter_server_t`` object.
- ``allow_undeclared_parameters``: Allows creation of parameters from external parameter clients. A new parameter will be created if a ``set`` operation is requested on a non-existing parameter.
- ``low_mem_mode``: Reduces the memory used by the parameter server, functionality constrains are applied.

.. code-block:: c

    // Parameter server object
    rclc_parameter_server_t param_server;

    // Initialize parameter server options
    const rclc_parameter_options_t options = {
        .notify_changed_over_dds = true,
        .max_params = 4,
        .allow_undeclared_parameters = true,
        .low_mem_mode = false; };

    // Initialize parameter server with configured options
    rcl_ret_t rc = rclc_parameter_server_init_with_option(&param_server, &node, &options);

    if (RCL_RET_OK != rc)
    {
        ...     // Handle error
        return -1;
    }

Low memory mode
^^^^^^^^^^^^^^^

There is a low memory mode that ports the parameter functionality to memory constrained devices. The following constrains are applied:

- Request size limited to one parameter on Set, Get, Get types and Describe operations.
- List parameter request has no prefixes enabled nor depth.
- Parameter description strings not allowed, ``rclc_add_parameter_description`` is disabled.

.. note::

    Using low memory mode in a STM32F4 with  7 parameters with ``RCLC_PARAMETER_MAX_STRING_LENGTH = 50`` and ``notify_changed_over_dds = true`` the memory usage drops from 11.7 kB to 4.1 kB.

Callback
--------

When adding the parameter server to the executor, a callback could to be configured. This callback would then be executed on the following events:

- Parameter value change: Internal and external parameter set on existing parameters.
- Parameter creation: External parameter set on unexisting parameter if ``allow_undeclared_parameters`` is set.
- Parameter delete: External parameter delete on existing parameter.
- The user can allow or reject this operations using the bool return value.

Callback parameters:

- ``old_param``: Parameter actual value, ``NULL`` for new parameter request.
- ``new_param``: Parameter new value, ``NULL`` for parameter removal request.
- ``context``: User context, configured on ``rclc_executor_add_parameter_server_with_context``.

.. code-block:: c

    bool on_parameter_changed(
            const Parameter* old_param,
            const Parameter* new_param,
            void* context)
    {
        (void) context;

        if (old_param == NULL && new_param == NULL)
        {
            printf("Callback error, both parameters are NULL\n");
            return false;
        }

        if (old_param == NULL)
        {
            printf("Creating new parameter %s\n", new_param->name.data);
        }
        else if (new_param == NULL)
        {
            printf("Deleting parameter %s\n", old_param->name.data);
        }
        else
        {
            printf("Parameter %s modified.", old_param->name.data);
            switch (old_param->value.type){
                case RCLC_PARAMETER_BOOL:
                    printf(
                        " Old value: %d, New value: %d (bool)", old_param->value.bool_value,
                        new_param->value.bool_value);
                    break;
                case RCLC_PARAMETER_INT:
                    printf(
                        " Old value: %ld, New value: %ld (int)", old_param->value.integer_value,
                        new_param->value.integer_value);
                    break;
                case RCLC_PARAMETER_DOUBLE:
                    printf(
                        " Old value: %f, New value: %f (double)", old_param->value.double_value,
                        new_param->value.double_value);
                    break;
                default:
                    break;
            }
            printf("\n");
        }

        return true;
    }

Parameters modifications are disabled while the callback ``on_parameter_changed`` is executed, causing the following methods to return ``RCLC_PARAMETER_DISABLED_ON_CALLBACK`` if they are invoked:

- ``rclc_add_parameter``
- ``rclc_delete_parameter``
- ``rclc_parameter_set_bool``
- ``rclc_parameter_set_int``
- ``rclc_parameter_set_double``
- ``rclc_set_parameter_read_only``
- ``rclc_add_parameter_constraint_double``
- ``rclc_add_parameter_constraint_integer``

Once the parameter server and the executor are initialized, the parameter server must be added to the executor in order to accept parameter commands from ROS 2:

.. code-block:: c

    // Add parameter server to the executor including defined callback
    rc = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);

Note that this callback is optional as its just an event information for the user. To use the parameter server without a callback:

.. code-block:: c

    // Add parameter server to the executor without a callback
    rc = rclc_executor_add_parameter_server(&executor, &param_server, NULL);

Configuration of the callback context:

.. code-block:: c

    // Add parameter server to the executor including defined callback and a context
    rc = rclc_executor_add_parameter_server_with_context(&executor, &param_server, on_parameter_changed, &context);

Add a parameter
---------------

The micro-ROS parameter server supports the following parameter types:

- Boolean:

    .. code-block:: c

        const char* parameter_name = "parameter_bool";
        bool param_value = true;

        // Add parameter to the server
        rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_BOOL);

        // Set parameter value (Triggers `on_parameter_changed` callback, if defined)
        rc = rclc_parameter_set_bool(&param_server, parameter_name, param_value);

        // Get parameter value and store it in "param_value"
        rc = rclc_parameter_get_bool(&param_server, "param1", &param_value);

        if (RCL_RET_OK != rc)
        {
            ...         // Handle error
            return -1;
        }

- Integer:

    .. code-block:: c

        const char* parameter_name = "parameter_int";
        int param_value = 100;

        // Add parameter to the server
        rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_INT);

        // Set parameter value
        rc = rclc_parameter_set_int(&param_server, parameter_name, param_value);

        // Get parameter value on param_value
        rc = rclc_parameter_get_int(&param_server, parameter_name, &param_value);

- Double:

    .. code-block:: c

        const char* parameter_name = "parameter_double";
        double param_value = 0.15;

        // Add parameter to the server
        rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_DOUBLE);

        // Set parameter value
        rc = rclc_parameter_set_double(&param_server, parameter_name, param_value);

        // Get parameter value on param_value
        rc = rclc_parameter_get_double(&param_server, parameter_name, &param_value);

The parameter string name size is controlled by the compile-time option ``RCLC_PARAMETER_MAX_STRING_LENGTH``, the default value is 50.

.. note::

    Parameters can also be created by external clients if the ``allow_undeclared_parameters`` flag is set. The client just needs to set a value on a non-existing parameter. Then this parameter will be created if the server has still capacity and the callback allows the operation.

Delete a parameter
------------------

Parameters can be deleted by both, the parameter server and external clients:

.. code-block:: c

    rclc_delete_parameter(&param_server, "param2");

For external delete requests, the server callback will be executed, allowing the node to reject the operation.

Parameters description
----------------------

- Parameter description: Adds a description of a parameter and its constraints, which will be returned on a describe parameter request:

    .. code-block:: c

        rclc_add_parameter_description(&param_server, "param2", "Second parameter", "Only even numbers");

    The maximum string size is controlled by the compilation time option ``RCLC_PARAMETER_MAX_STRING_LENGTH``, default value is 50.

- Parameter constraints: Informative numeric constraints that can be added to int and double parameters, returning these values on describe parameter requests:

    - ``from_value``: Start value for valid values, inclusive.
    - ``to_value``: End value for valid values, inclusive.
    - ``step``: Size of valid steps between the from and to bound.

    .. code-block:: c

        int64_t int_from = 0;
        int64_t int_to = 20;
        uint64_t int_step = 2;
        rclc_add_parameter_constraint_integer(&param_server, "param2", int_from, int_to, int_step);

        double double_from = -0.5;
        double double_to = 0.5;
        double double_step = 0.01;
        rclc_add_parameter_constraint_double(&param_server, "param3", double_from, double_to, double_step);

    .. note::

        This constrains will not be applied by the parameter server, leaving values filtering to the user callback.

- Read-only parameters: This flag blocks parameter changes from external clients, but allows changes on the server side:

    .. code-block:: c

        bool read_only = true;
        rclc_set_parameter_read_only(&param_server, "param3", read_only);

Memory requirements
-------------------

The parameter server uses five services and an optional publisher. These need to be taken into account on the `rmw_microxrcedds` package memory configuration:

.. note::

    Check :ref:`micro-ROS memory management for entity creating tutorial <tutorials_micro_memory_management_middleware_entity_creation>` for more information.


.. code-block:: python

    # colcon.meta example with memory requirements to use a parameter server
    {
        "names": {
            "rmw_microxrcedds": {
                "cmake-args": [
                    "-DRMW_UXRCE_MAX_NODES=1",
                    "-DRMW_UXRCE_MAX_PUBLISHERS=1",
                    "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=0",
                    "-DRMW_UXRCE_MAX_SERVICES=5",
                    "-DRMW_UXRCE_MAX_CLIENTS=0"
                ]
            }
        }
    }

At runtime, the variable ``RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES`` defines the necessary number of handles required by a parameter server for the rclc Executor:

.. code-block:: c

    // Executor init example with the minimum RCLC executor handles required
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(
        &executor, &support.context,
        RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);

Cleaning up
-----------

To destroy an initialized parameter server:

.. code-block:: c

    // Delete parameter server
    rclc_parameter_server_fini(&param_server, &node);

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the parameter server side.
