.. _tutorials_micro_memory_management:

Memory management
=================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

micro-ROS Client provides a full control over the memory usage during build, configuration and runtime.
This is one the most important requisites in order to fit in low resources system and also in order to guarantee a hard real-time operation.

The memory management in micro-ROS Client can be configured at multiple levels so the user can use different mechanisms for fitting its requirements within the micro-ROS environment.

In general, micro-ROS by default will use:

* Static allocated memory in **build time**
* A bounded stack memory consumption
* A bounded amount of **dynamic memory** during entity creation and destruction (configuration time)
* Zero **dynamic memory** during runtime


.. TODO(pgarrido): Link to profiling article


During a micro-ROS Client application development the user is able configure the memory at multiple level.
Along the following sections, those levels are analyzed in detail.

.. _tutorials_micro_memory_management_allocators:

Allocators
----------

As in the ROS 2 stack, in the micro-ROS stack the dynamic memory allocators can be customized at runtime.
By default those allocators relies on the ``libc`` implementation of ``malloc``, ``calloc``, ``realloc`` and ``free`` functions.
However, in some platforms those functions are not available or not encouraged to be used and they can be replaced by platform specific functions.
One example of this situation is `FreeRTOS allocators <https://www.freertos.org/a00111.html>`_.

An example on how to set custom allocators at runtime when using ``rcutils`` API is:

.. code-block:: c

  allocator = rcutils_get_zero_initialized_allocator();
  allocator.allocate = custom_allocate;
  allocator.deallocate = custom_deallocate;
  allocator.reallocate = custom_reallocate;
  allocator.zero_allocate = custom_zero_allocate;

  rcutils_set_default_allocator(&allocator);

A reference implementation of those allocators is:

.. code-block:: c

  void * custom_allocate(size_t size, void * state)
  {
    // Allocate and return a memory chunk of `size` bytes.
  }

  void custom_deallocate(void * pointer, void * state)
  {
    // Deallocate memory chunk pointed by `pointer`.
  }

  void * custom_reallocate(void * pointer, size_t size, void * state)
  {
    // Reallocate memory chunk pointed by `pointer` to `size` bytes.
  }

  void * custom_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
  {
    // Allocate and return a memory chunk of `number_of_elements * size_of_element` bytes, filled with zeros.
  }

One example implementation of the most basic allocator that targets platforms where no ``libc`` allocators are available is:

.. note::

  This is a naive implementation of an allocator that does not allows memory deallocation.
  User shall be aware of using `free_all_heap()` when the micro-ROS entities are no longer required.

.. code-block:: c

    static uint8_t heap[HEAP_SIZE];
    static size_t current_pointer = 0;

    void free_all_heap()
    {
        current_pointer = 0;
    }

    void assert_position()
    {
        if (current_pointer >= sizeof(heap)) {
            // Handle memory error
            while(1){};
        }
    }

    #define SYSTEM_ALIGNMENT 4

    size_t align_size(size_t size)
    {
        if (size % SYSTEM_ALIGNMENT != 0) {
            size += SYSTEM_ALIGNMENT - (size % SYSTEM_ALIGNMENT);
        }
    }

    void * custom_allocate(size_t size, void * state)
    {
        size = align_size(size);
        size_t p = current_pointer;
        current_pointer += size;
        assert_position();
        return (void *) &heap[p];
    }

    void custom_deallocate(void * pointer, void * state)
    {
        (void) state;
        (void) pointer;
    }

    void * custom_reallocate(void * pointer, size_t size, void * state)
    {
        size = align_size(size);
        size_t p = current_pointer;
        current_pointer += size;
        // Careful! pointer may have less than size memory, garbage can be copied!
        memcpy(&heap[p], pointer, size);
        assert_position();
        return (void *) &heap[p];
    }

    void * custom_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
    {
        size_t size = number_of_elements * size_of_element;
        size = align_size(size);
        size_t p = current_pointer;
        current_pointer += size;
        memset(&heap[p], 0, size);
        assert_position();
        return (void *) &heap[p];
    }

Middleware memory
-----------------

By default micro-ROS uses an RMW based on `eProsima Micro XRCE-DDS Client <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_.
This RMW implementation is known as ``rmw_microxrcedds`` and it main purpose in terms of memory is to avoid dynamic memory allocation and allowing the user to configure the memory usage during build time.

eProsima Micro XRCE-DDS Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the lower level of the middleware layers, the user can configure the maximum transfer unit of eProsima Micro XRCE-DDS Client by means of setting one of the following flag for ``microxrcedds_client`` package in the ``colcon.meta`` file:

- ``UCLIENT_UDP_TRANSPORT_MTU``: Maximum transfer unit for UDP transport. Default value: 512 bytes.
- ``UCLIENT_TCP_TRANSPORT_MTU``: Maximum transfer unit for TCP transport. Default value: 512 bytes.
- ``UCLIENT_SERIAL_TRANSPORT_MTU``: Maximum transfer unit for serial transport. Default value: 512 bytes.
- ``UCLIENT_CUSTOM_TRANSPORT_MTU``: Maximum transfer unit for custom transport. Default value: 512 bytes.

.. note::

    Note that although many micro-ROS ports uses UDP or Serial transport, most of them are implemented over  :ref:`Custom transport API<tutorials_micro_custom_transports>`. Therefore, the maximum transfer unit shall be set using ``UCLIENT_CUSTOM_TRANSPORT_MTU``.


The rest of configuration parameters at middleware level are located in ``rmw_microxrcedds`` package configuration.
The following parameters can be configured when building the micro-ROS Client library via ``colcon.meta`` file:

.. _tutorials_micro_memory_management_middleware_entity_creation:

Entity creation
^^^^^^^^^^^^^^^

By means of the following build flags, the user can configure the maximum number of entities that can be created during the micro-ROS Client execution:

- ``RMW_UXRCE_MAX_SESSIONS``: Maximum number of XRCE-DDS sessions. Default value: 1.
- ``RMW_UXRCE_MAX_NODES``: Maximum number of micro-ROS nodes. Default value: 4.
- ``RMW_UXRCE_MAX_PUBLISHERS``: Maximum number of micro-ROS publishers. Default value: 4.
- ``RMW_UXRCE_MAX_SUBSCRIPTIONS``: Maximum number of micro-ROS subscriptions. Default value: 4.
- ``RMW_UXRCE_MAX_SERVICES``: Maximum number of micro-ROS services. Default value: 4.
- ``RMW_UXRCE_MAX_CLIENTS``: Maximum number of micro-ROS clients. Default value: 4.
- ``RMW_UXRCE_MAX_WAIT_SETS``: Maximum number of micro-ROS wait sets. Default value: 4.
- ``RMW_UXRCE_MAX_GUARD_CONDITION``: Maximum number of micro-ROS guard conditions (used for timers among other things). Default value: 4.
- ``RMW_UXRCE_MAX_TOPICS``: Maximum number of micro-ROS topics. Default value: 4. If set to ``-1`` the value will be calculated as ``RMW_UXRCE_MAX_TOPICS`` = ``RMW_UXRCE_MAX_PUBLISHERS`` + ``RMW_UXRCE_MAX_SUBSCRIPTIONS`` + ``RMW_UXRCE_MAX_NODES``.

In the default configuration, micro-ROS Client will not be able to create more entities than the ones specified above.
If dynamic memory usage is allowed, by means of the following flag, the user can allow on-demand entity creating using dynamic memory when required.
This dynamic memory usage uses micro-ROS allocators.

- ``RMW_UXRCE_ALLOW_DYNAMIC_ALLOCATIONS``: Allow dynamic memory allocations when creating micro-ROS entities. Default value: ``OFF``.

Middleware related memory
^^^^^^^^^^^^^^^^^^^^^^^^^

By means of the following build flags, the user can configure the behavior of communication buffers:

- ``RMW_UXRCE_STREAM_HISTORY``: Maximum number of buffers of size ``UCLIENT_[XXX]_TRANSPORT_MTU`` that the XRCE-DDS layer is provided with. **It shall be power of 2**. Default value: 4.
- ``RMW_UXRCE_STREAM_HISTORY_INPUT``: Override for ``RMW_UXRCE_STREAM_HISTORY`` for input buffers. Default value: ``RMW_UXRCE_STREAM_HISTORY``.
- ``RMW_UXRCE_STREAM_HISTORY_OUTPUT``: Override for ``RMW_UXRCE_STREAM_HISTORY`` for output buffers. Default value: ``RMW_UXRCE_STREAM_HISTORY``.
- ``RMW_UXRCE_MAX_HISTORY``: Maximum number of slots for storing incoming data between ``wait()`` and ``take()`` operations at RMW layer. It size is ``UCLIENT_[XXX]_TRANSPORT_MTU * RMW_UXRCE_STREAM_HISTORY[_INPUT]``  Default value: 8.

.. note::

    When using Best Effort entities, the maximum serialized size of a topic shall fit in ``UCLIENT_[XXX]_TRANSPORT_MTU`` both for outgoing and incoming data.

    When using Reliable entities, the maximum serialized size of a topic shall fit in ``UCLIENT_[XXX]_TRANSPORT_MTU * RMW_UXRCE_STREAM_HISTORY`` both for outgoing and incoming data.

    When receiving data, and all ``RMW_UXRCE_MAX_HISTORY`` are occupied, the reception entity behavior is determined by History QoS.

Entity Names
^^^^^^^^^^^^

Regarding the name of entities, topics and types, the user can configure the maximum length of the name by means of the following flags:

- ``RMW_UXRCE_NODE_NAME_MAX_LENGTH``: Maximum number of characters for a node name. Default value: 60.
- ``RMW_UXRCE_TOPIC_NAME_MAX_LENGTH``: Maximum number of characters for a topic name. Default value: 60.
- ``RMW_UXRCE_TYPE_NAME_MAX_LENGTH``: Maximum number of characters for a type name. Default value: 100.


.. _tutorials_micro_memory_management_message_memory:

Message memory
--------------

Message memory handling is an important part of the micro-ROS Client memory handling due to the fact that **micro-ROS Client does not initialized by default the type memory**.
This means that the user must initialize the type memory before using it.
This consideration needs to be taken into account both for outgoing and incoming messages.

.. warning::

    micro-ROS provides an API for initializing the type memory that can be found in :ref:`micro-ROS Types Utilities<tutorials_micro_utilities_type_utilities>`.

    This section provides an explanation of micro-ROS type memory handling in the case that this API is not available or not used for some reason.

A message type, both used for topics or services, is composed defined in a ``.msg``, ``.srv`` or ``.action`` file.
Each one of those files will describer members of the type that shall be one of the following types:

- **Basic type**: integers, floats, booleans, etc.
- **Compound type**: another ROS 2 defined type.
- **Array type**: fixed size arrays of basic or compound types.
- **Sequence type**: variable size sequences of basic or compound types.

For example, the following ``.msg`` can be described as follows:

.. code-block::

    # MyType.msg
    std_msgs/Header header
    int32[] values
    float64 duration
    int8[10] coefficients
    string name

- the member ``duration`` is a **basic type** member.
- the member ``values`` is a **sequence type** member because it has a unbounded sequence of int32, in this case.
- the member ``coefficients`` is an **array type** member because it has a bounded sequence of 10 units of int8, in this case.
- the member ``header`` is an **compound type** member because it refers to type described in the same or other ROS 2 package.
- the member ``name`` is an **string type** member and should be understood as a char[] (sequence type member).

When dealing with the micro-ROS typesupport the developer needs to take into account how this message is going to be handled in the C99 API of micro-ROS.
In general, the micro-ROS typesupport will create a C99 ``struct`` representation of the message:

.. code-block:: c

    typedef struct mypackage__msg__MyType
    {
        std_msgs__msg__Header header;
        rosidl_runtime_c__int32__Sequence values;
        double duration;
        int8 coefficients[10];
        rosidl_runtime_c__String name;  // equal to rosidl_runtime_c__char__Sequence
    } mypackage__msg__MyType;

When in an application instances a variable of this type, for example ``mypackage__msg__MyType mymsg;``, it is ensured that:

- ``mymsg.coefficients`` has a C array of ``int8``.
- ``mymsg.duration`` is a ``double`` member.

But no memory is guaranteed to be allocated automatically for other members.

Sequence types
^^^^^^^^^^^^^^

A **sequence type member** is an especial type member that hosts a pointer ``data``, a ``size`` and a ``capacity`` value. The pointer should have memory for storing up to ``capacity`` values and ``size`` member shows how many element are currently in the sequence. Usually in micro-ROS, the user is in charge of assigning memory and values to this sequence members.

In the case of the previous example ``MyType.msg``, the ``values`` sequence member is represented in C99 as this struct:

.. code-block:: c

    typedef struct rosidl_runtime_c__int32__Sequence
    {
        int32_t* data;    /* The pointer to an array of int32 */
        size_t size;      /* The number of valid items in data */
        size_t capacity;  /* The number of allocated items in data */
    } rosidl_runtime_c__int32__Sequence;

In that sense, a developer that instantiate a ``mypackage__msg__MyType mymsg;`` variable, should ensure that ``mymsg.values.data`` has memory for storing up to ``mymsg.values.capacity`` values and ``mymsg.values.size`` shows how many element are currently in the sequence, as shown in the following example:

.. code-block:: c

    mypackage__msg__MyType mymsg;

    // mymsg.values.data is NULL or garbage now
    // mymsg.values.size is 0 or garbage now
    // mymsg.values.capacity is 0 or garbage now

    // Assigning dynamic memory to the sequence
    mymsg.values.capacity = 100;
    mymsg.values.data = (int32_t*) malloc(mymsg.values.capacity * sizeof(int32_t));
    mymsg.values.size = 0;

    // Assigning static memory to the sequence
    static int32_t memory[100];
    mymsg.values.capacity = 100;
    mymsg.values.data = memory;
    mymsg.values.size = 0;

    // Filling some data
    for(int32_t i = 0; i < 3; i++){
        mymsg.values.data = i;
        mymsg.values.size++;
    }

Compound types
^^^^^^^^^^^^^^

When dealing with a compound type, the user should recursively inspect the types in order to determine how to handle each internal member.

In the former ``MyType.msg`` example, the ``header`` member has the following structure:

.. code-block:: c

    typedef struct std_msgs__msg__Header
    {
        builtin_interfaces__msg__Time stamp;
        rosidl_runtime_c__String frame_id;
    } std_msgs__msg__Header;


It is important to note that ``rosidl_runtime_c__String`` is equivalent to ``rosidl_runtime_c__char__Sequence``.
On its side, ``builtin_interfaces__msg__Time`` looks like:

.. code-block:: c

    typedef struct builtin_interfaces__msg__Time
    {
        int32_t sec;
        uint32_t nanosec;
    } builtin_interfaces__msg__Time;

Given that, in order to initialize the ``header`` member of ``MyType.msg`` the following code is required:

.. code-block:: c

    mypackage__msg__MyType mymsg;

    // Assigning dynamic memory to the frame_id char sequence
    mymsg.header.frame_id.capacity = 100;
    mymsg.header.frame_id.data = (char*) malloc(mymsg.values.capacity * sizeof(char));
    mymsg.header.frame_id.size = 0;

    // Assigning value to the frame_id char sequence
    strcpy(mymsg.header.frame_id.data, "Hello World");
    mymsg.header.frame_id.size = strlen(mymsg.header.frame_id.data);

    // Assigning value to other members
    mymsg.stamp.sec = 10;
    mymsg.stamp.nanosec = 20;

Sequences of compound types
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Users should take into account that sequence type member of compound type member are also valid ROS 2 type.
For example, a complex ``.msg`` can be described as follows:

.. code-block::

    # MyComplexType.msg
    std_msgs/Header[] multiheaders
    int32[] values
    float64 duration
    int8[10] coefficients
    string name

In this case, the generated structure will be:

.. code-block:: c

    typedef struct mypackage__msg__MyComplexType
    {
    std_msgs__msg__Header__Sequence multiheaders;
    rosidl_runtime_c__int32__Sequence values;
    double duration;
    int8 coefficients[10];
    rosidl_runtime_c__String name;  // equal to rosidl_runtime_c__char__Sequence
    } mypackage__msg__MyComplexType;

In this case ``multiheaders`` is a **sequence type of compound type member**.
It shall be handled correctly and recursively by the user, as in the following example:

.. code-block:: c

    mypackage__msg__MyComplexType mymsg;

    // Init the multiheaders sequence
    mymsg.multiheaders.capacity = 10;
    mymsg.multiheaders.data = (std_msgs__msg__Header*) malloc(mymsg.values.capacity * sizeof(std_msgs__msg__Header));
    mymsg.multiheaders.size = 0;

    // Filling some data
    for(int32_t i = 0; i < 3; i++){
        mymsg.values.data = i;

        // Add memory to this sequence element frame_id
        mymsg.multiheaders.data[i].frame_id.capacity = 100;
        mymsg.multiheaders.data[i].frame_id.data = (char*) malloc(mymsg.multiheaders.data[i].frame_id.capacity * sizeof(char));
        mymsg.multiheaders.data[i].frame_id.size = 0;

        // Assigning value to the frame_id char sequence
        strcpy(mymsg.multiheaders.data[i].frame_id.data, "Hello World");
        mymsg.multiheaders.data[i].frame_id.size = strlen(mymsg.multiheaders.data[i].frame_id.data);

        // Assigning value to other members
        mymsg.multiheaders.data[i].stamp.sec = 10;
        mymsg.multiheaders.data[i].stamp.nanosec = 20;

        mymsg.multiheaders.size++;
    }
