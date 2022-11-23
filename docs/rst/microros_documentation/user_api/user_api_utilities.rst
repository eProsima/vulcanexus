.. _tutorials_micro_utilities:

micro-ROS Utilities
===================

.. contents:: Table of Contents
    :depth: 1
    :local:
    :backlinks: none

`micro_ros_utilities <https://github.com/micro-ROS/micro_ros_utilities>`_ is a package that provides a set of tools for easing the use of micro-ROS on different platforms. Currently, it has two `APIs <https://micro.ros.org/docs/api/utils/>`_:

- micro-ROS type utils.
- micro-ROS ``rosidl_runtime_c__String`` wrapper that reduces dynamic memory operations.

micro-ROS String Utilities
--------------------------

This API helps developers to manage strings in micro-ROS by means of providing a set of methods that allow initialization, destruction, set, and other common operations.

.. note::

    A full example can be found on how to use this API can be found `here <https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_types_handling/micro-ros_types_handling.ino>`_

.. warning::

    It is important to note that, unlike in ROS 2 rclcpp, in micro-ROS rclc memory in types is not initialized by default. It is required that the user initializes the memory for each type that is used in any micro-ROS procedure.

micro_ros_string_utilities_init
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a ``rosidl_runtime_c__String`` from a ``char`` pointer.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

.. list-table::
   :header-rows: 1

   * - Operation
     - Action
   * - Allocate
     - Yes, it does not take ownership of ``char*``
   * - Reallocate
     - No
   * - Free
     - No

micro_ros_string_utilities_init_with_size
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a ``rosidl_runtime_c__String`` from a size.

.. code-block:: c

    size_t size = 10;
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init_with_size(size);

.. list-table::
   :header-rows: 1

   * - Operation
     - Action
   * - Allocate
     - Yes
   * - Reallocate
     - No
   * - Free
     - No

micro_ros_string_utilities_set
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a ``rosidl_runtime_c__String`` from a ``char`` pointer reallocating an actual ``rosidl_runtime_c__String``.

.. code-block:: c

    const char * str = "Hello World";
    size_t size = strlen(str);

    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init_with_size(size);
    ros_str = micro_ros_string_utilities_set(ros_str, str);

.. list-table::
   :header-rows: 1

   * - Operation
     - Action
   * - Allocate
     - No
   * - Reallocate
     - Reallocates input ``rosidl_runtime_c__String``, does not take ownership of ``char*``
   * - Free
     - No

micro_ros_string_utilities_get_c_str
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Returns the char pointer to the ``rosidl_runtime_c__String`` data.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

    char * ptr_str = micro_ros_string_utilities_get_c_str(ros_str);

micro_ros_string_utilities_append
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Appends a char pointer to the end of a ``rosidl_runtime_c__String``.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

    ros_str = micro_ros_string_utilities_append(ros_str, "!");

.. list-table::
   :header-rows: 1

   * - Operation
     - Action
   * - Allocate
     - No
   * - Reallocate
     - Yes if input ``rosidl_runtime_c__String`` is not big enough
   * - Free
     - No

micro_ros_string_utilities_remove_tail_chars
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Removes characters from the end of a ``rosidl_runtime_c__String``.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

    ros_str = micro_ros_string_utilities_remove_tail_chars(ros_str, 5);

.. list-table::
   :header-rows: 1

   * - Operation
     - Action
   * - Allocate
     - No
   * - Reallocate
     - No
   * - Free
     - No

micro_ros_string_utilities_destroy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Destroys a ``rosidl_runtime_c__String``.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

    micro_ros_string_utilities_destroy(ros_str);

.. list-table::
   :header-rows: 1

   * - Operation
     - Action
   * - Allocate
     - No
   * - Reallocate
     - No
   * - Free
     - Yes


micro-ROS Types Utilities
-------------------------

This API helps developers to manage ROS types in micro-ROS. It handles the types structures recursively in order to initialize each member with the required memory size.

.. note::

    A full example can be found on how to use this API can be found `here <https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_types_handling/micro-ros_types_handling.ino>`_

micro_ros_utilities_memory_conf_t
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``micro_ros_utilites`` provides a functionality to instantiate sequences and strings of fixed sizes.

Memory can be allocated in two ways:

- *statically*: in an user provided buffer.
- *dynamically*: using ROS 2 allocators

Memory allocation can be configured using configuration structure that has the following members:

- ``max_string_capacity``: Maximum string capacity to use for message fields in case they don’t have a custom rule assigned to them.
- ``max_ros2_type_sequence_capacity``: Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain ROS 2 msg types, in case they don’t have a custom rule assigned to them.
- ``max_basic_type_sequence_capacity``: Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain basic types (ie: primitive field types), in case they don’t have a custom rule assigned to them.

.. code-block:: c

    static micro_ros_utilities_memory_conf_t conf = {0};

    // OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

micro_ros_utilities_type_info
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Returns a ``rosidl_runtime_c__String`` with the type introspection data.

.. code-block:: c

    #include <control_msgs/msg/joint_jog.h>

    control_msgs__msg__JointJog msg;
    rosidl_runtime_c__String ros_str = micro_ros_utilities_type_info(ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog));

micro_ros_utilities_get_static_size
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Returns the static memory size that will be used for a type.

.. code-block:: c

    #include <control_msgs/msg/joint_jog.h>

    static micro_ros_utilities_memory_conf_t conf = {0};

    // OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

    control_msgs__msg__JointJog msg;
    rosidl_runtime_c__String ros_str = micro_ros_utilities_get_static_size(ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog) conf);

micro_ros_utilities_create_message_memory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Allocates dynamic memory for a message.

.. code-block:: c

    #include <control_msgs/msg/joint_jog.h>

    static micro_ros_utilities_memory_conf_t conf = {0};

    // OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

    control_msgs__msg__JointJog msg;
    bool success = micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
        &msg,
        conf
    );

micro_ros_utilities_create_static_message_memory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Allocates memory for a message in a user-provided buffer.

.. code-block:: c

    #include <control_msgs/msg/joint_jog.h>

    uint8_t my_buffer[1000];
    static micro_ros_utilities_memory_conf_t conf = {0};

    // OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

    control_msgs__msg__JointJog msg;
    bool success = micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
        &msg_static,
        conf,
        my_buffer,
        sizeof(my_buffer)
    );

micro_ros_utilities_destroy_message_memory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Deallocates the dynamic memory of a message.

.. code-block:: c

    #include <control_msgs/msg/joint_jog.h>

    uint8_t my_buffer[1000];
    static micro_ros_utilities_memory_conf_t conf = {0};

    // OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

    control_msgs__msg__JointJog msg;
    bool success = micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
        &msg,
        conf
    );

    success &= micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
        &msg,
        conf
    );
