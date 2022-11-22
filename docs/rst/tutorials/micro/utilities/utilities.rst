.. _tutorials_micro_utilities:

Micro ROS Utilities
===================

.. contents:: Table of Contents
    :depth: 1
    :local:
    :backlinks: none

`micro_ros_utilities <https://github.com/micro-ROS/micro_ros_utilities>`_ it is a package that provides utilities for easing the use of micro-ROS on different platforms. Currently it has two `API <https://micro.ros.org/docs/api/utils/>`_:

- ``micro-ROS`` type initializer based on :ref:`rosidl_typesupport_introspection_c <docs/ros2_documentation/source/Concepts/About-Internal-Interfaces.rst>_`.
- ``micro-ROS`` string type wrapper that reduces dynamic memory operations.

micro-ROS String Utilities
--------------------------

API that helps developer to manage ROS strings in micro-ROS, full example can be found `here <https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_types_handling/micro-ros_types_handling.ino>`_

micro_ros_string_utilities_init
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a ``rosidl_runtime_c__String`` from a char pointer.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

micro_ros_string_utilities_init_with_size
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a ``rosidl_runtime_c__String from a size.`` from a size.

.. code-block:: c

    size_t size = 10;
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init_with_size(size);

micro_ros_string_utilities_set
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a ``rosidl_runtime_c__String`` from a char pointer reallocating an actual ``rosidl_runtime_c__String``.

.. code-block:: c

    const char * str = "Hello World";
    size_t size = strlen(str);

    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init_with_size(size);
    ros_str = micro_ros_string_utilities_set(ros_str, str);

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

micro_ros_string_utilities_remove_tail_chars
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Removes characters from the end of a string.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

    ros_str = micro_ros_string_utilities_remove_tail_chars(ros_str, 5);

micro_ros_string_utilities_destroy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Destroys a ``rosidl_runtime_c__String``.

.. code-block:: c

    const char * str = "Hello World";
    rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

    micro_ros_string_utilities_destroy(ros_str);


micro-ROS Types Utilities
-------------------------

API that helps developer to manage ROS types in micro-ROS, full example can be found `here <https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_types_handling/micro-ros_types_handling.ino>`_

micro_ros_utilities_memory_conf_t
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

micro_ros_utilites provides a functionality to instntiate sequences and strings of fixed sizes.

Memory configuration struct:

- max_string_capacity: Maximum string capacity to use for msg fields in case they don’t have a custom rule assigned to them.
- max_ros2_type_sequence_capacity: Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain ROS 2 msg types, in case they don’t have a custom rule assigned to them.
- max_basic_type_sequence_capacity: Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain basic types (ie: primitive field types), in case they don’t have a custom rule assigned to them.

.. code-block:: c

    static micro_ros_utilities_memory_conf_t conf = {0};

    // OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

micro_ros_utilities_type_info
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Returns a string with the type introspection data.

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

Allocates the dynamic memory required for a message.

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

Allocates the memory required for a message in a user-provided buffer.

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
