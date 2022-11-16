.. _tutorials_micro_custom_types:

Creating custom types
=====================

.. contents:: Table of Contents
    :depth: 1
    :local:
    :backlinks: none


This tutorial aims at providing step-by-step guidance for those users interested in adding custom message definition to the micro-ROS build. Those instructions can be used to extend the type definition for topics, services and actions provided by ROS 2 and micro-ROS.

Further information can be found in :ref:`Implementing custom interfaces <SinglePkgInterface>`.

.. note::

    This tutorial starts from a previously created micro-ROS environment. Check the first steps of `Getting started micro-ROS <https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/getting_started/getting_started.html>`_ for instructions on how to create a micro-ROS environment for embedded platforms.

    For creating custom types in a :ref:`Build System Component <micro_build_system_components>`, check the instructions in each component repository. For example `Renesas e2 Studio Readme <https://github.com/micro-ROS/micro_ros_renesas2estudio_component#adding-custom-packages-to-the-micro-ros-build>`_ or `ST Micro ST Cube IDE/MX <https://github.com/micro-ROS/micro_ros_stm32cubemx_utils#adding-custom-packages>`_.

Simple types
------------

Once the micro-ROS workspace is created, navigate to ``firmware/mcu_ws`` and create a new package for the custom messages:

.. code-block:: bash

  cd firmware/mcu_ws
  ros2 pkg create --build-type ament_cmake my_custom_message
  cd my_custom_message
  mkdir msg
  touch msg/MyCustomMessage.msg

In the generated ``CMakeLists.txt`` file the following lines shall be added just before ``ament_package()`` line:

.. code-block:: text

  ...
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MyCustomMessage.msg"
   )
  ...

In the generated ``package.xml`` file the following lines shall be added:

.. code-block:: xml

  ...
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  ...

The content of the ``msg/MyCustomMessage.msg`` file contains the message definition. For example:

.. code-block:: text

  bool bool_test
  byte byte_test
  char char_test
  float32 float32_test
  float64 double_test
  int8 int8_test
  uint8 uint8_test
  int16 int16_test
  uint16 uint16_test
  int32 int32_test
  uint32 uint32_test
  int64 int64_test
  uint64 uint64_test

.. note::

    Same approach can be taken with ``.srv`` and ``.action`` files for creating types for micro-ROS :ref:`services<micro_user_api_services>` and :ref:`actions <micro_user_api_actions>`.

Now, the micro-ROS workspace workspace can be built as usual. As explained in `Getting started micro-ROS <https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/getting_started/getting_started.html>`_, the ``ros2 run micro_ros_setup build_firmware.sh`` command will build all packages located inside ``mcu_ws``. In the micro-ROS application code the new message type can be used as usual:

.. code-block:: c

  #include <my_custom_message/msg/my_custom_message.h>

  ...

  my_custom_message__msg__MyCustomMessage msg = {0};

  msg.byte_test = 3;
  msg.uint32_test = 42;

  ...

  rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, MyCustomMessage), "my_custom_publisher");
  rcl_publish(&publisher, &msg, NULL);

  ...

Type composition
----------------

It is possible to create custom types that compose members from another ROS 2 message type packages. For example a member with type ``Point32`` from the ROS 2 package ``geometry_msgs``.

First of all, the dependency shall also be included in  in the ``CMakeLists.txt``:

.. code-block:: text

  ...
  find_package(rosidl_default_generators REQUIRED)
  find_package(geometry_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MyCustomMessage.msg"
    DEPENDENCIES geometry_msgs
   )
  ...

The dependency shall be included in ``package.xml``:

.. code-block:: xml

  ...
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <depend>geometry_msgs</depend>
  ...

At this point, message definition in ``msg/MyCustomMessage.msg`` can now include types from the ``geometry_msgs`` package:

.. code-block:: text

  ...
  int64 int64_test
  uint64 uint64_test
  geometry_msgs/Point32 point32_test

And finally, the new member can be accessed in the custom type:

.. code-block:: c

  #include <my_custom_message/msg/my_custom_message.h>

  ...

  my_custom_message__msg__MyCustomMessage msg;

  msg.byte_test = 3;
  msg.uint32_test = 42;

  msg.point32_test.x = 1.23;
  msg.point32_test.y = 2.31;
  msg.point32_test.z = 3.12;

  ...

.. TODO(pgarrido): add link here to memory handling tutorial and a reference for initializing the created types

