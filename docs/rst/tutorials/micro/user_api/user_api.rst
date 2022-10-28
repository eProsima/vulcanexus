.. _tutorials_micro_user_api:

micro-ROS User API
=======================

In this section, you’ll learn the basics of the micro-ROS C API: ``rclc``. This API implementation is based on the ROS 2 client support library (RCL), enriched with a set of convenience functions by the package RCLC.
That is, RCLC does not add a new layer of types on top of rcl (like RCLCPP and RCLPY do) but only provides functions that ease the programming with the RCL types. New types are introduced only for concepts that are missing in RCL, such as the concept of an executor.

This API implements an lightweight embedded version of the complete ROS 2 communications paradigm:

.. toctree::
    :maxdepth: 1

    user_api_nodes.rst
    user_api_pub_sub.rst
    user_api_services.rst
    user_api_executor_timers.rst
    user_api_actions.rst
    user_api_parameter_server.rst
    user_api_middleware.rst
