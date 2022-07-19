.. _tutorials_micro_user_api:

micro-ROS User API
=======================

In this section, you’ll learn the basics of the micro-ROS C API: ``rclc``. This API implementation is based on the ROS 2 client support library (rcl), enriched with a set of convenience functions by the package rclc.
That is, rclc does not add a new layer of types on top of rcl (like rclcpp and rclpy do) but only provides functions that ease the programming with the rcl types. New types are introduced only for concepts that are missing in rcl, such as the concept of an executor.

This API implements the embedded version of the complete ROS 2 communications paradigm:

.. toctree::
    :maxdepth: 1

    user_api_nodes.rst
    user_api_pub_sub.rst
    user_api_services.rst
    user_api_executor_timers.rst

.. TODO(acuadros95): Move ROS2 documentation references to future Vulcanexus tutorials.
.. TODO(acuadros95): Modify 'Handling messages memory tutorial' reference for vulcanexus future micro tutorial.
