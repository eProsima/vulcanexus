.. _micro_user_api_qos:

QoS configuration
=================

QoS (Quality of Services) configuration related to DDS communication can also applied to micro-ROS entities, as its default middleware `eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_ allows for complete customization using the provided RMW API.

Further information about ROS 2 available QoS can be found on :ref:`About Quality of Service settings <concepts_about_qos>`.

Initialization
^^^^^^^^^^^^^^

As shown in the initialization API of each entity, a ``rmw_qos_profile_t`` can be used to configure the QoS of a micro-ROS entity:

.. code-block:: c

  // Create a publisher with depth 100.
  rcl_publisher_t custom_publisher;
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  custom_qos.depth = 100;
  rclc_publisher_init(
      &custom_publisher, &node,
      &type_support, &topic_name, &custom_qos);

This ``rmw_qos_profile_t`` structure is provided by ROS 2 RMW headers: `rmw_qos_profile_t <http://docs.ros2.org/latest/api/rmw/structrmw__qos__profile__t.html>`_. A list with predefined qos profiles is also available: `qos_profiles.h <https://github.com/ros2/rmw/blob/kilted/rmw/include/rmw/qos_profiles.h>`__.

.. note::

  By default, the following qos configuration is applied on each entity:

    - Default Publisher/subscriber: `rmw_qos_profile_default <https://github.com/ros2/rmw/blob/kilted/rmw/include/rmw/qos_profiles.h#L51>`__.
    - Best effort Publisher/subscriber : `rmw_qos_profile_sensor_data <https://github.com/ros2/rmw/blob/kilted/rmw/include/rmw/qos_profiles.h#L25>`__.
    - Default Service/client: `rmw_qos_profile_services_default <https://github.com/ros2/rmw/blob/kilted/rmw/include/rmw/qos_profiles./h#L64>`__.
    - Best effort Service/client: `rmw_qos_profile_services_default <https://github.com/ros2/rmw/blob/kilted/rmw/include/rmw/qos_profiles.h#L64>`__ with ``reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT``

History behaviour
^^^^^^^^^^^^^^^^^

When micro-ROS history slots are complete and new data arrives, the behaviour is determined by the configured ``history`` kind and ``depth``.
More details about memory configuration can be found on the :ref:`Middleware related memory tutorial <micro_ros_middleware_memory>`.

- ``RMW_QOS_POLICY_HISTORY_KEEP_LAST``: New data will be stored on free history slots up to the configured depth value. If the entity already owns up to depth slots, a the oldest message will be freed and use for the received data.

- ``RMW_QOS_POLICY_HISTORY_KEEP_ALL``: New data will always be stored on free history slots up to the configured depth value. If the entity already owns up to depth slots, the new message will be discarded.

- There is a special case for ``depth = 0``, where the history kind will be ignored and history slots wont be reused before they are released, following the keep all approach.

.. note::

  History kind types not listed on this section are treated as ``RMW_QOS_POLICY_HISTORY_KEEP_LAST``.

