.. _tutorials_micro_user_api_nodes:

Nodes
=======================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none

ROS 2 nodes are the main participants on ROS 2 ecosystem. They will communicate between each other using publishers, subscriptions, services, etc.
Further information about ROS 2 nodes can be found on :ref:`Understanding nodes <ROS2Nodes>`.

Initialization
^^^^^^^^^^^^^^

Before a node is created, a ``rclc_support_t`` object needs to be created and initialized. The ``rclc_support_init`` method will handle micro-ROS initial configuration (memory initialization, transport configuration, ...).

.. note::

  The ``rclc_support_init`` function will try to establish communication with the Agent and will fail if its not reachable on a configurable time.
  A good practice is to the ping the Agent to check the connection before calling this method.

..   TODO(pgarrido): add here a link to the handling reconnection tutorial

- Initialize micro-ROS and create a node with default configuration:

  .. code-block:: c

    // Get configured allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize support object
    rclc_support_t support;
    rclc_support_init(&support, argc, argv, &allocator);

    // Node name
    const char * node_name = "test_node";

    // Node namespace (Can remain empty: "")
    const char * namespace = "test_namespace";

    // Init default node
    rcl_node_t node;
    rclc_node_init_default(&node, node_name, namespace, &support);

- Configure the node ``Domain ID``:

  ROS2 allows isolation of nodes through the ``ROS_DOMAIN_ID`` concept. Nodes can only discover and communicate nodes placed on the same domain.
  Further information about ROS 2 Domain ID can be found on :ref:`About Domain ID <DomainID>`.

  A node can be configured using the ``rclc_support_t`` internal ``rcl_init_options_t`` options structure.
  rcl provides the ``rcl_init_options_set_domain_id`` method to set a node domain. Example:

  .. code-block:: c

    // Get configured allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize and modify options (Set DOMAIN ID to 10)
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    // Initialize rclc support object with custom options
    rclc_support_t support;
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Node name
    const char * node_name = "test_node";

    // Node namespace (Can remain empty "")
    const char * namespace = "test_namespace";

    // Init node with configured support object
    rcl_node_t node;
    rclc_node_init_default(&node, node_name, namespace, &support);

  .. note::

    The configuration of the node will also be applied to its future elements (publishers, subscribers, services, ...).

Cleaning Up
^^^^^^^^^^^^^^

All entities owned by a node (publishers, subscribers, services, ...) have to be destroyed before the node itself:

.. code-block:: c

  // Destroy created entities (Example)
  rcl_publisher_fini(&publisher, &node);

  // Destroy the node
  rcl_node_fini(&node);

This will delete the node from ROS2 graph, including any generated infrastructure on the agent (if possible) and used memory on the client.
