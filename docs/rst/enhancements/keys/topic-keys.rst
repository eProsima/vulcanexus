.. _topic_keys:

Topic Keys
==========

Topic Keys endow developers with the ability of orchestrating efficient and tailored data distribution, facilitating streamlined communication and enhancing system scalability.
This exploration unravels their significance, working principles, and how to use them.

Motivation
^^^^^^^^^^

Robotic systems and applications often require modularity in order to divide complex problems into smaller, more manageable parts.
These sub-systems need to model real world objects, maintaining a state of it (e.g. in a robot perception pipeline it could be important to track the pose of an certain object in the environment or to retrieve the status of the base controller in a mobile robot).
Hence, communicating successive updates on the states of these objects is required.
In *Vulcanexus*, this is achieved by exchanging data messages over a topic.

Conceptually, topic instances are a way of multiplexing the transmission of updates of several objects of the same *logical* kind over the same resource, i.e. the topic.
Imagine a scenario where multiple robots are exploring an uncharted territory and are sending their positions to a central control node.
Instead of having multiple topics, one per robot, topic keys allow to have a single topic where each robot sends its status with a unique key (the robot id) that identifies the robot.
That is the power of topic instances.

.. list-table::
   :width: 100%
   :class: borderless

   * - .. image:: ../../figures/enhancements/keys/topic_instances_example.svg
          :width: 100%

     - .. image:: ../../figures/enhancements/keys/topics_example.svg
          :width: 100%

In addition, the use of topic instances report numerous advantages, including:

* Efficient middleware infrastructure, data distribution and resource utilization as the number of entities are dramatically reduced.
* A better history management, meaning that cached updates are managed in a more performant way for certain Quality of Service parameters.

These benefits are further detailed in the :ref:`benefits_of_topic_keys` section.

Background
^^^^^^^^^^
*Vulcanexus* uses :ref:`Fast DDS as middleware <vulcanexus_middleware>`.
One of the core concepts involved in the *Vulcanexus* communication architecture is topics, which serve as the bridge that enables data exchange among nodes within a particular system.

Topics are named communication channels that allow nodes (software modules) in a distributed system (such as a robotic application) to exchange messages with each other.
These messages can contain various types of data, such as sensor readings, control commands, or status updates, enabling different parts of the robot to collaborate and share information.

The following image depicts a simple scenario where two sensors publish data in different topics, and two remote nodes subscribe to it.
The first ``Logging`` node only subscribes to the Sensor 2 output topic, while the ``Controller`` node subscribes to both topics.

.. image:: ../../figures/enhancements/keys/topics.gif
    :align: center
    :width: 70%

When any new data is available in the sensors, it is transmitted through the corresponding topic and all the subscribers attached to that topic receive the message.

In this situation, each of the sensors has a dedicated topic for transmitting its data. This suggests excessive resource usage because the middleware has to create and manage additional entities (topics, publishers, and subscriptions) and can potentially lead to a discovery overhead as the number of sensors scale up.

Understanding Keyed Topics
^^^^^^^^^^^^^^^^^^^^^^^^^^

Now, let's delve into the concept of keyed topics. While traditional topics allow nodes to exchange messages freely, keyed topics add an additional logic of organization to this communication process.

In a keyed topic, every message is associated with a topic instance and each topic instance is identified by a **unique key**.
This key allows nodes to update different states of the same kind.
In this sense, keys can be thought as the "primary key" of a database.
While a topic's message is used to represent the state of a real world object, each unique key corresponds to the specific topic **instance** to which the topic's message belongs.

For example, imagine a robotic system where multiple sensors are generating data about different objects in the environment.
By using keyed topics, each sensor can publish its data in only one topic having a unique key corresponding to the object being observed, allowing for a more efficient resource management.

Following with the example above, the next image illustrates this concept, where the sensors publish data with a unique key (sensor id) corresponding to the object being observed.

.. image:: ../../figures/enhancements/keys/keyed-topics-cft.gif
    :align: center
    :width: 70%

In this case, only one topic and subscriber per node is required to receive the data of interest.

.. _benefits_of_topic_keys:

Benefits of Keyed Topics
^^^^^^^^^^^^^^^^^^^^^^^^

The use of topic keys report several benefits, including:

* Efficient middleware infrastructure and data distribution: the use of keyed topics reduce the amount of entities (subscriptions, publishers, and topics) needed in the data flow of the application which entails a more efficient usage of resources and bandwidth.
* A better history management: *Fast DDS* maintains a cache of recent updates, typically the last 10 updates for each instance.
  This caching mechanism, configured separately for publishers and subscriptions, prevents instances with frequent changes from overriding the latest value of another instance that changes less frequently.
  Moreover, this per-instance cache can be managed by the source application or a persistence service for late-joining subscribers, ensuring they initialize with the current value or recent changes of each object.
  Without keyed topics, the middleware lacks the ability to intelligently cache data on a per-instance basis, resulting in late joiners needing to process a larger history before accessing the current value of a specific instance.
* Easy integration with databases: individual records in a table are uniquely identified by the values of the fields marked as the “primary key” for that table.
  If those same fields are also marked as the key fields for the corresponding Topic, then the integration is seamless and can work hand-in-hand with the database table storage.
* Improved message filtering: by combining topic keys with `Content Filter Topic <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/contentFilteredTopic/contentFilteredTopic.html>`_, nodes have the ability to subscribe selectively to messages by utilizing their keys, allowing for precise filtering and minimizing superfluous message handling.
  Please consult the :ref:`vulcanexus_filtered_topic_keys_tutorial` for practical guidance.

In summary, keyed topics enhance the flexibility, efficiency, and organization of messages exchange.
By leveraging keyed topics, developers can design more robust and scalable robotic applications capable of handling diverse data sources and complex environments.

Using Topic Keys
^^^^^^^^^^^^^^^^

For details on how to use topic keys in *Vulcanexus*, please refer to the documented tutorials in this regard:

  * :ref:`topic_keys_tutorial`
  * :ref:`vulcanexus_filtered_topic_keys_tutorial`

