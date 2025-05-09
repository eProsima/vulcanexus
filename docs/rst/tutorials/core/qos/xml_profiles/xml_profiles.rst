.. include:: ../../../../exports/alias.include

.. _tutorials_xml_profiles_intro:

Configuring Fast-DDS QoS via XML profiles
=========================================

Vulcanexus offers the possibility of fully configuring Fast DDS' QoS policies through XML profile definition.
This can be done in several ways depending on whether the application used is a native Fast DDS application or a Vulcanexus application using Fast DDS's RMW implementation.
This guide will focus on the latter.
For more information regarding XML profiles in Fast DDS please refer to the `XML configuration page <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_
from Fast DDS's documentation.

.. _vulcanexus_configure:

Configuring *Fast DDS* in Vulcanexus for ROS 2 applications
-----------------------------------------------------------

ROS 2 only allows for the configuration of certain middleware QoS (see :ref:`ROS 2 QoS policies <concepts_about_qos>`).
However, *rmw_fastrtps* offers extended configuration capabilities to take full advantage of all the features available in *Fast DDS*.
This section describes how to specify this extended configuration.

.. _vulcanexus_configure_publication_mode:

Changing publication mode
-------------------------

*rmw_fastrtps* in Vulcanexus uses asynchronous publication by default.
This can be changed setting the environment variable ``RMW_FASTRTPS_PUBLICATION_MODE`` to one of the following allowed values:

* **ASYNCHRONOUS**: asynchronous publication mode.
  Setting this mode implies that when the publisher invokes the write operation, the data is copied into a queue, a background thread (asynchronous thread) is notified about the addition to the queue, and control of the thread is returned to the user before the data is actually sent.
  The background thread is in charge of consuming the queue and sending the data to every matched reader.
* **SYNCHRONOUS**: synchronous publication mode.
  Setting this mode implies that the data is sent directly within the context of the user thread.
  This entails that any blocking call occurring during the write operation would block the user thread, thus preventing the application from continuing its operation.
  It is important to note that this mode typically yields higher throughput rates at lower latencies, since there is no notification nor context switching between threads.
* **AUTO**: let Fast DDS select the publication mode.
  This implies using the publication mode set in the :ref:`XML file <vulcanexus_configure_xml>`, or otherwise, the default value set in Fast DDS (see |PublishModeQosPolicy|).

*rmw_fastrtps* defines two configurable parameters in addition to :ref:`ROS 2 QoS policies <concepts_about_qos>`.
Said parameters, and their default values under ROS 2, are:

.. list-table::
   :header-rows: 1
   :align: left

   * - Parameter
     - Description
     - Default ROS 2 value
   * - |MemoryManagementPolicy|
     - *Fast DDS* preallocates memory for the publisher |br|
       and subscriber histories. When those histories fill |br|
       up, a reallocation occurs to reserve more memory.
     - |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|
   * - |PublishModeQosPolicy|
     - User calls to publication method add the messages |br|
       in a queue that is managed in a different thread, |br|
       meaning that the user thread is available right |br|
       after the call to send data.
     - |ASYNCHRONOUS_PUBLISH_MODE-api|


.. _vulcanexus_configure_xml:

XML configuration
-----------------

To use specific *Fast-DDS* features within a Vulcanexus application, XML configuration files can be used to configure a wide set of *QoS*.
Please refer to |XML_profiles| for Fast DDS native applications in Fast DDS's documentation to see the whole list of configuration options available in *Fast DDS*.

When configuring *rmw_fastrtps* using XML files, there are certain points that have to be taken into account:

* QoS contained in `rmw_qos_profile_t <http://docs.ros2.org/latest/api/rmw/structrmw__qos__profile__t.html>`_are always honored, unless set to ``*_SYSTEM_DEFAULT``.
  In that case, XML values, or Fast DDS default values in the absences of XML ones, are applied.
  This means that if any QoS in ``rmw_qos_profile_t`` is set to something other than ``*_SYSTEM_DEFAULT``, the corresponding value in the XML is ignored.

* By default, *rmw_fastrtps* overrides the values for |MemoryManagementPolicy| and |PublishModeQosPolicy|.
  This means that the values configured in the XML for these two parameters will be ignored.
  Instead, |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api| and |ASYNCHRONOUS_PUBLISH_MODE-api| are used respectively.

* The overriding of MemoryManagementPolicy and PublishModeQosPolicy can be avoided by setting the environment variable ``RMW_FASTRTPS_USE_QOS_FROM_XML`` to ``1``
  (its default value is ``0``).
  This will make *rmw_fastrtps* use the values defined in the XML for MemoryManagementPolicy and PublishModeQosPolicy.
  Bear in mind that setting this environment variable but not setting these policies in the XML results in using the default values in *Fast DDS*.
  These are different from the aforementioned *rmw_fastrtps* default values (see |MemoryManagementPolicy| and |PublishModeQosPolicy|).
  In general, setting ``RMW_FASTRTPS_USE_QOS_FROM_XML`` effectively overrides whatever configuration was set with ``RMW_FASTRTPS_PUBLICATION_MODE``, setting the publication mode to the value specified in the XML, or to the *Fast DDS* default publication mode if none is set in the XML.


The following table summarizes which values are used or ignored according to the configured variables:

.. list-table::
   :header-rows: 1
   :align: left

   * - RMW_FASTRTPS_USE_QOS_FROM_XML
     - ``rmw_qos_profile_t``
     - Fast DDS XML QoS
     - Fast DDS XML history memory policy |br|
       and publication mode
   * - 0 (default)
     - Default values
     - Overridden by |br|
       ``rmw_qos_profile_t``
     - Overridden by |br|
       *rmw_fastrtps* default value
   * - 0 (default)
     - Non system default
     - overridden by |br|
       ``rmw_qos_profile_t``
     - Overridden by |br|
       *rmw_fastrtps* default value
   * - 0 (default)
     - System default
     - Used
     - Overridden by |br|
       *rmw_fastrtps* default value
   * - 1
     - Default values
     - Overridden by |br|
       ``rmw_qos_profile_t``
     - Used
   * - 1
     - Non system default
     - Overridden by |br|
       *rmw_qos_profile_t*
     - Used
   * - 1
     - System default
     - Used
     - Used

XML configuration file location
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

There are two possibilities for providing *Fast DDS* with XML configuration files:

* **Recommended**: Setting the location with environment variable ``FASTRTPS_DEFAULT_PROFILES_FILE`` to contain the path to the XML configuration file (see `Environment Variables <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#env-vars>`_).

.. code-block:: bash

      export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>

* **Alternative**: Placing the XML file in the running application directory under the name *DEFAULT_FASTRTPS_PROFILES.xml*.

For example:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
    ros2 run <package> <application>


Applying different profiles to different entities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*rmw_fastrtps* allows for the configuration of different entities with different QoS using the same XML file.
For doing so, *rmw_fastrtps* locates profiles in the XML based on topic names.

.. _vulcanexus_configure_pubsub_profile_names:

Creating publishers/subscribers with different profiles
.......................................................

* To configure a publisher, define a ``<data_writer>`` profile with attribute ``profile_name=topic_name``, where ``topic_name`` is the name of the topic prepended by the node namespace (which defaults to "" if not specified), i.e. the node's namespace followed by topic name used to create the publisher.
  Mind that topic names always start with ``/`` (it is added when creating the topic if not present), and that namespace and topic name are always separated by one ``/``.
  If such profile is not defined, *rmw_fastrtps* attempts to load the ``<data_writer>`` profile with attribute ``is_default_profile="true"``.

* To configure a subscriber, define a ``<data_reader>`` profile with attribute ``profile_name=topic_name``, where ``topic_name`` is the name of the topic prepended by the node namespace (which defaults to "" if not specified), i.e. the node's namespace followed by topic name used to create the subscriber.
  Mind that topic names always start with ``/`` (it is added when creating the topic if not present), and that namespace and topic name are always separated by one ``/``.
  If such profile is not defined, *rmw_fastrtps* attempts to load the ``<data_reader>`` profile with attribute ``is_default_profile="true"``.

The following table presents different combinations of node namespaces and user specified topic names, as well as the resulting topic names and the suitable profile names:

+---------------------------+-----------------+-------------------------+-------------------------+
| User specified topic name | Node namespace  | Final topic name        | Profile name            |
+===========================+=================+=========================+=========================+
| chatter                   | DEFAULT ("")    | /chatter                | /chatter                |
+---------------------------+-----------------+-------------------------+-------------------------+
| chatter                   | test_namespace  | /test_namespace/chatter | /test_namespace/chatter |
+---------------------------+-----------------+-------------------------+-------------------------+
| chatter                   | /test_namespace | /test_namespace/chatter | /test_namespace/chatter |
+---------------------------+-----------------+-------------------------+-------------------------+
| /chatter                  | test_namespace  | /chatter                | /chatter                |
+---------------------------+-----------------+-------------------------+-------------------------+
| /chatter                  | /test_namespace | /chatter                | /chatter                |
+---------------------------+-----------------+-------------------------+-------------------------+

.. important::

    Node namespaces are NOT prepended to user specified topic names starting with /, a.k.a Fully Qualified Names (FQN).
    For a complete description of topic name remapping please refer to
    `Remapping Names <http://design.ros2.org/articles/static_remapping.html>`_.

Creating services with different profiles
.........................................

ROS 2 services contain a subscriber for receiving requests, and a publisher to reply to them.
*rmw_fastrtps* allows for configuring each of these endpoints separately in the following manner:

* To configure the request subscriber, define a ``<data_reader>`` profile with attribute ``profile_name=topic_name``, where ``topic_name`` is the name of the service after mangling.
  For more information on name mangling, please refer to `Topic and Service name mapping to DDS <https://design.ros2.org/articles/topic_and_service_names.html>`_.
  If such profile is not defined, *rmw_fastrtps* attempts to load a ``<data_reader>`` profile with attribute ``profile_name="service"``.
  If neither of the previous profiles exist, *rmw_fastrtps* attempts to load the ``<data_reader>`` profile with attribute ``is_default_profile="true"``.

* To configure the reply publisher, define a ``<data_writer>`` profile with attribute ``profile_name=topic_name``, where ``topic_name`` is the name of the service after mangling.
  If such profile is not defined, *rmw_fastrtps* attempts to load a ``<data_writer>`` profile with ``attribute profile_name="service"``.
  If neither of the previous profiles exist, *rmw_fastrtps* attempts to load the ``<data_writer>`` profile with attribute ``is_default_profile="true"``.


Creating clients with different profiles
........................................

ROS 2 clients contain a publisher to send requests, and a subscription to receive the service's replies.
*rmw_fastrtps* allows for configuring each of these endpoints separately in the following manner:

* To configure the requests publisher, define a ``<data_writer>`` profile with attribute ``profile_name=topic_name``, where ``topic_name`` is the name of the service after mangling.
  If such profile is not defined, *rmw_fastrtps* attempts to load a ``<data_writer>`` profile with attribute ``profile_name="client"``.
  If neither of the previous profiles exist, *rmw_fastrtps* attempts to load the ``<data_writer>`` profile with attribute ``is_default_profile="true"``.

* To configure the reply subscription, define a ``<data_reader>`` profile with ``attribute profile_name=topic_name``, where ``topic_name`` is the name of the service after mangling.
  If such profile is not defined, *rmw_fastrtps* attempts to load a ``<data_reader>`` profile with attribute ``profile_name="client"``.
  If neither of the previous profiles exist, *rmw_fastrtps* attempts to load the ``<data_reader>`` profile with attribute ``is_default_profile="true"``.


.. _vulcanexus_configure_xml_creatingROScontext_nodes:

Creating ROS contexts and nodes
...............................

ROS *context* and *node* entities are mapped to *Fast DDS* Participant entity, according to the following table:


+--------------+--------------------------------------+----------------------------------------+
| ROS entity   | *Fast DDS* entity since *Foxy*       | *Fast DDS* entity in *Eloquent & below*|
+==============+======================================+========================================+
| Context      | Participant                          | *Not DDS direct mapping*               |
+--------------+--------------------------------------+----------------------------------------+
| Node         | *Not DDS direct mapping*             | Participant                            |
+--------------+--------------------------------------+----------------------------------------+

This means that on *Foxy* and later releases, contexts can be configured using a ``<Participant>`` profile with attribute ``is_default_profile="true"``.
The same profile will be used in *Eloquent* and below to configure nodes.

For example, a profile for a ROS 2 context on *Foxy* and later releases would be specified as:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="participant_profile_ros2" is_default_profile="true">
            <rtps>
                <name>profile_for_ros2_context</name>
            </rtps>
        </participant>
    </profiles>

.. _vulcanexus_example:

Example
-------

The following example uses both the ROS 2 `demo_nodes_cpp` package's talker/listener demo and the client/service demo, configuring *Fast DDS* to publish synchronously, and to have dynamically allocated publisher and subscriber histories.

Create a XML file `vulcanexus_example.xml` and save it in `path/to/xml/`

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

        <participant profile_name="participant_profile_ros2" is_default_profile="true">
            <rtps>
                <name>profile_for_ros2_context</name>
            </rtps>
        </participant>

        <!-- Default publisher profile -->
        <data_writer profile_name="default publisher profile" is_default_profile="true">
            <qos>
                <publishMode>
                    <kind>SYNCHRONOUS</kind>
                </publishMode>
            </qos>
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </data_writer>

        <!-- Publisher profile for topic helloworld -->
        <data_writer profile_name="helloworld">
            <qos>
                <publishMode>
                    <kind>SYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </data_writer>

        <!-- Request subscriber profile for services -->
        <data_reader profile_name="service">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </data_reader>

        <!-- Request publisher profile for clients -->
        <data_writer profile_name="client">
            <qos>
                <publishMode>
                    <kind>ASYNCHRONOUS</kind>
                </publishMode>
            </qos>
        </data_writer>

        <!-- Request subscriber profile for server of service "add_two_ints" -->
        <data_reader profile_name="rq/add_two_intsRequest">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </data_reader>

        <!-- Reply subscriber profile for client of service "add_two_ints" -->
        <data_reader profile_name="rr/add_two_intsReply">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </data_reader>

    </profiles>

For the talker/listener demo application.

#. Open one terminal and run:

   .. code-block:: bash

       export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/xml/vulcanexus_example.xml
       export RMW_FASTRTPS_USE_QOS_FROM_XML=1
       ros2 run demo_nodes_cpp talker

#. Open one terminal and run:

   .. code-block:: bash

       export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/xml/vulcanexus_example.xml
       export RMW_FASTRTPS_USE_QOS_FROM_XML=1
       ros2 run demo_nodes_cpp listener

The same approach can be used when running the add_two_ints client/service demo application.

#. Open one terminal, run:

   .. code-block:: bash

      # Client
       export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/xml/vulcanexus_example.xml
       export RMW_FASTRTPS_USE_QOS_FROM_XML=1
       ros2 run demo_nodes_cpp add_two_ints_client

#. Open another terminal and run:

   .. code-block:: bash

      # Server
       export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/xml/vulcanexus_example.xml
       export RMW_FASTRTPS_USE_QOS_FROM_XML=1
       ros2 run demo_nodes_cpp add_two_ints_server
