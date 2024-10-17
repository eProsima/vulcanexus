.. |br| raw:: html

  <br/>

.. _tutorials_qos_persistency_persistency:

Persistent Data using Durability QoS
====================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

The DDS `Durability QoS Policy
<https://fast-dds.docs.eprosima.com/en/{FASTDDS_BRANCH}/fastdds/dds_layer/core/policy/standardQosPolicies.html#durabilityqospolicy>`_
specifies how much importance ROS 2 nodes give to the exchanged data. The
possible options are briefly explained :ref:`here <concepts_about_qos>`:

* Durability

  * *Volatile*: Old data values are ignored. Any new node will not receive any previous data.
  * *Transient local*: Old data values are important. Any new node will receive data generated before its creation.
  * *Transient*: Data is so important that it is backed up or persisted into a database file. This will guarantee that if a
    node crashes it can be reenacted and keep operating without data losses.

Only *Volatile* and *Transient Local* durability options are ROS 2 builtins. In order to set up *Transient* nodes, xml
configuration files are required as explained in :ref:`the XML profiles tutorial <tutorials_xml_profiles_intro>`. This tutorial
provides guidelines on how to set up persistent nodes.

Prerequisites
-------------

Vulcanexus jazzy should be installed (:ref:`follow the steps here <linux_binary_installation>`).
For testing sake, using a docker container is often a more convenient approach (:ref:`docker setup <docker_installation>`).
In order to test this feature, a *Vulcanexus* lightweight docker image (as *core* or *micro*) is enough.
The images are available for download on *Vulcanexus* `website <https://vulcanexus.org/download>`_.

.. code-block:: bash

    docker load -i ./ubuntu-vulcanexus-{DISTRO}-micro.tar

    # Terminal 1
    docker run -ti --name persistence_testing ubuntu-vulcanexus:ironicro

    # Terminal 2
    docker exec -ti persistence_testing /vulcanexus_entrypoint.sh /bin/bash

This tutorial requires two terminals: one for the data writer (1) and the other for the reader (2).
Note that for user convenience, the the terminal that launches the container (1) has the overlay loaded in the
*entrypoint* but any other terminal that connects to it (2) must explicitly load the overlay calling the
``vulcanexus_entripoint.sh`` script.

The concepts introduced here are applied to any ROS 2 nodes and the tutorial will not use a specific package but rely on
the ROS 2 :ref:`CLI capabilities <ROS2Topics>`.

To create a publisher node from the ROS 2 command line that sends *TIMES* samples:

.. code-block:: bash

    ros2 topic pub [-t TIMES] [--qos-durability {system_default,transient_local,volatile,unknown}] topic_name message_type [values]

To create a subscriber node from ROS 2 command line:

.. code-block:: bash

    ros2 topic echo [--qos-durability {system_default,transient_local,volatile,unknown}] topic_name [message_type]

.. note::

    The *ros2 topic* command line interface provides convenient parameters for defining node QoS' as ``--qos-durability``. |br|
    Specifying those on ROS 2 packages requires modifying the sources to appropriately set
    :ref:`ROS 2 client library <ROS-2-Client-Libraries>` QoS structures as
    `rclcpp::QoS <https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1QoS.html>`_ or
    `rclpy.qos <https://docs.ros2.org/latest/api/rclpy/api/qos.html#rclpy.qos.QoSProfile>`_.

XML Profile definition
----------------------

In order to specify the desired custom configuration for the Durability QoS policy, an XML file is required (see `Fast
DDS XML profiles <https://fast-dds.docs.eprosima.com/en/{FASTDDS_BRANCH}/fastdds/xml_configuration/xml_configuration.html>`_).

Usually a single configuration file is enough but this tutorial requires two independent persistent databases (for
writer and reader) in order to show the persistency advantages.

In the working directory of choice (henceforth */home/tutorial/*) create two files:

* A publisher node configuration file: ``writer_config.xml``

.. literalinclude:: /resources/tutorials/core/qos/persistency/writer_config.xml
    :language: xml

* A subscriber node configuration file: ``reader_config.xml``

.. literalinclude:: /resources/tutorials/core/qos/persistency/reader_config.xml
    :language: xml

For each node is specified:

* A database filename as the ``dds.persistence.sqlite3.filename`` property in the default participant profile.

* An endpoint (``data_writer`` or ``data_reader``) profile which:

  + is associated to the node using the *topic name* as ``profile_name`` attribute. In this tutorial the
    *topic name* will be **persistency_test**.

  + The endpoint requires a `GUID <https://en.wikipedia.org/wiki/Universally_unique_identifier>`_ specified as
    the ``dds.persistence.guid`` property.

  + The endpoint Durability QoS must be set to *transient*.

Environment variables set up
----------------------------

ROS 2 nodes will locate their associated xml configuration files using the ``FASTRTPS_DEFAULT_PROFILES_FILE``
environment variable. For example:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/tutorial/writer_config.xml

Testing Publisher Persistency
-----------------------------

The following steps will show how using persistent endpoints prevents subscribers from receiving duplicated data
when a publisher crashes.

#. Launch persistent publisher and subscriber.

    .. code-block:: bash

        # Terminal 1
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/tutorial/writer_config.xml
        ros2 topic pub --times 5 --qos-durability system_default /persistency_test std_msgs/String "{data: 'Hello'}"

        # Terminal 2
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/tutorial/reader_config.xml
        ros2 topic echo --qos-durability system_default /persistency_test std_msgs/String

   Note that we must specify *system_default* as durability in order to enforce the use of the xml file provided value.

#. Relaunch the publisher again and check the subscriber is able to receive the samples.

    .. code-block:: bash

        # Terminal 1
        ros2 topic pub --times 5 --qos-durability system_default /persistency_test std_msgs/String "{data: 'Hello'}"

#. Delete the publisher database. Now the publisher state is reset. Relaunch the publisher and check the first 10 samples are
   discarded by the subscriber because they were already received.

    .. code-block:: bash

        # Terminal 1
        rm writer_database.db
        ros2 topic pub --times 11 --qos-durability system_default /persistency_test std_msgs/String "{data: 'Hello'}"

Testing Subscriber Persistency
------------------------------

The following steps will show how using persistent endpoints prevents subscribers from receiving duplicated data
when the subscriber crashes and it's relaunched.

#. Launch persistent publisher and subscriber.

    .. code-block:: bash

        # Terminal 1
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/tutorial/writer_config.xml
        ros2 topic pub --times 5 --qos-durability system_default /persistency_test std_msgs/String "{data: 'Hello'}"

        # Terminal 2
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/tutorial/reader_config.xml
        ros2 topic echo --qos-durability system_default /persistency_test std_msgs/String

#. Delete the publisher database. Relaunch the publisher and check the first 5 samples are
   discarded by the subscriber because they were already received.

    .. code-block:: bash

        # Terminal 1
        rm writer_database.db
        ros2 topic pub --times 10 --qos-durability system_default /persistency_test std_msgs/String "{data: 'Hello'}"

#. Delete the publisher database. Restart the subscriber node. Relaunch the publisher and check the first 10 samples are
   discarded by the subscriber because they were already received. Note the new subscriber deduced it from its
   persistence database.

    .. code-block:: bash

        # Terminal 1
        rm writer_database.db
        ros2 topic pub --times 11 --qos-durability system_default /persistency_test std_msgs/String "{data: 'Hello'}"

        # Terminal 2
        <CTRL-C>
        ros2 topic echo --qos-durability system_default /persistency_test std_msgs/String
