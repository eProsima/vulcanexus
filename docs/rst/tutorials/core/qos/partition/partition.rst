.. _tutorials_qos_partition_partition:

Modifying Partition QoS Policy
==============================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Fast DDS over Vulcanexus offers the possibility of fully configuring QoS policy through XML profile definition.
This tutorial provides step-by-step instructions to modify the Partition QoS Policy within the ROS 2 talker/listener demo (see `Partition QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#partitionqospolicy>`_).

This QoS Policy allows the introduction of a logical partition inside the physical partition introduced by a domain.
For a DataReader to see the changes made by a DataWriter, not only the Topic must match, but also they have to share at least one logical partition.

Prerequisites
-------------

The first prerequisite is to have Vulcanexus Iron installed (see `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_ or `Linux instalation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`_).

Please, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/iron/setup.bash

In a terminal sourced with the previous line, run the following command to install the ROS 2 demo-nodes-cpp tutorial (administrative privileges may be required):

.. code-block:: bash

    apt-get update && apt install -y ros-iron-demo-nodes-cpp

XML Profile definition
----------------------

In order to specify the desired custom configuration for the Partition QoS policy, an XML file is required (see `Fast DDS XML profiles <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_).
In any directory, run the following commands to create two files named `corresponding_partition.xml` and `another_partition.xml`:

.. code-block:: bash

    touch corresponding_partition.xml
    touch another_partition.xml

Open those files with your preferred editor, and write down the following XML code to the `corresponding_partition.xml` file.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <data_writer profile_name="/chatter">
            <qos>
                <partition>
                    <names>
                        <name>part1</name>
                        <name>part2</name>
                    </names>
                </partition>
            </qos>
        </data_writer>

        <data_reader profile_name="/chatter">
            <qos>
                <partition>
                    <names>
                        <name>part1</name>
                        <name>part4</name>
                    </names>
                </partition>
            </qos>
        </data_reader>
    </profiles>

Write down the following XML code to the `another_partition.xml` file.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <data_writer profile_name="/chatter">
            <qos>
                <partition>
                    <names>
                        <name>part3</name>
                    </names>
                </partition>
            </qos>
        </data_writer>
    </profiles>

For the next section of this tutorial, let us consider both created XML files are stored in the ``~/`` directory.

Execute ROS 2 demo nodes with modified QoS
------------------------------------------

Open one terminal and source Vulcanexus environment.
To set `corresponding_partition.xml` to define the profile configuration used on the creation of ROS 2 nodes, populating the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to point out to the file is needed.
Then, you can run `ros-demo-nodes-cpp` program to create a listener belonging to `part1` and `part4` partitions:

.. code-block:: bash

    source /opt/vulcanexus/iron/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/corresponding_partition.xml
    ros2 run demo_nodes_cpp listener

Open another terminal and source Vulcanexus environment.
To create `ros-demo-nodes-cpp` talker belonging to `part1` and `part2`, run the following commands:

.. code-block:: bash

    source /opt/vulcanexus/iron/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/corresponding_partition.xml
    ros2 run demo_nodes_cpp talker

.. note::

    Note that the profile used by the listener is the data_reader profile, and the one used by the talker is the data_writer one.

Now, both nodes should be communicating, as they belong to at least one same partition, which is `part1` in this case.
It can be seen that, the `Hellow World` messages that the talker sends, are being received by the listener.

Talker process can be killed using `Ctr+C`.
Then, in the same terminal, to create `ros-demo-nodes-cpp` talker belonging to `part3`, we will set `FASTRTPS_DEFAULT_PROFILES_FILE` to point out to `another_partition.xml`.
Run the following commands:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=~/another_partition.xml
    ros2 run demo_nodes_cpp talker

Now talker and listener are not communicating, as they don't belong to any same partition.
Talker and listener are isolated from one another, as listener reads messages just from `part1` or `part4` partitions, while talker is publishing its messages for `par3` only.
