.. _tutorials_qos_partition_partition:

Modifying Partition QoS Policy and Endpoint Partitions Property Policy
======================================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Fast DDS over Vulcanexus offers the possibility of fully configuring QoS policy through XML profile definition.
This tutorial provides step-by-step instructions to modify the Partition QoS Policy and Endpoint Partitions Property Policy
within the ROS 2 talker/listener demo.
Those conform two ways of uing partitions to configure Fast DDS communication.
(see `Partition QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#partitionqospolicy>`_
and `Endpoint Partitions Property <https://fast-dds.docs.eprosima.com/en/latest/fastdds/property_policies/non_consolidated_qos.html?highlight=partitions#endpoint-partitions>`_).

These two options allow the introduction of a logical partition inside the physical partition introduced by a domain.
For a DataReader to see the changes made by a DataWriter, not only the Topic must match,
but also they have to share at least one logical partition.

Prerequisites
-------------

Please, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

In a terminal sourced with the previous line, run the following command to install the ROS 2 demo-nodes-cpp tutorial
(administrative privileges may be required):

.. code-block:: bash

    apt-get update && apt-get install -y ros-humble-demo-nodes-cpp

XML Profile definition for Partition QoS Policy
-----------------------------------------------

In order to specify the desired custom configuration for the Partition QoS policy, it is needed to construct an XML file
(see `Fast DDS XML profiles <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_).
In that file, the desired configuration is set.

In any directory, run the following commands to create two files named `profiles1.xml` and `profiles2.xml`:

.. code-block:: bash

    touch profiles1.xml
    touch profiles2.xml

Open those files with your preferred editor, and write down the following XML code to the `profiles1.xml` file.

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

Write down the following XML code to the `profiles2.xml` file.

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

For the next section of this tutorial, lets consider both created XML files are stored in `~/` path.

Execute ROS 2 demo nodes with modified Partition QoS
----------------------------------------------------

Open one terminal and source Vulcanexus environment.
To set `profiles1.xml` to define the profile configuration used on the creation of ROS 2 nodes,
it is needed to populate the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to point out to the file.
Thus, in the terminal, run the following command:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=~/profiles1.xml

Then, you can run `ros-demo-nodes-cpp` program to create a listener belonging to `part1` and `part4` partitions:

.. code-block:: bash

    ros2 run demo_nodes_cpp listener

Open another terminal and source Vulcanexus environment.
To create `ros-demo-nodes-cpp` talker belonging to `part1` and `part2`, run the following commands:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=~/profiles1.xml
    ros2 run demo_nodes_cpp talker

Now both terminals should be communicating, as they belong to at least one same partition, which is `part1` in this case.
Can be seen that the `Hellow World` messages that talker sends are being received by listener.

Talker process can be killed using `Ctr+C`.
Then, to create `ros-demo-nodes-cpp` talker belonging to `part3`, run the following commands:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=~/profiles2.xml
    ros2 run demo_nodes_cpp talker

Now talker and listener are not communicating, as they don't belong to any same partition.


