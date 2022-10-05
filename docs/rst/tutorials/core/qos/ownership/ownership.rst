.. _tutorials_qos_ownership_ownership:

Modifying Ownership and Ownership Strength QoS Policy
=====================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Fast DDS over Vulcanexus offers the possibility of fully configuring QoS policy through XML profile definition.
This tutorial provides step-by-step instructions to modify the Ownership QoS within the ROS 2 talker/listener demo (see `Ownership QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#ownershipqospolicy>`_).

This QoS Policy specifies whether it is allowed for multiple DataWriters to update the same instance of data, and if so, how these modifications should be arbitrated.

Prerequisites
-------------

First prerequisite is to have Vulcanexus Humble installed (see `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_ or `Linux instalation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`_)

Please, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

In a terminal sourced with the previous line, run the following command to install the ROS 2 demo-nodes-cpp tutorial (administrative privileges may be required):

.. code-block:: bash

    apt-get update && apt install -y ros-humble-demo-nodes-cpp

XML Profile definition
----------------------

In order to specify the desired custom configuration for the Ownership QoS policy, an XML file is required (see `Fast DDS XML profiles <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_).

In any directory, run the following commands to create two files named `small_strength.xml` and `large_strength.xml`:

.. code-block:: bash

    touch small_strength.xml
    touch large_strength.xml

Open those files with your preferred editor and write down the following XML code to the `small_strength.xml` file.
For the `large_strength.xml` file, write down the same code, but changing the value ``10`` for any other larger number, for example, ``58``.

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <data_writer profile_name="/chatter">
            <qos>
                <ownership>
                    <kind>EXCLUSIVE</kind>
                </ownership>
                <ownershipStrength>
                    <value>10</value>
                </ownershipStrength>
            </qos>
        </data_writer>

        <data_reader profile_name="/chatter">
            <qos>
                <ownership>
                    <kind>EXCLUSIVE</kind>
                </ownership>
            </qos>
        </data_reader>
    </profiles>

For the next section of this tutorial, let us consider both created XML files are stored in `~/` path.

Execute ROS 2 demo nodes with modified QoS
------------------------------------------

Open one terminal and source Vulcanexus environment.
To set `small_strength.xml` to define the profile configuration used on the creation of ROS 2 nodes, populating the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to point out to the file is needed.
Then, you can run `ros-demo-nodes-cpp` program to create a listener with `EXCLUSIVE_OWNERSHIP_POLICY` QoS:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/small_strength.xml
    ros2 run demo_nodes_cpp listener

Open another terminal and source Vulcanexus environment.
To create `ros-demo-nodes-cpp` talker, run the following commands:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/small_strength.xml
    ros2 run demo_nodes_cpp talker

Now both nodes should be communicating.
It can be seen that, the `Hellow World` messages that the talker sends, are being received by the listener.
The number of those messages coincides.

In a third terminal, source Vulcanexus environment.
To create another `ros-demo-nodes-cpp` talker, but now with greater ownership strength (see `Ownership Strength QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#ownershipstrengthqospolicy>`_), this time the `FASTRTPS_DEFAULT_PROFILES_FILE` will point out to `large_strength.xml`:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/large_strength.xml
    ros2 run demo_nodes_cpp talker

Now it must be seen that the first talker keeps sending messages, but the messages being read by listener are those of the newly created talker (The number of the message being sent by last talker should be the same as the number of the arriving message in listener).
This is happening due to the fact that this last talker has a higher Ownership Strength value, than the first talker.

If now the second talker process is killed, the messages being received by the listener are the ones from the first talker.
