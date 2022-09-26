.. _tutorials_qos_ownership_ownership:

Modifying Ownership and Ownership Strenght QoS Policy
=====================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Fast DDS over Vulcanexus offers the possibility of fully configuring QoS policy through XML profile definition.
This tutorial provides step-by-step instructions to modify the Ownership QoS within the ROS 2 talker/listener demo
(see `Ownership QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#ownershipqospolicy>`).

This QoS Policy specifies whether it is allowed for multiple DataWriters to update the same instance of data,
and if so, how these modifications should be arbitrated.

Prerequisites
-------------

Please, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

In a terminal sourced with the previous line, run the following command to install the ROS 2 demo-nodes-cpp tutorial
(administrative privileges may be required):

.. code-block:: bash

    apt-get update && install -y ros-humble-demo-nodes-cpp

XML Profile definition
----------------------

In order to specify the desired custom configuration for the Ownership QoS policy, it is needed to construct an XML file
(see `Fast DDS XML profiles <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_).
In that file, the desired configuration is set.

In any directory, run the following commands to create two files named `profiles1.xml` and `profiles2.xml`:

.. code-block:: bash

    touch profiles1.xml
    touch profiles2.xml

Open those files with your preferred editor, and write down the following XML code to the `profiles1.xml` file.
For the `profiles2.xml` file, write down the same code, but changing the value 10 for any other larger number, for example, 58.

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

For the next section of this tutorial, lets cosider both created XML files are stored in `~/` path.

Execute ROS 2 demo nodes with modified QoS
------------------------------------------


