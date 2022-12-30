.. _tutorials_micro_discovery_server:

micro-ROS and Discovery Server
==============================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

micro-ROS can be use with Fast DDS `Discovery Server <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Advanced/Discovery-Server/Discovery-Server.html>`_ as discovery protocol.
Fast DDS Discovery Server can be useful for several reasons, some of the most important:

- An scenario where a considerable amount of participants and endpoints share the same domain.
- A network where multicast communication is not available.
- A network where multicast does not work properly, such as WiFi networks.
- DDS isolation needs to be ensured between different data spaces but DDS domains cannot be used.

.. note::

    Further information about `Discover Server setup <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html>`_.

.. note::

    Further information about `how to use Discover Server <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`_.


micro-ROS Agent configuration
-----------------------------

micro-ROS Agent uses Fast DDS as middleware, so it can be configured using Fast DDS XML configuration files.
Also, micro-ROS Agent can be configured using environment variables.

As any Fast DDS application, micro-ROS Agent can be configured in two ways:

- `Environment variable <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#ros-discovery-server>`_.
- Configuration XML


ROS 2 tools with Discover Server
--------------------------------

In order to be able to use ROS 2 tools to interface with micro-ROS entities created in a micro-ROS Agent with Discovery Server enabled an XML profile shall be loaded in the ROS environment.

The following example provides an example XML for configuring ROS 2 Daemon as a SuperClient:

.. code-block::

    <?xml version="1.0" encoding="UTF-8" ?>
    <dds>
        <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
            <participant profile_name="super_client_profile" is_default_profile="true">
                <rtps>
                    <builtin>
                        <discovery_config>
                            <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                            <discoveryServersList>
                                <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                            </discoveryServersList>
                        </discovery_config>
                    </builtin>
                </rtps>
            </participant
        </profiles
    </dds>


In order to use the previous XML file, the environment variable ``FASTRTPS_DEFAULT_PROFILES_FILE`` shall be set to the path of the XML file:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=[PATH_TO_XML_FILE]
