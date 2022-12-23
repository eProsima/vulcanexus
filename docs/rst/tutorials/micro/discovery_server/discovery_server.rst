.. _tutorials_micro_discovery_server:

-> tutorial: https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Advanced/Discovery-Server/Discovery-Server.html
-> tutorial: https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html
-> variable entorno: https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#ros-discovery-server


Micro ROS with Discovery Server
===============================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

micro-ROS can be use with Fast DDS `Discovery Server <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Advanced/Discovery-Server/Discovery-Server.html>`_ as discovery protocol, this could be useful for several reasons, some of the most important:

- You have a massive amount of participants and endpoints.
- There is no multicast available on your network.
- There is multicast but you are usign wifi network, some routers does not work properly on this scenario.
- You need to configure several DDS domains but you dont want to flash different firmware on your devices.

.. note::

    Further information about `Discover-Server setup <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html>`_.

.. note::

    Further information about `how to use Discover-Server <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`_.


micro-ROS agent configuration
=============================

As any Fast DDS application can be configured in two ways:

- `Environment variable <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#ros-discovery-server>`_.
- Configuration XML


ros2 tools with Discover-Server
===============================

In order to be able to use ros2 tools with Discovery-Server you need to configure ros2 Daemon as a SuperClient using an xml configuration file.

.. code-block:: xml

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


Export env variable to indicat xml file path:

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=/super_client.xml
