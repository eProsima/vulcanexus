.. _vulcanexus_fastddsspy:

Fast DDS Spy
============

*Fast DDS Spy* is CLI interactive tool that allows the introspection of a ROS 2 network in human readable format.
It is possible to query the network about the Participants connected, their endpoints (Publishers and Subscriptions) and the topics they communicate in.
It is also possible to see the user data sent through network topics in a schematic format in run time.

Overview
--------

*Fast DDS Spy* is a tool that captures DDS packages in the network and maintain a local database that is accessible from a interactive CLI.
*Fast DDS Spy* responds to user commands introduced by text and prints to ``stdout`` the information requested.
This tool has several commands to interact with it, allowing the user to get information about the status of the network.
It supports listing of existing topics, Participants, Publishers, and Subscriptions, and printing data in real time in a human readable format.

.. figure:: /rst/figures/intro/tools/fastddsspy/fastddsspy.png
    :align: center

*Fast DDS Spy* is easily configurable and installed with a default setup, so that DDS topics, data types and entities are automatically discovered without the need to specify the types of data.
This is because this tool leverages the DynamicTypes functionality of `eProsima Fast DDS <https://fast-dds.docs.eprosima.com>`_, the C++ implementation of the `DDS (Data Distribution Service) Specification <https://www.omg.org/spec/DDS/About-DDS/>`_ defined by the `Object Management Group (OMG) <https://www.omg.org/>`_.

Next Steps
----------

Visit `eProsima Fast DDS Spy Documentation <https://fast-dds-spy.readthedocs.io/en/latest>`_ for more information on how to configure and use *Fast DDS Spy*.
Also feel free to review :ref:`Vulcanexus Tools Tutorials <tutorials_tools_tools_tutorials>` to discover the possibilities that this tool has to offer in different scenarios.
