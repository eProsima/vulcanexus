.. _tutorials_tools_fastddsspy:

Getting Started with Fast DDS Spy
==================================

Background
----------

Vulcanexus integrates `eProsima Fast DDS Spy <https://fast-dds-spy.readthedocs.io/en/latest/index.html>`_,
an interactive command-line tool that allows inspecting DDS entities such as DomainParticipants, DataWriters,
DataReaders, topics, and live user data in a human-readable format.
Since ROS 2 relies on the `DDS specification <https://www.omg.org/spec/DDS/1.4/About-DDS/>`_ for communication between nodes,
*Fast DDS Spy* is a useful tool for observing an existing DDS system and understanding what entities and data are visible.

This tutorial explains how to use *Fast DDS Spy* to inspect a running DDS network step by step.
By the end of this tutorial, you will be able to:

* Start a simple DDS scenario using *ShapesDemo*.
* Launch *Fast DDS Spy*.
* Discover DDS DomainParticipants.
* List DataWriters and DataReaders.
* List active DDS topics.
* Inspect topic information.
* Display the IDL/type information of a topic.
* Echo live topic data from the DDS network.

Prerequisites
-------------

Ensure that the Vulcanexus installation includes Vulcanexus Tools (either ``vulcanexus-kilted-desktop``, ``vulcanexus-kilted-tools``, or ``vulcanexus-kilted-base``).
Also, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/kilted/setup.bash

In addition, you will need a DDS application publishing data. In this tutorial, *ShapesDemo* is used.
Both *ShapesDemo* and *Fast DDS Spy* must be running in the same DDS domain.

.. note::

    *Fast DDS Spy* can also be installed as a standalone tool. See the
    `Fast DDS Spy documentation <https://fast-dds-spy.readthedocs.io/en/latest/rst/installation/docker.html>`_
    for alternative installation options.

Scenario Overview
-----------------

In this tutorial, *ShapesDemo* publishes data on a DDS topic, and *Fast DDS Spy* is used to inspect what is happening
in the DDS network.

.. code-block:: text

    ShapesDemo publisher
         |
         v
    DDS network
         |
         v
    Fast DDS Spy

The goal is to observe an existing DDS system and understand what entities and data are visible,
not to `create a publisher or subscriber from scratch <https://fast-dds.docs.eprosima.com/en/latest/fastdds/getting_started/simple_app/simple_app.html>`_.

Step 1: Start ShapesDemo
------------------------

Open *ShapesDemo* in a terminal:

.. code-block:: bash

    ShapesDemo

You should see the *ShapesDemo* application open.

.. figure:: /rst/figures/tutorials/tools/fastddsspy/ShapesDemo.png
    :align: center

Configure the participant
^^^^^^^^^^^^^^^^^^^^^^^^^

In the toolbar at the top, open the ``Options`` menu and select ``Participant Configuration``.
This window lets you choose the transport mode, domain ID, and other advanced options.
In this tutorial, set the Domain ID to ``10``.

.. figure:: /rst/figures/tutorials/tools/fastddsspy/ParticipantConfiguration.png
    :align: center

.. note::

    In this tutorial, the statistics option is left unchecked. Once you have completed the tutorial,
    you can try running the demo again with statistics enabled and observe the differences.

You should see a small banner under the app indicating ``Participant ready in domainId 10``.

Run the publisher and the subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Click the ``Publish`` button and, in the pop-up menu, select the ``Square`` shape, the ``RED`` color, and a size of ``30``,
then click ``OK``.

.. figure:: /rst/figures/tutorials/tools/fastddsspy/Publish.png
    :align: center

Once *ShapesDemo* starts publishing the squares, create the subscriber. Click the ``Subscribe`` button and
select the ``Square`` shape.

.. figure:: /rst/figures/tutorials/tools/fastddsspy/Subscribe.png
    :align: center

You should now see *ShapesDemo* running and publishing square samples.

.. figure:: /rst/figures/tutorials/tools/fastddsspy/ShapesDemoRunning.png
    :align: center

There is now an active DDS DataWriter publishing data on the ``Square`` topic as well as a DataReader receiving it.
*Fast DDS Spy* will later discover these entities and the topic they communicate on.

.. raw:: html

    <video width=100% height=auto autoplay loop controls>
        <source src="../../../../_static/ShapesDemo.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>

Step 2: Launch Fast DDS Spy
----------------------------

Open a new terminal and launch *Fast DDS Spy*:

.. code-block:: bash

    source /opt/vulcanexus/kilted/setup.bash
    fastddsspy --domain 10

.. note::

    The default Domain ID is ``0``, so the ``--domain`` option is used to switch to domain ``10``,
    where *ShapesDemo* is publishing.

*Fast DDS Spy* should start and display an interactive prompt.

.. code-block:: text

     ____|             |        __ \   __ \    ___|        ___|
     |     _` |   __|  __|      |   |  |   | \___ \      \___ \   __ \   |   |
     __|  (   | \__ \  |        |   |  |   |       |           |  |   |  |   |
    _|   \__,_| ____/ \__|     ____/  ____/  _____/      _____/   .__/  \__, |
                                                                  _|     ____/


    Insert a command for Fast DDS Spy:
    >>

*Fast DDS Spy* is running and ready to receive commands. It will automatically discover DDS entities available in the configured domain.

Step 3: Display the Help Menu
------------------------------

Run:

.. code-block:: bash

    help

*Fast DDS Spy* will display the list of available commands.

.. code-block:: text

    >> help
    Fast DDS Spy is an interactive CLI that allows you to introspect DDS networks.
    Each command shows data related with the network in Yaml format.
    Commands available and the information they show:
        help                                      : this help.
        version                                   : tool version.
        quit                                      : exit interactive CLI and close program.
        participants                              : DomainParticipants discovered in the network.
        participants verbose                      : verbose information about DomainParticipants discovered in the network.
        participants <Guid>                       : verbose information related with a specific DomainParticipant.
        writers                                   : DataWriters discovered in the network.
        writers verbose                           : verbose information about DataWriters discovered in the network.
        writers <Guid>                            : verbose information related with a specific DataWriter.
        reader                                    : DataReaders discovered in the network.
        reader verbose                            : verbose information about DataReaders discovered in the network.
        reader <Guid>                             : verbose information related with a specific DataReader.
        topics                                    : Topics discovered in the network in compact format.
        topics v                                  : Topics discovered in the network.
        topics vv                                 : verbose information about Topics discovered in the network.
        topics <name>                             : Topics discovered in the network filtered by name (wildcard allowed (*)).
        topics <name> idl                         : Display the IDL type definition for topics matching <name> (wildcards allowed).
        filters                                   : Display the active filters.
        filters clear                             : Clear all the filter lists.
        filters remove                            : Remove all the filter lists.
        filter clear <category>                   : Clear <category> filter list.
        filter remove <category>                  : Remove <category> filter list.
        filter set <category> <filter_str>        : Set <category> filter list with <filter_str> as first value.
        filter add <category> <filter_str>        : Add <filter_str> in <category> filter list.
        filter remove <category> <filter_str>     : Remove <filter_str> in <category> filter list.
        echo <name>                               : data of a specific Topic (Data Type must be discovered).
        echo <wildcard_name>                      : data of Topics matching the wildcard name (and whose Data Type is discovered).
        echo <name> verbose                       : data with additional source info of a specific Topic.
        echo <wildcard_name> verbose              : data with additional source info of Topics matching the topic name (wildcard allowed (*)).
        echo all                                  : verbose data of all topics (only those whose Data Type is discovered).

    Notes and comments:
        To exit from data printing, press enter.
        Each command is accessible by using its first letter (h/v/q/p/w/r/t/s/f).

    For more information about these commands and formats, please refer to the documentation:
    https://fast-dds-spy.readthedocs.io/en/latest/

The help menu is useful when you do not remember the exact command syntax. It also confirms that the interactive CLI is working correctly.

Step 4: Discover DomainParticipants
-------------------------------------

Run:

.. code-block:: bash

    participants

You should see at least one discovered DomainParticipant related to *ShapesDemo*. The output includes the participant name and its GUID.

.. code-block:: text

    >> participants
    - name: Fast DDS ShapesDemo Participant
      guid: 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.1.c1

*Fast DDS Spy* has discovered at least one DDS participant in the network, confirming that discovery is working.

.. note::

    If no participants appear, check that *ShapesDemo* is still running, that both applications use the same DDS domain,
    and that your network configuration allows DDS discovery. Wait a few seconds and run the command again.

Verbose output
^^^^^^^^^^^^^^

Use the verbose option to check more detailed information, including endpoints and the topics they are associated with.

.. code-block:: bash

    participants verbose

.. code-block:: text

    >> participants verbose
    - name: Fast DDS ShapesDemo Participant
      guid: 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.1.c1
      datawriters:
        - Square [ShapeType] (1)
      datareaders:
        - Square [ShapeType] (1)

Step 5: Discover DataWriters
------------------------------

Run:

.. code-block:: bash

    writers

You should see one DataWriter associated with the ``Square`` topic.

.. code-block:: text

    >> writers
    - guid: 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.1.2
      participant: Fast DDS ShapesDemo Participant
      topic: Square [ShapeType]

A DataWriter is a DDS entity that publishes data. Seeing a writer associated with ``Square`` confirms that *Fast DDS Spy* can discover the publisher side of the DDS communication.

Verbose output
^^^^^^^^^^^^^^

Running the command with the verbose option shows more information about the topic and its QoS.

.. code-block:: bash

    writers verbose

.. code-block:: text

    >> writers verbose
    - guid: 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.1.2
      participant: Fast DDS ShapesDemo Participant
      topic:
        name: Square
        type: ShapeType
        partitions: ""
      qos:
        durability: volatile
        reliability: reliable

Step 6: Discover DataReaders
------------------------------

Run:

.. code-block:: bash

    readers

You should see one DataReader associated with the ``Square`` topic.

.. code-block:: text

    >> readers
    - guid: 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.2.7
      participant: Fast DDS ShapesDemo Participant
      topic: Square [ShapeType]

A DataReader is a DDS entity that subscribes to data. Seeing a reader associated with the ``Square`` topic confirms that *Fast DDS Spy* can discover the subscriber side of the communication.

Verbose output
^^^^^^^^^^^^^^

Running this command with the verbose option shows more detailed information about the reader's topic and QoS.

.. code-block:: bash

    readers verbose

.. code-block:: text

    >> readers verbose
    - guid: 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.2.7
      participant: Fast DDS ShapesDemo Participant
      topic:
        name: Square
        type: ShapeType
        partitions: ""
      qos:
        durability: volatile
        reliability: best-effort

.. raw:: html

    <video width=100% height=auto autoplay loop controls>
        <source src="../../../../_static/EntitiesProperties.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>

Step 7: List DDS Topics
------------------------

Run:

.. code-block:: bash

    topics

You should see the active topics discovered in the DDS network.

.. code-block:: text

    >> topics
    - topic: Square (ShapeType) (1|1) [13.334426 Hz]

*Fast DDS Spy* has discovered the topic names used by the DDS network. Listing topics is usually the first command to run when you want to understand what data is available.

Step 8: Inspect a Topic in Detail
-----------------------------------

Run:

.. code-block:: bash

    topics Square v

For more detailed output, run:

.. code-block:: bash

    topics Square vv

The verbose output includes the topic name, type, associated DataWriters and DataReaders, publication rate, and type discovery information.

.. code-block:: text

    >> topics Square v
    - name: Square
      type: ShapeType
      datawriters: 1
      datareaders: 1
      rate: 13.3329 Hz

.. code-block:: text

    >> topics Square vv
    - name: Square
      type: ShapeType
      datawriters:
        - 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.1.2 [""]
      datareaders:
        - 01.0f.91.a5.1f.00.1c.08.00.00.01.00|0.0.2.7 [""]
      rate: 13.3355 Hz
      dynamic_type_discovered: true

Step 9: Display the Topic IDL
-------------------------------

Run:

.. code-block:: bash

    topics Square idl

*Fast DDS Spy* will display the IDL/type definition associated with the ``Square`` topic.

.. code-block:: text

    >> topics Square idl

    @extensibility(APPENDABLE)
    struct ShapeType
    {
        @key string color;
        long x;
        long y;
        long shapesize;
    };

The IDL output shows the structure of the data being published. This helps you understand what fields will appear when live data is echoed.

Step 10: Echo Live Topic Data
------------------------------

Run:

.. code-block:: bash

    echo Square

*Fast DDS Spy* will print live samples published on the ``Square`` topic.

.. code-block:: text

    >> echo Square
    ---
    {
        "color": "RED",
        "shapesize": 30,
        "x": 141,
        "y": 164
    }
    ---

    ---
    {
        "color": "RED",
        "shapesize": 30,
        "x": 135,
        "y": 159
    }
    ---

The exact values will change while *ShapesDemo* is running. Press ``Enter`` to stop printing live data and return to the *Fast DDS Spy* prompt.

For more detailed output including topic name and type, writer GUID, partitions, and a timestamp, run:

.. code-block:: bash

    echo Square verbose

*Fast DDS Spy* is now reading and displaying user data from the DDS network. This confirms that the topic data type was discovered and that live samples are being received.

.. raw:: html

    <video width=100% height=auto autoplay loop controls>
        <source src="../../../../_static/Topics.mp4">
        Your browser does not support the video tag.
    </video>
    <br></br>

Step 11: Exit Fast DDS Spy
----------------------------

Run:

.. code-block:: bash

    quit

*Fast DDS Spy* will close and return you to the normal terminal prompt.

Optional: Use a YAML Configuration File
-----------------------------------------

*Fast DDS Spy* can be configured using a YAML file. This is useful when you want to change the DDS domain, configure transports, or filter topics.

A minimal configuration example:

.. code-block:: yaml

    dds:
      domain: 0

You can also use topic filters to focus only on specific topics:

.. code-block:: yaml

    topics:
      allowlist:
        - name: Square

.. note::

    See the `Fast DDS Spy configuration documentation <https://fast-dds-spy.readthedocs.io/en/latest/rst/user_manual/configuration.html>`_
    for the complete YAML schema supported by your version.

Next Steps
----------

After completing this tutorial, you can continue with:

* Running *Fast DDS Spy* with a YAML configuration file.
* Filtering topics with allowlists or blocklists.
* Inspecting a real ROS 2 application instead of *ShapesDemo*.
* Using *Fast DDS Spy* as part of a debugging or validation workflow.

Visit `eProsima Fast DDS Spy Documentation <https://fast-dds-spy.readthedocs.io/en/latest>`_ for more information
on how to configure and use *Fast DDS Spy*.
