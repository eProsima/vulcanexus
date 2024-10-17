.. _tutorials_tools_monitor_with_backend:

Monitoring ROS 2 with Fast DDS Statistics Backend
=================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Vulcanexus integrates :ref:`fastdds_statistics_backend`, which is a useful tool for monitoring and studying a ROS 2 network since ROS 2 relies on the `DDS specification <https://www.omg.org/spec/DDS/1.4/About-DDS/>`_ to communicate the different nodes.
This tool will be used to create a monitoring application tailored to the user's needs.
Therefore, this tutorial provides a step by step tutorial on how to create your first application as a ROS 2 package to monitor your ROS 2 deployment.

.. figure:: /rst/figures/tutorials/tools/monitor_backend/monitor_talker_listener_summary.png

This tutorial will use the ``demo_nodes_cpp`` package, available in the Vulcanexus Desktop distribution. First, a ROS 2 ``talker`` is launched and then a ``listener`` node is started in the same machine.
At this point, the monitor application created using the *Fast DDS Statistics Backend* library will be deployed to record and present the overall communication performance.

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Ensure that the Vulcanexus installation includes Vulcanexus Tools (either ``vulcanexus-jazzy-desktop``, ``vulcanexus-jazzy-tools``, or ``vulcanexus-jazzy-base``).

Creating the monitor package and application
--------------------------------------------

In this section the monitoring application is created from scratch, covering the creation of the working environment and the development of the source code.

Creating the application workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The application workspace will have the following structure at the end of the project.

.. code-block:: shell-session

    ros2_ws
    └── src
        └── monitor_tutorial
            ├── src
                └── monitor.cpp
            ├── CMakeLists.txt
            └── package.xml

Let's create the ROS 2 workspace and package by running the following commands:

.. code-block:: bash

    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ros2 pkg create --build-type ament_cmake monitor_tutorial --dependencies fastcdr fastrtps fastdds_statistics_backend

You will now have a new folder within your workspace `src` directory called `monitor_tutorial`.

Writing the monitor application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From the `ros_ws/src/monitor_tutorial/src` directory in the workspace, run the following command to download the `monitor.cpp` file.

.. code-block:: bash

    wget -O monitor.cpp \
        https://raw.githubusercontent.com/eProsima/vulcanexus/main/code/monitor_tutorial/src/monitor.cpp

This is the C++ source code for the application. This source code can also be found `here <https://github.com/eProsima/vulcanexus/blob/main/code/monitor_tutorial/src/monitor.cpp>`_.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++

Examining the code
^^^^^^^^^^^^^^^^^^

At the beginning of the file, the Doxygen style comment block with the ``@file`` field states the name of the file.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 15-18

Below are the includes of the C++ headers that allow the use of *Fast DDS* and *Fast DDS Statistics Backend* API.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 28-31

Next, we define the namespace that contains the *Fast DDS Statistics Backend* classes and functions that we are going to use in our application.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 33

The next line creates the :class:`Monitor` class that implements the monitor.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 35

The public constructor and destructor of the :class:`Monitor` class are defined below.
The constructor initializes the protected data members of the class to the values passed as arguments.
The class destructor stops the monitor.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 39-54

Continuing with the public member functions of the :class:`Monitor` class, the next snippet of code defines the public initialization member function.
This function performs several actions:

#.  Initialize the monitor.
#.  Check the monitor has a valid id in the Statistics Backend database.
#.  Assign the physical listener to the Statistics Backend.
    This listener will capture any update in the discovery of DDS entities.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 56-68

To run the monitor, the public member function ``run()`` is implemented.
This function search for the ``rt/chatter`` topic in the Statistics Backend database by calling the ``get_topic_id()`` function.
If this is found, the we can proceed to compute the actual statistics data.
In order to do so, it calls ``get_fastdds_latency_mean()`` and ``get_publication_throughput_mean()`` public member functions explained below.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 70-86

As introduced before, the ``get_topic_id()`` public member function get the id of the topic searching by topic name and data type name.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 88-108

The public member function ``get_fastdds_latency_mean()`` gets the Fast DDS latency mean of the last ``t_interval`` seconds between the ``talker`` and the ``listener``.
To achieve this the function performs several actions:

#.  Get the Publishers and Subscriptions in a Topic.
#.  Get the current time.
#.  Get the mean of the ``FASTDDS_LATENCY`` of the last time interval between the Publishers and Subscriptions publishing under and subscribed to the given topic, ``rt/chatter`` in this case.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 109-155

Finally, the public member function ``get_publication_throughput_mean()`` gets the publication throughput mean of the last ``t_interval`` seconds of the ``talker``.
The function has a similar execution procedure than the previous one but in this case it query the mean of the ``PUBLICATION_THROUGHPUT`` instead of the ``FASTDDS_LATENCY``.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 156-197


Then, the protected :class:`Listener` class is defined by inheriting from the `PhysicalListener <https://fast-dds-statistics-backend.readthedocs.io/en/latest/rst/api-reference/listener/physicallistener.html>`_ class.
This class overrides the default PhysicalListener callbacks, which allows the execution of routines in case of an event.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 214-226

Within the PhysicalListener class, we can override several callback to adapt how the monitor application reacts to some events.
These overridden callbacks are:

*   ``on_host_discovery()`` allows the definition of a series of actions when a new host is detected.

    .. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
        :language: C++
        :lines: 226-241

*   ``on_user_discovery()`` detects when a new user is discovered.

    .. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
        :language: C++
        :lines: 242-257

*   ``on_process_discovery()`` involves when a new process is discovered.

    .. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
        :language: C++
        :lines: 258-273

*   ``on_topic_discovery()`` is called when a new Topic is discovered.

    .. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
        :language: C++
        :lines: 274-295

*   ``on_participant_discovery()`` is called when a new participant is discovered.

    .. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
        :language: C++
        :lines: 296-313

*   ``on_datareader_discovery()`` and ``on_datawriter_discovery()`` involves when a new DataReader or DataWriter respectively are discovered.

    .. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
        :language: C++
        :lines: 314-349

Finally, the monitor application is initialized and run in ``main`` function.

.. literalinclude:: /../code/monitor_tutorial/src/monitor.cpp
    :language: C++
    :lines: 360-375

CMakeLists.txt
^^^^^^^^^^^^^^

Include at the end of the `CMakeList.txt` file you created earlier the following code snippet.
This adds all the source files needed to build the executable, and links the executable and the library together.

.. literalinclude:: /../code/monitor_tutorial/CMakeLists.txt
    :language: cmake
    :lines: 40-49

This file can also be downloaded with this command in `ros_ws/src/monitor_tutorial` directory:

.. code-block:: bash

    wget -O CMakeList.txt \
        https://raw.githubusercontent.com/eProsima/vulcanexus/main/code/monitor_tutorial/CMakeLists.txt


Running the application
-----------------------

At this point the project is ready for building, compiling and running the application.
From the base workspace directory (`ros_ws`), run the following commands.

.. code-block:: bash

    colcon build
    source install/setup.bash
    ros2 run monitor_tutorial monitor_tutorial

Then open two more terminals and load the Vulcanexus environment.
Then, in one of them run a ``talker`` and in the other one a ``listener`` of the ``demo_nodes_cpp`` ROS 2 package, available in the Vulcanexus Desktop distribution.

*   Terminal 1:

    .. code-block:: bash

        source /opt/vulcanexus/jazzy/setup.bash
        export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;PHYSICAL_DATA_TOPIC"
        ros2 run demo_nodes_cpp talker

*   Terminal 2:

    .. code-block:: bash

        source /opt/vulcanexus/jazzy/setup.bash
        export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;PHYSICAL_DATA_TOPIC"
        ros2 run demo_nodes_cpp listener

.. note::

    In order to monitor other *statistics topics*, add them to environment variable :code:`FASTDDS_STATISTICS`.
    Check the *statistics topics* available in the `Fast DDS Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/dds_layer/topic_names.html#statistics-topic-names>`_.

You should be able to see something similar to the following image.

.. figure:: /rst/figures/tutorials/tools/monitor_backend/monitor_talker_listener_terminal.png

Next steps
----------

Now you can develop more functionalities in your application, such as collecting more performance data or monitoring other topics.
You can check also :ref:`this tutorial <tutorials_tools_prometheus>` explaining how to connect an application developed with the *Fast DDS Statistics Backend* to a visualization tool like *Grafana*.

For more information about *Fast DDS Statistics Backend* features please refer to the `project's documentation <https://fast-dds-statistics-backend.readthedocs.io/en/latest/>`_.
