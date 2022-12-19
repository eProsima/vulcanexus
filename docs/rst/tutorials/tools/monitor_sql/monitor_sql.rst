.. _tutorials_tools_monitor_sql:

Fast DDS Statistics Backend Monitoring to SQL
=============================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Vulcanexus integrates :ref:`fastdds_statistics_backend`, which is a useful tool for monitoring and studying a ROS 2 network since ROS 2 relies on the `DDS specification <https://www.omg.org/spec/DDS/1.4/About-DDS/>`_ to communicate the different nodes.
The interaction with this tool is through a C++ API.
This could be done to create powerful monitoring tools specialized by the user.
In this tutorial we show how to create an application of a :ref:`fastdds_statistics_backend` connected with a SQL in-disk database.

.. figure:: /rst/figures/tutorials/tools/sql_monitor_backend/summary.png

This tutorial assumes that the user has already finished :ref:`previous tutorial <tutorials_tools_monitor_with_backend>`
and understand how the *Fast DDS Statistics Backend* works and how to interact with it.
The final application created will store latency and throughput data of :code:`chatter` topic in an in-disk relation SQL database.

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Ensure that the Vulcanexus installation includes Vulcanexus Tools (either ``vulcanexus-humble-desktop``, ``vulcanexus-humble-tools``, or ``vulcanexus-humble-base``).

.. note::

    This tutorial uses SQLite3 as SQL library to connect with an in-disk database.
    This SQLite3 is already installed in ROS2 environment.


Resultant Database
------------------

This tutorial executable :code:`monitor_sql_tutorial` produces a database stored with name :code:`vulcanexus_monitor.db` in the workspace where executed.
This database contains one table called :code:`data` with 3 columns:

* **timestamp [key]** *milliseconds since epoch* Time since epoch in milliseconds
* **latency_median** *nanoseconds* Median of Latency in the interval :code:`timestamp - 5000 : timestamp`
* **throughput_median** *MB/second* Median of Throughput in the interval :code:`timestamp - 5000 : timestamp`

Each 5000 ms the program calls the *Statistics Backend* API and store the results for latency median and
throughput mean for all Nodes using :code:`chatter` topic.
The :code:`timestamp` column is the **key** of the table as it cannot be repeated.
It is stored as a number and not a string or timestamp to simplify the tutorial.

There is a useful browser application to see the data inside a database file: `<http://inloop.github.io/sqlite-viewer/>`__.
The resultant database will look like this:

.. figure:: /rst/figures/tutorials/tools/sql_monitor_backend/database.png

.. warning::

    It is possible that some data is not available because it is not being published from the entities.
    In these cases *Statistics Backend* returns :code:`NaN`
    This is parsed as a :code:`0` when inserted in the database to avoid format issues.

Creating the monitor package and application
--------------------------------------------

This section explains the source code required to implement this tutorial.
However, some code is reused from :ref:`previous tutorial <tutorials_tools_monitor_with_backend>` and will not be repeated here.


Creating the application workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The application workspace will have the following structure at the end of the project.

.. code-block:: shell-session

    ros2_ws
    └── src
        └── monitor_sql_tutorial
            ├── src
            |   └── sql_monitor.cpp
            ├── CMakeLists.txt
            └── package.xml

Let's create the ROS 2 workspace and package by running the following commands:

.. code-block:: bash

    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ros2 pkg create --build-type ament_cmake monitor_sql_tutorial --dependencies fastcdr fastrtps fastdds_statistics_backend sqlite3_vendor

You will now have a new folder within your workspace `src` directory called `monitor_sql_tutorial`.

Writing the monitor application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From the `ros_ws/src/monitor_sql_tutorial/src` directory in the workspace, run the following command to download the `sql_monitor.cpp` file.

.. code-block:: bash

    wget -O sql_monitor.cpp \
        https://raw.githubusercontent.com/eProsima/vulcanexus/main/code/monitor_sql_tutorial/src/sql_monitor.cpp

This is the C++ source code for the application. This source code can also be found `here <https://github.com/eProsima/vulcanexus/blob/main/code/monitor_sql_tutorial/src/sql_monitor.cpp>`_.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++

Examining the code
^^^^^^^^^^^^^^^^^^

Before declaring the main class, there are some definitions that could be changed before compiling.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 38-43

The main class called :code:`Monitor` creates the database and initializes the monitor in construction.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 49-69

And it closes them in destruction.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 71-75

The routine of the instance is an infinite loop where, if the topic has already been discovered, it stores
data in the database. This is similar to :ref:`previous tutorial <tutorials_tools_monitor_with_backend>`.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 77-94

The database is opened if exists, or created otherwise.
It is initialized with the table :code:`data`.
In case open or creating the table fails, the execution will finish.
In exit, it closes the database.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 176-197

The routine to store new data in the database firstly call the *Statistics Bakend*.
Then it loads the data received in a query previously written.
Finally it executes the query to insert the data.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 204-225

The queries used to interact with the database are defined inside the class as static const variables.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 229-238

Finally, the monitor application is initialized and run in ``main`` function.

.. literalinclude:: /../code/monitor_sql_tutorial/src/sql_monitor.cpp
    :language: C++
    :lines: 251-260


CMakeLists.txt
^^^^^^^^^^^^^^

Include at the end of the `CMakeList.txt` file you created earlier the following code snippet.
This adds all the source files needed to build the executable, and links the executable and the library together.

.. literalinclude:: /../code/monitor_sql_tutorial/CMakeLists.txt
    :language: cmake
    :lines: 40-49

This file can also be downloaded with this command in `ros_ws/src/monitor_sql_tutorial` directory:

.. code-block:: bash

    wget -O CMakeList.txt \
        https://raw.githubusercontent.com/eProsima/vulcanexus/main/code/monitor_sql_tutorial/CMakeLists.txt


Running the application
-----------------------

At this point the project is ready for building, compiling and running the application.
From the base workspace directory (`ros_ws`), run the following commands.

.. code-block:: bash

    colcon build
    source install/setup.bash
    ros2 run monitor_sql_tutorial monitor_sql_tutorial

Then open two more terminals and load the Vulcanexus environment.
Then, in one of them run a ``talker`` and in the other one a ``listener`` of the ``demo_nodes_cpp`` ROS 2 package, available in the Vulcanexus Desktop distribution.

*   Terminal 1:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;PHYSICAL_DATA_TOPIC"
        ros2 run demo_nodes_cpp talker

*   Terminal 2:

    .. code-block:: bash

        source /opt/vulcanexus/humble/setup.bash
        export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;PHYSICAL_DATA_TOPIC"
        ros2 run demo_nodes_cpp listener

.. note::

    In order to monitor other *statistics topics*, add them to environment variable :code:`FASTDDS_STATISTICS`.
    Check the *statistics topics* available in the `Fast DDS Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/dds_layer/topic_names.html#statistics-topic-names>`_.

Next steps
----------

Now you can develop more functionalities in your application, such as collecting more performance data or monitoring other topics.
You can check also :ref:`this tutorial <tutorials_tools_prometheus>` explaining how to connect an application developed with the *Fast DDS Statistics Backend* to a visualization tool like *Grafana*.

For more information about *Fast DDS Statistics Backend* features please refer to the `project's documentation <https://fast-dds-statistics-backend.readthedocs.io/en/latest/>`_.
