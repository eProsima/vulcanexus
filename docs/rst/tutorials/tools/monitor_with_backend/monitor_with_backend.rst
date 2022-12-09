.. _tutorials_tools_monitor_with_backend:

ROS 2 Monitor example with statistics backend
=============================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Vulcanexus integrates :ref:`vulcanexus_monitor`, which is a useful tool for monitoring and studying a ROS 2 network as ROS 2 relies on the `DDS specification <https://www.omg.org/spec/DDS/1.4/About-DDS/>`_ to communicate the different nodes.

This tutorial provides an explanation of the ROS 2 Monitor code.

Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Ensure that the Vulcanexus installation includes Vulcanexus Tools (either ``vulcanexus-humble-desktop``, ``vulcanexus-humble-tools``, or ``vulcanexus-humble-base``).

Create the application workspace
--------------------------------

The application workspace will have the following structure at the end of the project.

.. code-block:: shell-session

    .
    └── monitor
        ├── include
        │   ├── monitor
        └── src
            └── monitor.cpp
        ├── CMakeLists.txt
        └── package.xml

Let's create the ROS 2 package by running the command:

.. code-block:: bash

    ros2 pkg create --build-type ament_cmake monitor --dependencies fastcdr fastrtps fastdds_statistics_backend

You will now have a new folder within your workspace ``src`` directory called ``monitor``.

Import linked libraries and its dependencies
--------------------------------------------

The DDS application requires the Fast DDS and Fast CDR libraries.
Depending on the installation procedure followed the process of making these libraries available for our DDS application
will be slightly different.

Installation from binaries and manual installation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If we have followed the installation from binaries or the manual installation, these libraries are already accessible
from the workspace.
On Linux, the header files can be found in directories `/usr/include/fastrtps/` and
`/usr/include/fastcdr/` for Fast DDS and Fast CDR respectively. The compiled libraries of both can be found in
the directory `/usr/lib/`.

Colcon installation
^^^^^^^^^^^^^^^^^^^

From a Colcon installation there are several ways to import the libraries.
If the libraries need to be available just for the current session, run the following command.

.. code-block:: bash

    source <path/to/Fast-DDS/workspace>/install/setup.bash

They can be made accessible from any session by adding the Fast DDS installation directory to your ``$PATH``
variable in the shell configuration files for the current user running the following command.

.. code-block:: bash

    echo 'source <path/to/Fast-DDS/workspace>/install/setup.bash' >> ~/.bashrc

This will set up the environment after each of this user's logins.

Write the Monitor
-----------------

This is the C++ source code for the application.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :linenos:

Examining the code
------------------

At the beginning of the file we have a Doxygen style comment block with the ``@file`` field that tells us the name of
the file.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 15-18

Below are the includes of the C++ headers that allow the use of the Fast DDS and Statistics Backend API.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 29-33

Next, we define the namespaces that contains the eProsima Fast DDS and Statistics Backend classes and functions that we are going to use in our application.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 35-36

The next line creates the :class:`Monitor` class that implements the monitor.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 38

The public constructor and destructor of the :class:`Monitor` class are defined below.
The constructor initializes the protected data members of the class to the values passed as arguments.
The class destructor stop the monitor.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 42-55

Continuing with the public member functions of the :class:`Monitor` class, the next snippet of code defines
the public initialization member function.
This function performs several actions:

1.  Initializes the monitor.
2.  Assigns the listener to the Statistics Backend.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 57-69

To run the monitor, the public member function ``run()`` is implemented. It called the public member functions ``get_fastdds_latency_mean()`` and ``get_publication_throughput_mean()`` explained below.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 71-89

Next comes the public member function ``stop()`` which stops the monitor setting the protected data member ``stop_`` to ``true``.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 91-94

Then the public member function ``get_fastdds_latency_mean()`` that get the Fast DDS latency mean of the last t_interval seconds between the talker and the listener.
To achieve this the function performs several actions:

1.  Get the DataWriters and DataReaders in a Topic.
2.  Get the current time.
3.  Get the mean of the FASTDDS_LATENCY of the last 5 seconds between the DataWriters and DataReaders publishing under and subscribed to the given topic. In this case ``rt/chatter``.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 97-153

And the public member function ``get_publication_throughput_mean()`` that get the publication throughput mean of the last t_interval seconds of the talker.
The function has a similar execution than the previous one but in this case get the mean of the PUBLICATION_THROUGHPUT instead of the FASTDDS_LATENCY.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 155-208

The last public member function is ``timestamp_to_string()`` which is in charge of serialize the timestamp of a given data value.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 210-221

Finally, the ROS 2 Monitor is initialized and run in main.

.. literalinclude:: ./code/monitor/src/monitor.cpp
    :language: C++
    :lines: 399-421

CMakeLists.txt
--------------

Include at the end of the CMakeList.txt file you created earlier the following code snippet. This adds all the source
files needed to build the executable, and links the executable and the library together.

.. literalinclude:: ./code/monitor/CMakeLists.txt
    :language: cmake
    :lines: 40-55

At this point the project is ready for building, compiling and running the application.
From the build directory in the workspace, run the following commands.

.. code-block:: bash

    colcon build
    . install/setup.bash
    ros2 run monitor monitor
