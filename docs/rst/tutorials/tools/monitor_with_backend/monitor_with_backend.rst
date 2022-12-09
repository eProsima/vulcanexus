.. _tutorials_tools_monitor_with_backend:

ROS2 Monitor example with statistics backend
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

Write the Monitor
-----------------

This is the C++ source code for the application.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :linenos:

Examining the code
------------------

At the beginning of the file we have a Doxygen style comment block with the ``@file`` field that tells us the name of
the file.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 15-18

Below are the includes of the C++ headers that allow the use of the Fast DDS and Statistics Backend API.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 29-33

Next, we define the namespaces that contains the eProsima Fast DDS and Statistics Backend classes and functions that we are going to use in our application.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 35-36

The next line creates the :class:`Monitor` class that implements the monitor.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 38

The public constructor and destructor of the :class:`Monitor` class are defined below.
The constructor initializes the protected data members of the class to the values passed as arguments.
The class destructor stop the monitor.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 42-55

Continuing with the public member functions of the :class:`Monitor` class, the next snippet of code defines
the public initialization member function.
This function performs several actions:

1.  Initializes the monitor.
2.  Assigns the listener to the Statistics Backend.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 57-69

To run the monitor, the public member function ``run()`` is implemented. It called the public member functions ``get_fastdds_latency_mean()`` and ``get_publication_throughput_mean()`` explained below.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 71-89

Next comes the public member function ``stop()`` which stops the monitor setting the protected data member ``stop_`` to ``true``.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 91-94

Finally, the Monitor is initialized and run in main.

.. literalinclude:: ./code/Monitor/src/Monitor.cpp
    :language: C++
    :lines: 414-437
