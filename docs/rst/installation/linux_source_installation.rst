.. _linux_source_installation:

Linux installation from sources
===============================

This section explains how to build Vulcanexus in Ubuntu Jammy.
Since Vulcanexus is a ROS 2 all-in-one tool set, certain ROS 2 prerequisites need to be met before building.

ROS 2 prerequisites
-------------------

First of all, set up a `UTF-8` locale as required by ROS 2.
Locale settings can be checked and set up with the following commands:

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_LOCALE
   :end-before: ##!

ROS 2 also requires that the Ubuntu Universe repository is enabled.
This can be checked and enabled with the following commands:

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_UBUNTU_UNIVERSE
   :end-before: ##!


Now download ROS 2 GPG key into the keystore.

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_KEYSTORE
   :end-before: ##!


And then add ROS 2 repository to the repository manager sources list.

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_REPO_SOURCELIST
   :end-before: ##!


With the ROS 2 repository properly set up the next step is to install the required dependencies and tools for cloning
and testing the ROS 2 packages within the workspace.

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_ROS2_DEPS
   :end-before: ##!


Get ROS 2 code
--------------

Create a workspace for Vulcanexus and clone the ROS 2 repositories

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_CLONE_ROS2_REPOS
   :end-before: ##!

Now download the required dependencies for these packages.

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_DOWN_ROS2_DEPS
   :end-before: ##!


Get Vulcanexus code
-------------------

Add the Vulcanexus repositories and metadata files to the Vulcanexus workspace:

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_CLONE_VULCA
   :end-before: ##!


Install Vulcanexus dependencies
-------------------------------

Some additional dependencies which are required for the Vulcanexus distribution must be installed.
Install the Vulcanexus required development tools with the following command:

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_VULCA_DEPS
   :end-before: ##!


Build the code in the workspace
-------------------------------

If any other Vulcanexus or ROS 2 distribution has been installed from binaries, please ensure that the build is done in a fresh environment (previous installation is not sourced).
This can be checked running the following command:

.. code-block:: bash

    printenv | grep 'VULCANEXUS\|ROS'

The output should be empty.
Please, be aware that in case the environment sourcing has been added to ``.bashrc``, it must be removed in order to get a fresh environment.

Build Fast DDS-Gen (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.
This tool must be built separately following the instructions below.
Please, refer to `Fast DDS-Gen documentation <https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/introduction/introduction.html>`_ for more information about this tool.

.. code-block:: bash

    cd src/eProsima/Fast-DDS-Gen
    ./gradlew assemble

.. note::

   Java version 17 is required to build Fast DDS Gen.
   It can be installed by executing the following commands

   .. code-block:: bash

       sudo apt update
       sudo apt install openjdk-17-jdk

The generated Java application can be found in ``src/eProsima/Fast-DDS-Gen/share/fastddsgen``.
However, the ``scripts`` folder provides some user friendly scripts that are recommended to be used.
This scripts can be made accessible to the session adding the ``scripts`` folder path to the ``PATH`` environment variable.

.. code-block:: bash

    export PATH=~/vulcanexus_jazzy/src/eProsima/Fast-DDS-Gen/scripts:$PATH

Build workspace
^^^^^^^^^^^^^^^

In order to build the workspace, the command line tool `colcon <https://colcon.readthedocs.io/en/released/>`_ is used.
This tool is based on `CMake <https://cmake.org/>`_ and it is aimed at building sets of software packages, handling ordering and setting up the environment to use them.

.. literalinclude:: ../../resources/scripts/linux_source_installation.bash
   :language: bash
   :start-after: ##LINUX_SOURCE_VULCA_COMPILE
   :end-before: ##!


.. important::

    In case that only a set of packages are going to be built, please ensure to include always ``vulcanexus_base`` package in the set.
    E.g.:

    .. code-block:: bash

        colcon build --packages-up-to demo_nodes_cpp vulcanexus_base

    This auxiliary package is required to set several environment variables required by the distribution such as ``VULCANEXUS_DISTRO`` and ``VULCANEXUS_HOME``.

Environment setup
-----------------

In order to use the Vulcanexus installation, the environment must be set up sourcing the following file:

.. code-block:: bash

    source ~/vulcanexus_jazzy/install/setup.bash

Try some examples
-----------------

In one terminal, source the setup file and then run a C++ ``talker``:

.. code-block:: bash

    . ~/vulcanexus_jazzy/install/local_setup.bash
    ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a Python ``listener``:

.. code-block:: bash

    . ~/vulcanexus_jazzy/install/local_setup.bash
    ros2 run demo_nodes_py listener

At this point, the ``talker`` should prompt ``Publishing: Hello World: <id>`` messages while the ``listener`` echoes ``I heard: [Hello World: <id>]``.
This verifies both the C++ and Python APIs are working properly.
Hooray!


