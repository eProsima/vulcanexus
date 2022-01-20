.. _linux_source_installation:

Linux installation from sources
===============================

This section explains how to build Vulcanexus in Ubuntu Focal.
Please, start following the `building instructions for ROS 2 Galactic Geochelone <https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html>`_ following the steps in the `System setup <https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html#system-setup>`_, `Get ROS 2 code <https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html#get-ros-2-code>`_, and the `Install dependencies using rosdep <https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html#install-dependencies-using-rosdep>`_ sections.

Install Vulcanexus dependencies
-------------------------------

Besides the dependencies installed using the ROS 2 ``rosdep`` tool, some additional dependencies are required for the Vulcanexus distribution.
First, add the Qt 5.15 repository:

.. code-block:: bash

    sudo apt install -y software-properties-common
    sudo add-apt-repository ppa:beineri/opt-qt-5.15.2-focal

Next, install the Vulcanexus required development tools:

.. code-block:: bash

    sudo apt update && sudo apt install -y \
      ed \
      clang \
      libp11-dev \
      libengine-pkcs11-openssl \
      libyaml-cpp-dev \
      openjdk-8-jdk \
      qt5-default \
      qt5153d \
      qt515charts-no-lgpl \
      qt515graphicaleffects \
      qt515quickcontrols \
      qt515quickcontrols2 \
      qt515quicktimeline-no-lgpl \
      qt515svg \
      qt515tools \
      qt515translations \
      swig

Get Vulcanexus code
-------------------

Following ROS 2 building instructions, the ROS 2 code is already downloaded into ``ros2_galactic`` folder because ``rosdep`` tool requires access to the packages to resolve the required dependencies.
Vulcanexus provides several tools and packages to the ROS 2 distribution that has to be included in the workspace:

.. code-block::

    cd ~
    mv ~/ros2_galactic ~/vulcanexus_galactic
    cd vulcanexus_galactic
    wget https://raw.githubusercontent.com/eProsima/vulcanexus/galactic/vulcanexus.repos
    wget https://raw.githubusercontent.com/eProsima/vulcanexus/galactic/colcon.meta
    vcs import --force src < vulcanexus.repos

Build the code in the workspace
-------------------------------

If any other Vulcanexus or ROS 2 distribution have been installed from binaries, please ensure that the build is done in a fresh environment (previous installation is not sourced).
This can be checked running the following command:

.. code-block:: bash

    printenv | grep 'VULCANEXUS\|ROS'

The output should be empty.
Please, be aware that in case you have added the environment sourcing to your ``.bashrc``, you must remove it in order to get a fresh environment.

Build Fast DDS-Gen
^^^^^^^^^^^^^^^^^^

*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.
This tool must be built separately following the instructions below.
Please, refer to `Fast DDS-Gen documentation <https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/introduction/introduction.html>`_ for more information about this tool.

.. code-block:: bash

    cd src/eProsima/fastddsgen
    ./gradlew assemble

The generated Java application can be found in ``share/fastddsgen``.
However, the ``scripts`` folder provides some user friendly scripts that are recommended to be used.
This scripts can be made accesible to the session adding the ``scripts`` folder path to the ``PATH`` environment variable.

.. code-block:: bash

    export PATH=~/vulcanexus_galactic/src/eProsima/fastddsgen/scripts:$PATH

Build workspace
^^^^^^^^^^^^^^^

In order to build the workspace, the command line tool `colcon <https://colcon.readthedocs.io/en/released/>`_ is used.
This tool is based on `CMake <https://cmake.org/>`_ and it is aimed at building sets of software packages, handling ordering and setting up the environment to use them.

.. code-block:: bash

    cd ~/vulcanexus_galactic
    colcon build

Environment setup
-----------------

In order to use the Vulcanexus installation, the environment must be set up sourcing the following file:

.. code-block:: bash

    source ~/vulcanexus_galactic/install/setup.bash

Next steps
----------

Please, refer to the :ref:`vulcanexus_tutorials` section to keep learning about Vulcanexus capabilities and features.
