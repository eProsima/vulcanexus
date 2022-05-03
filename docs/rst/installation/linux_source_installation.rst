.. _linux_source_installation:

Linux installation from sources
===============================

This section explains how to build Vulcanexus in Ubuntu Focal.
Since Vulcanexus is a ROS 2 all-in-one tool set, certain ROS 2 prerequisites need to be met before building.

ROS 2 prerequisites
-------------------

First of all, setup a `UTF-8` locale as required by ROS 2.
Locale settings can be checked and set up with the following commands:

.. code-block:: bash

    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    # Any UTF-8 locale will work. Using en_US as an example
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

ROS 2 also requires that the Ubuntu Universe repository is enabled.
This can be checked and enabled with the following commands:

.. code-block:: bash

    apt-cache policy | grep universe

    # This should print something similar to:
    #
    #  500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
    # release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64
    #
    # Otherwise run

    sudo apt install software-properties-common
    sudo add-apt-repository universe

Now download ROS 2 GPG key into the keystore.

.. code-block:: bash

    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

And then add ROS 2 repository to the repository manager sources list.

.. code-block:: bash

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

With the ROS 2 repository properly set up the next step is to install the required dependencies and tools for cloning and testing the ROS 2 packages within the workspace.

.. code-block:: bash

    sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget
    # install some pip packages needed for testing
    python3 -m pip install -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    setuptools


Get ROS 2 code
--------------

Create a workspace for Vulcanexus and clone the ROS 2 repositories

.. code-block:: bash

    mkdir -p ~/vulcanexus_galactic/src
    cd ~/vulcanexus_galactic
    wget https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos
    vcs import src < ros2.repos

Now download the required dependencies for these packages.

.. code-block:: bash

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

Get Vulcanexus code
-------------------

Add the Vulcanexus repositories and metadata files to the Vulcanexus workspace:

.. code-block::

    cd ~
    cd vulcanexus_galactic
    wget https://raw.githubusercontent.com/eProsima/vulcanexus/galactic/vulcanexus.repos
    wget https://raw.githubusercontent.com/eProsima/vulcanexus/galactic/colcon.meta
    vcs import --force src < vulcanexus.repos

Install Vulcanexus dependencies
-------------------------------

Some additional dependencies which are required for the Vulcanexus distribution must be installed.
Start by adding the Qt 5.15 repository required for the installation of several Fast DDS Monitor dependencies:

.. code-block:: bash

    sudo apt install -y software-properties-common
    sudo add-apt-repository -y ppa:beineri/opt-qt-5.15.2-focal

Next, install the Vulcanexus required development tools:

.. code-block:: bash

    sudo apt update && sudo apt install -y \
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

    cd src/eProsima/fastddsgen
    ./gradlew assemble

The generated Java application can be found in ``share/fastddsgen``.
However, the ``scripts`` folder provides some user friendly scripts that are recommended to be used.
This scripts can be made accessible to the session adding the ``scripts`` folder path to the ``PATH`` environment variable.

.. code-block:: bash

    export PATH=~/vulcanexus_galactic/src/eProsima/fastddsgen/scripts:$PATH

Build workspace
^^^^^^^^^^^^^^^

In order to build the workspace, the command line tool `colcon <https://colcon.readthedocs.io/en/released/>`_ is used.
This tool is based on `CMake <https://cmake.org/>`_ and it is aimed at building sets of software packages, handling ordering and setting up the environment to use them.

.. code-block:: bash

    cd ~/vulcanexus_galactic
    colcon build

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

    source ~/vulcanexus_galactic/install/setup.bash

Next steps
----------

Please, refer to the :ref:`vulcanexus_tutorials` section to keep learning about Vulcanexus capabilities and features.
