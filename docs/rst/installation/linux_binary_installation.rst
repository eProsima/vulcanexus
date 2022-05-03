.. _linux_binary_installation:

Linux binary installation
=========================

Debian packages for Vulcanexus Galactic Gamble are currently available for Ubuntu Focal.
Since Vulcanexus is a ROS 2 all-in-one tool set, certain ROS 2 prerequisites need to be met before installing.

ROS 2 prerequisites
-------------------

First of all, set up a `UTF-8` locale as required by ROS 2.
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

And finally add ROS 2 repository to the repository manager sources list.

.. code-block:: bash

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

Setup Vulcanexus sources
------------------------

Once all ROS 2 prerequisites have been met, it is time to start setting up Vulcanexus.

First, add the Qt 5.15 repository, required for the installation of several Fast DDS Monitor dependencies, running the commands:

.. code-block:: bash

    sudo apt install software-properties-common
    sudo add-apt-repository ppa:beineri/opt-qt-5.15.2-focal

Next, add Vulcanexus GPG key so apt can retrieve the packages:

.. code-block:: bash

    sudo curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg

Finally, add the eProsima Vulcanexus repository to the repository manager sources list:

.. code-block:: bash

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.com/debian $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null

Install eProsima Vulcanexus packages
------------------------------------

Remember to update the apt repository caches after setting up the repositories:

.. code-block:: bash

    sudo apt update

Desktop install (Recommended): includes all the simulation tools, demos, and tutorials.

.. code-block:: bash

    sudo apt install vulcanexus-galactic-desktop

Base Install: basic installation without simulation tools, demos, and tutorials.

.. code-block:: bash

    sudo apt install vulcanexus-galactic-base

For other Vulcanexus packages, please refer to the :ref:`Introduction <vulcanexus_introduction>` section for more
information.

Environment setup
-----------------

In order to use the Vulcanexus installation, the environment must be set up sourcing the following file:

.. code-block:: bash

    source /opt/vulcanexus/galactic/setup.bash

Uninstall eProsima Vulcanexus packages
--------------------------------------

To uninstall Vulcanexus, it is enough to run the following command :

.. code-block:: bash

    sudo apt autoremove vulcanexus-galactic-desktop

Next steps
----------

Please, refer to the :ref:`vulcanexus_tutorials` section to keep learning about Vulcanexus capabilities and features.
