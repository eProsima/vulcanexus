.. _linux_installation:

Linux binary installation
=========================

Debian packages for Vulcanexus Galactic Gamble are currently available for Ubuntu Focal.
Please, start following the `Installation Guide for ROS 2 Galactic Geochelone <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>`_ setting the locale to support ``UTF-8`` (if necessary) and the ROS 2 apt repository.
Before proceeding installing the ROS 2 packages, follow the steps below.

Setup sources
-------------

First, enable the following required repository running the command:

.. code-block:: bash

    sudo add-apt-repository ppa:beineri/opt-qt-5.15.2-focal

Next, authorize Vulcanexus GPG key with apt:

.. code-block:: bash

    sudo curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/master/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg

Finally, add the eProsima Vulcanexus repository to your sources list:

.. code-block:: bash

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] TODO(URL) $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null

Install eProsima Vulcanexus packages
------------------------------------

Remember to update your apt repository caches after setting up the repositories:

.. code-block:: bash

    sudo apt update

Desktop install (Recommended): includes all the simulation tools, demos, and tutorials.

.. code-block:: bash

    sudo apt install vulcanexus-galactic-desktop

Base Install: basic installation without simulation tools, demos, and tutorials.

.. code-block:: bash

    sudo apt install vulcanexus-galactic-base

There are other packages available. Please refer to :ref:`main_feature` for more information.
