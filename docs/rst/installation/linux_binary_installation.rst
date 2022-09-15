.. _linux_binary_installation:

Linux binary installation
=========================

Debian packages for Vulcanexus Humble Hierro are currently available for Ubuntu Focal.
Since Vulcanexus is a ROS 2 all-in-one tool set, certain ROS 2 prerequisites need to be met before installing.

ROS 2 prerequisites
-------------------

First of all, set up a `UTF-8` locale as required by ROS 2.
Locale settings can be checked and set up with the following commands:

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_LOCALE
   :end-before: ##!

ROS 2 also requires that the Ubuntu Universe repository is enabled.
This can be checked and enabled with the following commands:

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_UBUNTU_UNIVERSE
   :end-before: ##!

Now download ROS 2 GPG key into the keystore.

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_KEYSTORE
   :end-before: ##!


And finally add ROS 2 repository to the repository manager sources list.

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_REPO_SOURCELIST
   :end-before: ##!


Setup Vulcanexus sources
------------------------

Once all ROS 2 prerequisites have been met, it is time to set up Vulcanexus.

Add Vulcanexus GPG key so apt can retrieve the packages:

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_VULCA_SOURCES
   :end-before: ##!

Next, add the eProsima Vulcanexus repository to the repository manager sources list:

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_VULCA_ADD_REPO
   :end-before: ##!


Install eProsima Vulcanexus packages
------------------------------------

Remember to update the apt repository caches after setting up the repositories:

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_VULCA_UPDATE
   :end-before: ##!

Desktop install (Recommended): includes all the simulation tools, demos, and tutorials.

.. literalinclude:: ../../resources/scripts/linux_binary_installation.bash
   :language: bash
   :start-after: ##LINUX_BINARY_VULCA_INSTALL
   :end-before: ##!

Base Install: basic installation without simulation tools, demos, and tutorials.

.. code-block:: bash

    sudo apt install -y vulcanexus-humble-base

For other Vulcanexus packages, please refer to :ref:`vulcanexus_binary_distribution` for more
information.

Environment setup
-----------------

In order to use the Vulcanexus installation, the environment must be set up sourcing the following file:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

Uninstall eProsima Vulcanexus packages
--------------------------------------

To uninstall Vulcanexus, it is enough to run the following command :

.. code-block:: bash

    sudo apt autoremove vulcanexus-humble-desktop
