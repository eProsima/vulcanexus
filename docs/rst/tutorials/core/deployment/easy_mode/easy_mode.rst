.. include:: ../../../../exports/alias.include

.. _easy_mode_tutorial:

Easy Mode Tutorial
==================

This tutorial aims to demonstrate the new :ref:`easy_mode` feature in *Vulcanexus* by running a simple talker-listener example demo.

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Prerequisites
-------------

* It is recommended to have a basic understanding of ``Vulcanexus Easy Mode`` introduced in the :ref:`easy_mode` section.
* An up-to-date (latest) Vulcanexus installation using one of the following installation methods:

  * :ref:`linux_binary_installation`
  * :ref:`linux_source_installation`
  * :ref:`docker_installation`

Preparation
-----------

Lets start by setting up the Vulcanexus environment.
For this, there are two possible options:

#.  Running the Vulcanexus Docker image (recommended).

    Run the Vulcanexus Docker image with:

    .. code-block:: bash

        docker run -it --rm ubuntu-vulcanexus:{DISTRO}-desktop

    Then, within the container, source the Vulcanexus installation with:

    .. code-block:: bash

            source /opt/vulcanexus/{DISTRO}/setup.bash

    Repeat these steps in a second terminal.

#.  Running the tutorial between two hosts (an available connection to the same network will be needed).
    For this second option, it is necessary to have the ``vucanexus-jazzy-base`` package installed.

    In both hosts, source the following file to setup the Vulcanexus environment:

    .. code-block:: bash

            source /opt/vulcanexus/{DISTRO}/setup.bash

Running the demo
----------------

The demo will be exemplified using Docker.
It consists in two docker containers (hosts) running a ROS 2 talker - listener example with the new ``Vulcanexus Easy Mode`` enabled.
Both hosts are in the same network and domain id as shown in the following diagram:

.. image:: ../../../../figures/enhancements/easy_mode/easy_mode_tutorial.png
    :align: center
    :width: 55%

Discovery server in Host A will serve as the master server.
Hence, the IP address of Host A will be used to enable the ``Easy Mode`` in both containers.
Run the following commands in each container:

.. code-block:: bash

    # Container 1
    EASY_MODE=172.17.0.2 ros2 run demo_nodes_cpp talker

    # Container 2
    EASY_MODE=172.17.0.2 ros2 run demo_nodes_cpp listener

.. note::

    If the tutorial is run using two hosts, the IP address of the ``EASY_MODE`` environment variable needs to be set to the IP address of the first host.
    It is also possible to run the demo in a single host by opening two terminals and running the commands in each terminal.
    In this case, the IP address should be set to the hosts's IP and only one discovery server will be used.

After a moment, the listener should start receiving samples from the talker, meaning that both Discovery Servers are now connected to each other.
As the talker and listener nodes share the same topic ``chatter``, nodes are discovered and data exchange can happen.
Further benefits of using the new ``Easy Mode`` are detailed in the :ref:`easy_mode_benefits` section.
