.. include:: ../../../../exports/alias.include

.. _easy_mode_cli_tutorial:

Easy Mode CLI Tutorial
======================

This tutorial demonstrates the CLI usage of new :ref:`easy_mode` feature in *Vulcanexus*,
serving as a follow up of the :ref:`easy_mode_tutorial`. We will run two talkers and two listeners
distributed across different hosts and domains and we will use the :ref:`easy_mode` CLI to connect and disconnect
them as needed.

.. note::

  * The :ref:`easy_mode` feature works automatically and does not require the use of any of the following CLI commands in order to work.
    However, the following CLI commands  can be useful in certain complex scenarios.


.. contents::
    :depth: 2
    :local:
    :backlinks: none

Prerequisites
-------------

As mentioned earlier, this tutorial is a follow up of the :ref:`easy_mode_tutorial`. Therefore,
it is essential to have completed the prerequisites section of that tutorial beforehand.

Preparation
-----------

We will be working with up to 3 different hosts distributed in Docker containers.
First, create an isolated Docker network for the Vulcanexus containers:

.. code-block:: bash

        docker network create --subnet=172.18.0.0/16 vulcanexus_net

Run three containers using the Vulcanexus Docker image and source the Vulcanexus installation. For convenience,
we will store the IP addresses of the other hosts, as well as the container's own IP address, in environment variables.

.. code-block:: bash

    # Terminal 1 -> Host A
    docker run --net vulcanexus_net --ip 172.18.0.2 -it --rm ubuntu-vulcanexus:{DISTRO}-desktop
    source /opt/vulcanexus/{DISTRO}/setup.bash
    export OWN_IP=172.18.0.2 && export B_IP=172.18.0.3 && export C_IP=172.18.0.4

    #Terminal 2 -> HOST A (just another terminal from the same container as Terminal 1)
    docker exec -it $(docker ps -q) bash
    source /opt/vulcanexus/{DISTRO}/setup.bash
    export OWN_IP=172.18.0.2 && export B_IP=172.18.0.3 && export C_IP=172.18.0.4

    # Terminal 3 -> Host B
    docker run --net vulcanexus_net --ip 172.18.0.3 -it --rm ubuntu-vulcanexus:{DISTRO}-desktop
    source /opt/vulcanexus/{DISTRO}/setup.bash
    export A_IP=172.18.0.2 && export OWN_IP=172.18.0.3 && export C_IP=172.18.0.4

    # Terminal 4 -> Host C
    docker run --net vulcanexus_net --ip 172.18.0.4 -it --rm ubuntu-vulcanexus:{DISTRO}-desktop
    source /opt/vulcanexus/{DISTRO}/setup.bash
    export A_IP=172.18.0.2 && export B_IP=172.18.0.3 && export OWN_IP=172.18.0.4

.. note::

    It is also possible to run the tutorial between three hosts sharing the same network.

Running the demo
----------------

In this tutorial we will explore the CLI commands introduced in the :ref:`easy_mode_cli` section.

Before demonstrating each command, we will first showcase the standard :ref:`easy_mode` usage without CLI commands.
This will help establish the context for the subsequent CLI demonstrations.

* Host B will run a listener node in domain 1, along with its associated master Discovery Server. Since this is the master server,
  the environment variable ``ROS2_EASY_MODE`` will be set to its own IP address.
* Host C will have the same setup as Host B but running in domain 2 instead.
* Host A will run two talker nodes, one in domain 1 and another in domain 2, each spawning a Discovery Server that connects to Host B
  and Host C in their respective domains. Therefore, the environment variable ``ROS2_EASY_MODE`` will be set to the IP address of Host B
  for the domain 1 talker and to the IP address of Host C for the domain 2 talker.
* With this configuration alone, the talkers and listeners will be fully connected and exchanging data.

.. code-block:: bash

    # Terminal 3 -> Host B
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$B_IP ros2 run demo_nodes_cpp listener

    # Terminal 4 -> Host C
    ROS_DOMAIN_ID=2 ROS2_EASY_MODE=$C_IP ros2 run demo_nodes_cpp listener

    # Terminal 1 -> Host A
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$B_IP ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello world in domain 1 from HOST_A'}" --once --spin-time 2

    # Terminal 2 -> Host A
    ROS_DOMAIN_ID=2 ROS2_EASY_MODE=$C_IP ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello world in domain 1 from HOST_A'}" --once --spin-time 2

.. image:: ../../../../figures/tutorials/core/easy_mode/Diagrams_cli_0.png
    :align: center

.. image:: ../../../../figures/tutorials/core/easy_mode/cli_tutorial_0_easy.gif
    :align: center



START command
^^^^^^^^^^^^^

The ``start`` command is used to launch the Discovery Server daemon with the specified remote connections. This functionality is also implicitly executed
when running any ROS 2 node with the ``ROS2_EASY_MODE`` environment variable:

.. code-block:: bash

    ROS2_EASY_MODE=<master_ip> fastdds discovery start -d <domain_id> <master_ip>:<domain_id>

.. note::
    Throughout this tutorial, we will use the term ``master_ip`` to refer to the IP address set in the ``ROS2_EASY_MODE`` environment variable. However, the concept
    of a single master server is no longer strictly valid, as we will use commands that allow us to define multiple remote servers, rather than a single central master.

We will now replicate the previous example (focusing only on domain 1 for simplicity) using the start command. When running the nodes, you will notice a message
indicating that the Discovery Server is already running:

.. code-block:: bash

    # Terminal 3 -> Host B
    ROS2_EASY_MODE=$B_IP fastdds discovery start -d 1 $B_IP:1
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$B_IP ros2 run demo_nodes_cpp listener

    # Terminal 1 -> Host A
    ROS2_EASY_MODE=$B_IP fastdds discovery start -d 1 $B_IP:1
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$B_IP ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello world in domain 1 from HOST_A'}" --once --spin-time 2

.. image:: ../../../../figures/tutorials/core/easy_mode/Diagrams_cli_1.png
    :align: center

.. image:: ../../../../figures/tutorials/core/easy_mode/cli_tutorial_1_start_bien.gif
    :align: center


The ``start`` command requires setting the ``ROS2_EASY_MODE`` environment variable to be set to the master IP address for internal daemon operations. If the value
does not match the one specified in the command, an error will be thrown:

.. image:: ../../../../figures/tutorials/core/easy_mode/cli_tutorial_1_start_error.gif
    :align: center

LIST command
^^^^^^^^^^^^

The ``list`` command is used to list the Discovery Servers running in the host in all different domains.

.. code-block:: bash

    fastdds discovery list

In the following example, we will start three separate Discovery Servers on the same host, each in a different domain, and then list them:

.. code-block:: bash

    # Terminal 1 -> Host A
    ROS2_EASY_MODE=$OWN_IP fastdds discovery start -d 1 $OWN_IP:1
    ROS2_EASY_MODE=$OWN_IP fastdds discovery start -d 2 $OWN_IP:2
    ROS2_EASY_MODE=$OWN_IP fastdds discovery start -d 3 $OWN_IP:3
    fastdds discovery list

.. image:: ../../../../figures/tutorials/core/easy_mode/cli_tutorial_2_list.gif

STOP command
^^^^^^^^^^^^

The ``stop`` command is used to stop Discovery Servers. It can terminate all Discovery Servers running on the same host, regardless of the domain. Alternatively,
the user can specify a domain to stop only the corresponding server, while the daemon remains active.

.. code-block:: bash

    fastdds discovery stop -d <domain_id>

In the following example, we will resume the previous terminal session and stop only the server running in domain 3. After verifying its termination using the ``list``
command, we will proceed to stop all remaining servers and confirm that no servers, including the daemon, are running.


.. code-block:: bash

    # Terminal 1 -> Host A
    ROS2_EASY_MODE=$OWN_IP fastdds discovery start -d 1 $OWN_IP:1
    ROS2_EASY_MODE=$OWN_IP fastdds discovery start -d 2 $OWN_IP:2
    ROS2_EASY_MODE=$OWN_IP fastdds discovery start -d 3 $OWN_IP:3
    fastdds discovery stop -d 3
    fastdds discovery stop

.. image:: ../../../../figures/tutorials/core/easy_mode/cli_tutorial_2_stop.gif

ADD command
^^^^^^^^^^^

The ``add`` command adds new remote Discovery Servers to the local server connecting both servers and their sub-networks without modifying the
existing remote servers.

.. code-block:: bash

    fastdds discovery add -d <domain_id> <remote_ip>:<domain_id>

.. note::
    This command can be used on any server to connect it to any other server, removing the concept of master servers and enabling a fully meshed network where all
    servers can communicate with each other. For example, if A connects to B and B connects to C, then A and C are also connected.

In the following example, we will start Discovery Servers on Host B and Host C, both in domain 1, without pointing to any remote server. Then:

* First, we will start a Discovery Server on Host A, also without pointing to any remote server. Since none of the servers are connected to a remote server, they remain isolated.
  This can be observed as the publisher gets stuck in the "Waiting for at least one matching subscriber" state.
* Next, we will add Host B as a remote server to the Discovery Server on Host A. This establishes a connection between A and B, but Host C still does not receive any data.
* Finally, we will add Host C as a remote server to the Discovery Server on Host B. Due to the meshed network setup, Hosts A, B, and C are now fully connected.

.. image:: ../../../../figures/tutorials/core/easy_mode/Diagrams_cli_3_gif.png
    :align: center


.. image:: ../../../../figures/tutorials/core/easy_mode/cli_tutorial_3_add.gif

SET command
^^^^^^^^^^^

The ``set`` command is used to modify the remote Discovery Servers connected to the local server. This replaces existing remote servers with the newly specified connections.
Its functionality is equivalent to stopping the server and restarting it with the new connections. Due to this ``start``-like behaviour, the ``ROS2_EASY_MODE`` environment variable
must be set to the master IP address.

.. code-block:: bash

    ROS2_EASY_MODE=<master_ip> fastdds discovery set -d <domain_id> <master_ip>:<domain_id>

In the following example, we will start a Discovery Server in Host A, initially pointing to Host B in domain 1. We will publish a message to verify that data is received on Host B
but not on Host C. Then, we will update the remote server to point to Host C in domain 1 and observe that the connection successfully switches, allowing data to be received on
Host C instead of Host B.

.. code-block:: bash

    # Terminal 3 -> Host B
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$B_IP ros2 run demo_nodes_cpp listener

    # Terminal 4 -> Host C
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$C_IP ros2 run demo_nodes_cpp listener

    # Terminal 1 -> Host A
    ROS2_EASY_MODE=$B_IP fastdds discovery start -d 1 $B_IP:1
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$B_IP ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello world in domain 1 from HOST_A'}" --once --spin-time 2
    ROS2_EASY_MODE=$C_IP fastdds discovery set -d 1 $C_IP:1
    ROS_DOMAIN_ID=1 ROS2_EASY_MODE=$C_IP ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello world in domain 1 from HOST_A'}" --once --spin-time 2

.. image:: ../../../../figures/tutorials/core/easy_mode/Diagrams_cli_4_gif.png
    :align: center

.. image:: ../../../../figures/tutorials/core/easy_mode/cli_tutorial_4_set.gif


With the examples shown in this tutorial, we have demonstrated how to manage Discovery Server connections dynamically using CLI commands. By leveraging these tools, it is possible to
create flexible and fully connected networks, modify connections on the fly, and control the discovery process efficiently. This provides greater adaptability in distributed ROS 2
systems, ensuring seamless communication between nodes across different domains and hosts.


