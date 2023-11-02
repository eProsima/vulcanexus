.. _tutorials_deployment_discovery_server_init:

Discovery Server
==============================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

By default, ROS 2 uses the Simple Discovery Protocol (SDP), which relies on multicast communication to discover other nodes.
While SDP is suitable for many scenarios, it may not be the best choice in complex and large-scale network configurations.
This is where the Fast DDS (Data Distribution Service) Discovery Server comes into play. Fast DDS offers an alternative discovery mechanism that allows ROS 2 nodes to connect to a Discovery Server.
This server acts as a centralized point for node registration and discovery, providing more flexibility and control in managing the network of nodes and is particularly advantageous in scenarios such as:

* Large networks
  
* Complex networks
  
* Dynamical environments

In this tutorial, we will explore how to set up and use the Fast DDS Discovery Server with ROS 2, enabling robust and dynamic node discovery communication.

.. figure:: /rst/figures/tutorials/core/discovery_server/Discovery_Server.svg
   :align: center
   :scale: 150%


Prerequisites
-------------

It is required to have previously installed Vulcanexus using one of the following installation methods:

* `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`__
* `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`__
* `Docker installation <https://docs.vulcanexus.org/en/latest/rst/installation/docker.html>`__

In this tutorial we will follow the instructions explained in *Configuring environment* to set up a Docker Image with the aim of using `source ROS 2 <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html>`__ with *Fast DDS* discovery tool.

Configuring environment
------------------------
A Docker image is a self-contained and executable package that includes all the necessary components to run an application, such as code, libraries, dependencies, and runtime environment.
In this tutorial the configuration chosen for setting up Vulcanexus in our computer will be contained in a Docker image which contains Vulcanexus’s Desktop installation.
This containerized environment can be found in `Vulcanexus's Downloads <https://vulcanexus.org/download>`__, where you will find a number of different Docker images to set up the environment for this tutorial.

1. First install Docker:
   
.. code-block:: bash

    sudo apt install docker.io

2. Then load the downloaded image:
   
.. code-block:: bash

    docker load -i ubuntu-vulcanexus-iron-desktop.tar

3. Finally run the Docker image and source it:
   
.. code-block:: bash

    xhost local:root
    docker run \
        -it \
        --privileged \
        --net host \
        --ipc host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        ubuntu-vulcanexus:iron-desktop       #Downloaded Docker image
    source /opt/vulcanexus/iron/setup.bash
 

In every new terminal that we open to start another session within the same container run the following commands to have access to the image:

.. tabs::

    .. tab:: Execution

        .. code-block:: bash

            docker exec -it ubuntu-vulcanexus:iron-desktop bash
           
    .. tab:: Sourcing

        .. code-block:: bash

            source /opt/vulcanexus/iron/setup.bash



Run this tutorial
------------------

Once we finish with the set up of Vulcanexus environment we are ready to start with the *talker-listener* demo tutorial. 
In this demo both a ``talker`` and a ``listener`` nodes are created: the talker node will publish a "hello world" message every second, while the listener node will listen to these messages.
By sourcing ROS 2 you will get access to the CLI tool ``fastdds``. This tool gives access to the `discovery tool <https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html#discovery>`__, which can be used to launch a discovery server. This server will manage the discovery process for the nodes that connect to it.

.. important::

    Do not forget to source ROS 2 every time you open another session to have access to the CLI tool ``fastdds``.


Setup Discovery Server
^^^^^^^^^^^^^^^^^^^^^^
Start by launching a discovery server with id 0, port 11811 (default port) and listening on all available interfaces.
The role of the server is to re-distribute the clients (and servers) discovery information to their known clients (and servers).

.. code-block:: bash

    fastdds discovery --server-id 0


Launch Talker Node
^^^^^^^^^^^^^^^^^^^^^^
Execute the Talker demo to publish on the ``/chatter`` topic. When setting the ``ROS_DISCOVERY_SERVER`` environment variable you are indicating that ROS node should act as a client that connects to a discovery server to discover other nodes on the network.
Use the argument --remap __node:=talker_discovery_server to change the node’s name for this tutorial.

.. code-block:: bash

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811  #Localhost:127.0.0.1, port:11811 (default)
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server


Launch Listener Node
^^^^^^^^^^^^^^^^^^^^^^
Execute the Listener demo to listen to the ``/chatter`` topic.

.. code-block:: bash

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server


