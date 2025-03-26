.. _dds2vulcanexus_ros2idl:

Communication between Vulcanexus and Fast DDS using ROS 2 generated IDLs
========================================================================

This tutorial presents a step-by-step demonstration on how to intercommunicate Vulcanexus applications with native Fast DDS applications, when using ROS 2 generated IDLs.

.. contents::
    :depth: 2
    :local:
    :backlinks: none

.. _dds2vulcanexus_ros2idl_prerequisites:

Prerequisites
-------------

For convenience, this tutorial is built and run using Docker environments to avoid requiring local installations, although Docker is not required. All steps can be executed in a local environment, provided the necessary dependencies are properly installed.
The tutorial focuses on the explanations regarding topic compatibility rather than giving an in depth explanation about the code used. The only prerequisites are, thus, downloading both Docker images.

For the ROS 2 Docker image, the following command can be used to download the latest version of Vulcanexus Jazzy Jolo:

.. code-block:: bash

    docker pull eprosima/vulcanexus:jazzy-desktop

The Fast DDS Docker image can be found on the `eProsima website <https://www.eprosima.com/product-download>`_.

.. note::

    For this tutorial, FastDSS suite 3.2 or higher is necessary.

Vulcanexus application
-----------------------

To start the Vulcanexus application, we initiate the Docker container, and source the appropriate repository:

.. code-block:: bash

    docker run -it --rm eprosima/vulcanexus:jazzy-desktop
    source opt/vulcanexus/jazzy/setup.bash

For this tutorial, we will use a simple example ROS 2 talker already installed in Vulcanexus, which sends periodically string messages to the chatter topic.

In this example, we'll use domain :code:`42`. To configure the ROS 2 talker accordingly, set the following environment variable before launching it:

.. code-block:: bash

    export ROS_DOMAIN_ID=42

To run the example, simply execute the following command:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

With this, we have a ROS 2 publisher running in the Vulcanexus container. We won't need to interact with it anymore for the purposes of the tutorial.

Fast DDS Monitor
----------------

To inspect the messages being sent by the ROS 2 publisher, we will use Fast DDS Monitor, a graphical Desktop application aimed at monitoring DDS environments deployed using the eProsima Fast DDS library.
We will be using the Fast DDS Docker image downloaded previously, which can be run with the following commands:

.. code-block:: bash

    cd <path_to_fastdds_docker_image>
    docker load -i ubuntu-fastdds-suite\ <FastDDS-Version>.tar
    xhost local:root
    docker run -it --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \ubuntu-fastdds-suite:<FastDDS-Version>

Now that the container is up and running, we can access the Fast DDS Monitor:

.. code-block:: bash

    fastdds_monitor

.. note::

    In this tutorial we are not interested in the statistical analysis capabilities of Fast DDS Monitor, but rather in the ability to visualize the type IDL information from the messages being sent by the ROS 2 talker. For more information on the Fast DDS Monitor capabilities, please refer to :ref:`tutorials_tools_fastdds_monitor`.

We can now see the starting screen of Fast DDS Monitor, where we click on :code:`Start monitoring!` to initiate the program.

.. image:: /rst/figures/tutorials/core/ros2_idl/Monitor_cover.png
    :align: center

The next step is the selection of the domain. For this tutorial, we will choose Domain :code:`42`.

.. image:: /rst/figures/tutorials/core/ros2_idl/Monitor_domain.png
    :align: center

Selecting the Domain View and expanding all the different panels in the explorer view, we can inspect the status of our network, composed of a single ROS 2 talker with two writers and one reader.

.. image:: /rst/figures/tutorials/core/ros2_idl/Monitor_talker.png
    :align: center

But this is not all the information that is being exchanged in this DDS network. To see all the topics existing in the network, we need to click on :code:`View` and select :code:`Show Metatraffic`. After pressing *Refresh*, this reveals many more topics, writers and readers present in our current network.

.. image:: /rst/figures/tutorials/core/ros2_idl/Monitor_fulltalker.png
    :align: center

For the purposes of this tutorial, we are going to select a topic which is already known to lead to incompatibilities if the bare ROS 2 IDL is used: :code:`ros_discovery_info`. Finding the topic and pressing right-click over the topic name, then choosing *Data type IDL view*, we can see the IDL representation of the topic.
Note that by default, the monitor will perform a demangling operation over the ROS 2 type messages received, removing the `dds_` namespace and modules among other modifications. The info sign on the top right corner informs the user when this operation has been performed.

.. image:: /rst/figures/tutorials/core/ros2_idl/Monitor_idl.png
    :align: center

Creating the readers and writers
--------------------------------

First, we are now going to create a new folder in the Fast DDS docker to contain our code. To do so, run:

.. code-block:: bash

    mkdir my_IDL
    cd my_IDL/

For the next steps of the tutorial, in which we create the DataReaders and DataWriters, there are two different approaches.

Option 1: Demangled IDL
~~~~~~~~~~~~~~~~~~~~~~~~

We can directly employ the demangled IDL shown by default in the Fast DDS Monitor. In the IDL view, right-clicking on the screen and selecting *Copy* will allow us to copy the full IDL information into the clipboard.
We can then paste this information into a new file, which we will name `ParticipantEntitiesInfo.idl`. First we will create the file, then open it with the program nano (any other program to pen the file may be used).
To do so, run the following commands:

.. code-block:: bash

    touch ParticipantEntitiesInfo.idl
    apt-get update
    apt-get install nano
    nano ParticipantEntitiesInfo.idl

In the nano editor, paste the copied IDL information. Due to reserved words in Fast DDS operation, there is a small modification we need to apply, which will be deleted later in the automatic operation of our program.
Inside the IDL file, replace the line :code:`rmw_dds_common::msg::Gid gid;`` with :code:`rmw_dds_common::msg::Gid _gid;`, that is, add an underscore before gid since gid is a reserved member name.
Then, save and exit by pressing :code:`Ctrl+X`, then :code:`Y` and :code:`Enter`. This is the IDL file we will use to create the DataReaders and DataWriters.
To create the necessary code, we will use a tool called `Fast DDS-Gen <https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/introduction/introduction.html>`_. This tool will automatically generate the necessary code to create the DataReaders and DataWriters.
To generate the code, run the following command:

.. code-block:: bash

    fastddsgen -example CMake -typeros2 ParticipantEntitiesInfo.idl

Option 2: Mangled ROS 2 IDL
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A different approach is to use the mangled ROS 2 IDL directly. To do so, in the monitor, go to *View->Revert ROS 2 Demangling* to recover the IDL as received by the monitor, then in the IDL view, right-clicking on the screen and selecting *Copy* will allow us to copy the full IDL information into the clipboard.
We can then paste this information into a new file, which we will name `ParticipantEntitiesInfo.idl`. First we will create the file, then open it with the program nano (any other program to open the file may be used).
To do so, run the following commands:

.. code-block:: bash

    touch ParticipantEntitiesInfo.idl
    apt-get update
    apt-get install nano
    nano ParticipantEntitiesInfo.idl

In the nano editor, paste the copied IDL information. Then, save and exit by pressing :code:`Ctrl+X`, then :code:`Y` and :code:`Enter`. This is the IDL file we will use to create the DataReaders and DataWriters.
To create the necessary code, we will use a tool called `Fast DDS-Gen <https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/introduction/introduction.html>`_.
This tool will automatically generate the necessary code to create the DataReaders and DataWriters. To generate the code, run the following command:

.. code-block:: bash

    fastddsgen -example CMake ParticipantEntitiesInfo.idl

Launching a Fast DDS DataReader
-------------------------------

For the purpose of this tutorial, we will only be creating a single DataReader to connect with the ROS 2 talker. The code for the DataReader was generated in the previous step, and requires only a few simple modifications.
Opening the file `ParticipantEntitiesInfomain.cxx`, we need to perform a small modification to the code. The line :code:`int domain_id = 0;` must be replaced with :code:`int domain_id = 42;`, so our participants are created in the same domain as the ROS 2 talker.
Save and exit by pressing :code:`Ctrl+X`, then :code:`Y` and :code:`Enter`.

Additionally, we need to ensure that the DataReader joins the same topic as the ROS 2 talker, so we need to perform an additional modification. Open the file `ParticipantEntitiesInfoSubscriberApp.cxx`, and replace the line

.. code-block:: cpp

    topic_ = participant_->create_topic("ParticipantEntitiesInfoTopic", type_.get_type_name(), topic_qos);

with

.. code-block:: cpp

    topic_ = participant_->create_topic("ros_discovery_info", type_.get_type_name(), topic_qos);

This same operation needs to be done on the file `ParticipantEntitiesInfoPublisherApp.cxx` if we wanted a DataWriter to connect to this topic too.

After these modifications, we can compile the code using CMake by running the following commands:

.. code-block:: bash

    mkdir build
    cd build
    apt-get install -y cmake
    cmake ..
    make

This created an application we can directly run from the command line. We can now launch the DataReader by running:

.. code-block:: bash

    ./ParticipantEntitiesInfo subscriber

The connection will be made between them, as can be illustrated by the monitor itself in the Domain View, which can be filtered to see only endpoints associated to `ros_discovery_info` topic (right-click over the Topic name, and choose Filter topic graph, opening the filtered graph in a new Tab). Additionally, the following message will be displayed by the console:

.. code-block:: bash

    rmw_dds_common::msg::ParticipantEntitiesInfo Subscriber matched.
