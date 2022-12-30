.. _tutorials_qos_mutable_mutable:

Change mutable QoS through get native API
=========================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Fast DDS over Vulcanexus offers the possibility of not only configuring the QoS policy when creating ROS nodes, but also to modify the mutable ones after the node/publisher/subscription creation.
The QoS that allow their modification after entity creation, are called mutable QoS.
Please refer to `Fast DDS QoS Policy documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/policy.html>`_ to get a list of all supported QoS and whether they are mutable.

:ref:`tutorials_qos_ownership_ownership` explained the use of the Ownership and Ownership Strength QoS (see `Ownership QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#ownershipqospolicy>`_) and how to configure them within the ROS 2 talker/listener demo.

This tutorial will show how to change Ownership Strength QoS in runtime, after all nodes have been already deployed.
This will be done through the interaction of three nodes (one subscriber and two publishers):
After creation, the subscriber will only be receiving data from the publisher with the largest ownership strength (corresponding to exclusive ownership QoS).
After that, the ownership strength of the other publisher will be changed to become larger than that of the first one, thus making the subscriber to start showing the data of the latter publisher.

This will be done creating a custom package, following similar steps as in :ref:`<intermediate_monitor_parameters>` to be able to change a node's parameter, and respond to that change by changing the Partition QoS of the publisher.

The ROS 2 middleware layer (see :ref:`<concepts_ros2_vendors>`) provides APIs to get handles to the objects of the inner DDS implementation, which is needed to be able to change the mutable Qos.
Thus, this tutorial will also show how to use that powerful tool.
For another demo on how to access inner RMW entities, see `demo_nodes_cpp_native <https://github.com/ros2/demos/tree/humble/demo_nodes_cpp_native>`_.

Prerequisites
-------------

The first prerequisite is to have Vulcanexus Humble installed (see :ref:`Linux binary installation <linux_binary_installation>` or :ref:`Linux installation from sources <linux_source_installation>`).

Also, before starting this tutorial, user should be familiar with creating a workspace and creating a package, as well as familiar with parameters and their function in a ROS 2 system.
The recommendation is to first complete the following tutorials:

    * :ref:`tutorials_qos_ownership_ownership`

    * :ref:`CppPubSub`

    * :ref:`intermediate_monitor_parameters`

    * :ref:`CppParamNode`

    * :ref:`intermediate_monitor_parameters`

This tutorial focuses on the explanations regarding mutable QoS change in runtime and regarding access to innerobjects of DDS middleware implementation, and for that reason not all the code is going to be explained, as it is already explained in the aforementioned tutorials.
Create a clean workspace and download the Vulcanexus - Change Mutable QoS Through get_native API project:

.. code-block:: bash

    # Create directory structure
    mkdir ~/vulcanexus_ws
    mkdir ~/vulcanexus_ws/src
    mkdir ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos
    mkdir ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos/src

    # Download profile config files for Fast DDS participants
    wget -O large_ownership_strength.xml https://raw.githubusercontent.com/eProsima/vulcanexus/humble/docs/resources/tutorials/core/qos/mutable/large_ownership_strength.xml
    wget -O small_ownership_strength.xml https://raw.githubusercontent.com/eProsima/vulcanexus/humble/docs/resources/tutorials/core/qos/mutable/small_ownership_strength.xml
    
    # Download project source code
    cd ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos
    wget -O CMakeLists.txt https://raw.githubusercontent.com/eProsima/vulcanexus/humble/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/CMakeLists.txt
    wget -O package.xml https://raw.githubusercontent.com/eProsima/vulcanexus/humble/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/package.xml

    cd ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos/src
    wget -O change_mutable_qos_pub https://raw.githubusercontent.com/eProsima/vulcanexus/humble/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_pub.cpp
    wget -O change_mutable_qos_sub https://raw.githubusercontent.com/eProsima/vulcanexus/humble/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_sub.cpp

The resulting directory structure should be:

.. code-block:: bash

    ~/vulcanexus_ws/
    ├── large_ownership_strength.xml
    ├── small_ownership_strength.xml
    └── src
        └── vulcanexus_change_mutable_qos
            ├── CMakeLists.txt
            ├── package.xml
            └── src
                ├── change_mutable_qos_pub.cpp
                └── change_mutable_qos_sub.cpp

Finally, the package can be built.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    cd ~/vulcanexus_ws
    colcon build

Explaining the source code
--------------------------

In the case of the Publishers, the code is analogous, so here the code is going to be explained just for Publisher 1.
For the case of the Subscriber, this tutorial is not going to explain it, as it is just the minimal subscriber, listening on the topic `/chatter`, already explained in the :ref:`Writing a simple publisher and subscriber (C++) <CppPubSub>` tutorial.

For the Publisher, here not all the code is going to be explained, as the referred tutorials of the prerequisites section explain big part of it.
For instance, the `/chatter` temporized publisher is explained in the :ref:`Writing a simple publisher and subscriber (C++) <CppPubSub>`, and the mechanism to respond by means of a user callback to a change in a node's parameter is explained in :ref:`Monitoring for parameter changes (C++) <MonitorParams>`.

The `demo_nodes_cpp_native <https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native>`_ shows how to access inner RMW and Fast DDS entities, although it is not actually explained.
In this tutorial, that same mechanism is used.
In the private section of the `Node_ChangeMutableQoS_PubX` class, the pointers to the native handlers are declared:

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_pub1.cpp
    :language: c++
    :lines: 82-85
    :dedent: 4


In the constructor, the pointers are populated by calling the APIs provided by the rmw and rmw_fastrtps_cpp, until obtaining the `eprosima::fastdds::dds::DataWriter` handle:

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_pub1.cpp
    :language: c++
    :lines: 42-45
    :dedent: 4


When the `pubX_ownership_strength` is updated (for instance, via command line using `ros2 param set` command), the `cb` parameter callback is raised, and the `eprosima::fastdds::dds::DataWriter` handle is used to update its ownership strength.

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_pub1.cpp
    :language: c++
    :lines: 54-72
    :dedent: 8

In this case, as in the current version of Fast DDS the builtin statistics are enabled by default (see `DomainParticipantQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#domainparticipantqos>`_), it is needed to retrieve the internal QoS by means of `::get_qos()`, then perform the modifications and update the QoS by means of `::set_qos()`:
The value of the ownership strength is set from the value of the updated parameter.

Update CMakeLists.txt and package.xml
-------------------------------------

We need to add the instructions to compile the new source files, and to account for its dependencies both in CMakeLists.txt and package.xml files.

Make sure that the find_package lines in the CMakeLists.txt are the following, so substitute what you have for the following lines:

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/CMakeLists.txt
    :language: ccmake
    :lines: 9-12


Then add the following lines to compile and install each node:

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/CMakeLists.txt
    :language: ccmake
    :lines: 14-36


Inside package.xml file, make sure that the <depend> tags, are the following, so substitute what you have for the following lines:

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/package.xml
    :language: xml
    :lines: 12-14


Configure initial QoS
---------------------

Ownership Strength Policy is mutable, but Ownership Policy is not. Then, it is needed to configure EXCLUSIVE_OWNERSHIP_POLICY to all participants before running the ROS nodes.
To do that, create a new xml file in the root of the workspace:

.. code-block:: bash

    cd ~/vulcanexus_ws
    touch profiles1.xml

Open the newly created file with your preferred editor and paste the following xml code:

.. literalinclude:: /resources/tutorials/core/qos/mutable/profiles1.xml
    :language: xml


This xml includes one profile for a publisher (data writer) and one profile for a subscriber (data reader), and sets them to exclusive ownership, and ownership strength of value 10 for the publisher.
This will be applied to the Publisher 1 and to the Subscriber.
We need another profile in a separate file to assign a different ownership strength to the Publisher 2:

.. code-block:: bash

    touch profiles2.xml


.. literalinclude:: /resources/tutorials/core/qos/mutable/profiles2.xml
    :language: xml


This will assign an ownership strength of value 2 to the Publisher 2.

Build
-----

Now the package is ready to be built.
Change your directory to the workspace folder and build using colcon:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    cd ~/vulcanexus_ws
    colcon build


Run
---

Open three terminals in the workspace folder.
On each, you need to source Vulcanexus installation, as well as the package installation.
Then, export the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to point out to the corresponding profiles file.
And run the ROS 2 node.

First, in the first terminal, run the subscriber node, configured with the profiles1.xml file:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    cd ~/vulcanexus_ws
    . install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=./profiles1.xml
    ros2 run vulcanexus_change_mutable_qos change_mutable_qos_sub


Then, in another terminal, run the first publisher, configured also with the profiles1.xml file.
This Publisher will then be configured with ownership strength value of 10.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    cd ~/vulcanexus_ws
    . install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=./profiles1.xml
    ros2 run vulcanexus_change_mutable_qos change_mutable_qos_pub1


At this point you will be able to see that both nodes are communicating, and the messages from Publisher 1 can be seen in the Subscriber.

In the third terminal, run the second publisher, configured with the profiles2.xml file.
This Publisher will then be configured with ownership strength value of 2.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    cd ~/vulcanexus_ws
    . install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=./profiles2.xml
    ros2 run vulcanexus_change_mutable_qos change_mutable_qos_pub2


This Publisher 2 starts sending messages (it can be seen that the number of the message starts from 1 while the messages from Publisher 1 are already in a higher number), and the Subscriber is still receiving messages from Publisher 1 and not from Publisher 2.
This is because of the exclusive ownership.
Publisher 1 has higher ownership strength than Publisher 2.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/qos/mutable/first_launch.mp4">
        Your browser does not support the video tag.
    </video>


Change mutable QoS via command line
-----------------------------------

In this last section the ROS command `param set` will be used to change the value of the node's parameter created earlier.
The parameter change will cause the parameter-change callback to be called, and then resulting in a change in the ownership strength.
In another terminal, try the following code:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    cd ~/vulcanexus_ws
    . install/setup.bash
    ros2 param set /node2_change_mutable_qos pub2_ownership_strength 50


With that execution, the ownership strength of the Publisher 2 has changed to become bigger than that of the Publisher 1.
You now should be watching the Subscriber receiving the messages from the Publisher 2 and not from the Publisher 1.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/qos/mutable/param_change.mp4">
        Your browser does not support the video tag.
    </video>
