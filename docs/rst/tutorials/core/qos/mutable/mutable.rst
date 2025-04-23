.. _tutorials_qos_mutable_mutable:

Change mutable QoS through get native API
=========================================

Background
----------

Fast DDS over Vulcanexus offers the possibility of not only configuring the QoS policy when creating ROS nodes, but also to modify the mutable ones after the node/publisher/subscription creation.
The QoS that allow their modification after entity creation, are called mutable QoS.
Please refer to `Fast DDS QoS Policy documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/policy.html>`_ to get a list of all supported QoS and whether they are mutable.

:ref:`tutorials_qos_ownership_ownership` explained the use of the Ownership and Ownership Strength QoS (see `Ownership QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#ownershipqospolicy>`_) and how to configure them within the ROS 2 talker/listener demo.

This tutorial will show how to change Ownership Strength QoS in runtime, after all nodes have been already deployed.
This will be done through the interaction of three nodes (one subscriber and two publishers):
After creation, the subscriber will only be receiving data from the publisher with the largest ownership strength (corresponding to exclusive ownership QoS).

.. figure:: /rst/figures/tutorials/core/qos/mutable/publication_graph_before.png
   :align: center

After that, the ownership strength of the other publisher will be changed to become larger than that of the first one, thus making the subscriber to start showing the data of the latter publisher.

.. figure:: /rst/figures/tutorials/core/qos/mutable/publication_graph_after.png
   :align: center

This will be done creating a custom package, following similar steps as in :ref:`intermediate_monitor_parameters` to be able to change a node's parameter, and respond to that change by changing the Partition QoS of the publisher.

The ROS 2 middleware layer (see :ref:`concepts_ros2_vendors`) provides APIs to get handles to the objects of the inner DDS implementation, which is needed to be able to change the mutable Qos.
Thus, this tutorial will also show how to use that powerful tool.
For another demo on how to access inner RMW entities, see `demo_nodes_cpp_native <https://github.com/ros2/demos/tree/jazzy/demo_nodes_cpp_native>`__.

Prerequisites
-------------

The first prerequisite is to have Vulcanexus jazzy installed (see :ref:`Linux binary installation <linux_binary_installation>` or :ref:`Linux installation from sources <linux_source_installation>`).

Also, before starting this tutorial, user should be familiar with creating a workspace and creating a package, as well as familiar with parameters and their function in a ROS 2 system.
The recommendation is to first complete the following tutorials:

    * :ref:`tutorials_qos_ownership_ownership`

    * :ref:`CppPubSub`

    * :ref:`ROS2Params`

    * :ref:`CppParamNode`

    * :ref:`intermediate_monitor_parameters`

This tutorial focuses on the explanations regarding mutable QoS change in runtime and regarding access to inner objects of DDS middleware implementation, and for that reason, not all the code is going to be explained, as it is already explained in the aforementioned tutorials.
Create a clean workspace and download the Vulcanexus - Change Mutable QoS Through get_native API project:

.. code-block:: bash

    # Create directory structure
    mkdir ~/vulcanexus_ws
    mkdir ~/vulcanexus_ws/src
    mkdir ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos
    mkdir ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos/src

    # Download project source code
    cd ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos
    wget -O CMakeLists.txt https://raw.githubusercontent.com/eProsima/vulcanexus/jazzy/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/CMakeLists.txt
    wget -O package.xml https://raw.githubusercontent.com/eProsima/vulcanexus/jazzy/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/package.xml

    cd ~/vulcanexus_ws/src/vulcanexus_change_mutable_qos/src
    wget -O change_mutable_qos_publisher.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/jazzy/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_publisher.cpp

    # Download profile config files for Fast DDS participants
    wget -O large_ownership_strength.xml https://raw.githubusercontent.com/eProsima/vulcanexus/jazzy/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/large_ownership_strength.xml
    wget -O small_ownership_strength.xml https://raw.githubusercontent.com/eProsima/vulcanexus/jazzy/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/small_ownership_strength.xml
    wget -O subscriber_exclusive_ownership.xml https://raw.githubusercontent.com/eProsima/vulcanexus/jazzy/docs/resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/subscriber_exclusive_ownership.xml


The resulting directory structure should be:

.. code-block:: bash

    ~/vulcanexus_ws/
    └── src
        └── vulcanexus_change_mutable_qos
            ├── CMakeLists.txt
            ├── package.xml
            └── src
                ├── change_mutable_qos_publisher.cpp
                ├── large_ownership_strength.xml
                ├── small_ownership_strength.xml
                └── subscriber_exclusive_ownership.xml


Explaining the source code
--------------------------

In the case of the Subscriber, this tutorial only needs a minimal subscriber listening on the topic `/chatter`.
For convenience, the `listener` node from `demo_nodes_cpp` package will be used, as it is just a minimal subscriber listening for the aforementioned topic.
For more information on how to create a minimal subscriber, :ref:`CppPubSub` tutorial shows how to write one.

In the case of the Publishers, the package is using only one executable, which takes an argument to assign the name of the Node.
Here not all the code is going to be explained, as the referred tutorials of the prerequisites section explain big part of it.
For instance, the `/chatter` temporized publisher is explained in the :ref:`CppPubSub`, and the mechanism to respond by means of a user callback to a change in a node's parameter is explained in :ref:`intermediate_monitor_parameters`.

The `demo_nodes_cpp_native <https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native>`__ shows how to access inner RMW and Fast DDS entities, although it is not actually explained.
In this tutorial, that same mechanism is used.
In the private section of the :class:`Node_ChangeMutableQoS_Publisher` class, the pointers to the native handlers are declared:

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_publisher.cpp
    :language: c++
    :lines: 93-96
    :dedent: 2


In the constructor, the pointers are populated by calling the APIs provided by the rmw and rmw_fastrtps_cpp, until obtaining the :class:`eprosima::fastdds::dds::DataWriter` handle:

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_publisher.cpp
    :language: c++
    :lines: 56-59
    :dedent: 4


When the `Publisher_X_ownership_strength` is updated (for instance, via command line using `ros2 param set` command), the parameter callback is raised, and the `eprosima::fastdds::dds::DataWriter` handle is used to update its ownership strength.
Below, a snippet of code from the constructor of the node, where the parameter is declared, the subscription to its changes is registered, and the callback to be run on the parameter change event is defined.

.. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/change_mutable_qos_publisher.cpp
    :language: c++
    :lines: 61-82
    :dedent: 4


In this case, as in the current version of Fast DDS the builtin statistics are enabled by default (see `DomainParticipantQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#domainparticipantqos>`_), it is needed to retrieve the internal QoS by means of `::get_qos()`, then perform the modifications and update the QoS by means of `::set_qos()`:
The value of the ownership strength is set from the value of the updated parameter.

Configuration of initial QoS
----------------------------

Ownership Strength Policy is mutable, but Ownership Policy is not, so configuring EXCLUSIVE_OWNERSHIP_POLICY to all participants before running the ROS nodes is needed.
To do that, inside the package, there are three xml files.
Each one of them defines a profile for a publisher with a "large" ownership strength, another with a "small" ownership strength and a subscriber (that does not need an ownership strength definition).
For the three of them, exclusive ownership is defined.

.. tab-set::

    .. tab-item:: large_ownership_strength.xml

        .. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/large_ownership_strength.xml
            :language: xml

    .. tab-item:: small_ownership_strength.xml

        .. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/small_ownership_strength.xml
            :language: xml

    .. tab-item:: subscriber_exclusive_ownership.xml

        .. literalinclude:: /resources/tutorials/core/qos/mutable/vulcanexus_change_mutable_qos/src/subscriber_exclusive_ownership.xml
            :language: xml


Build
-----

Now the package is ready to be built.
Change the directory to the workspace folder and build using colcon:

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash
    cd ~/vulcanexus_ws
    colcon build


Run
---

Open three terminals in the workspace folder.
On each of them, Vulcanexus installation, as well as the package installation is needed.
Then, export the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to point out to the corresponding profiles file and run the node.

    * In the first terminal, run the `listener` node from the `demo_nodes_cpp`, configured with the `subscriber_exclusive_ownership.xml` file.
    * Then, in another terminal, run the first publisher, configured also with the `large_ownership_strength.xml` file.
      This Publisher will then be configured with ownership strength value of 10.
      At this point both nodes should be communicating, and the messages from Publisher 1 should be shown in the Subscriber.
    * In the third terminal, run the second publisher, configured with the `small_ownership_strength.xml` file.
      This Publisher will then be configured with ownership strength value of 2.
      This Publisher 2 starts sending messages (it can be seen that the number of the message starts from 1 while the messages from Publisher 1 are already in a higher number), and the Subscriber is still receiving messages from Publisher 1 and not from Publisher 2.
      This is because of the exclusive ownership.

The code to execute in each terminal can be found in the tabs below:

.. tab-set::

    .. tab-item:: First terminal

        .. code-block:: bash

            source /opt/vulcanexus/jazzy/setup.bash
            cd ~/vulcanexus_ws
            `# Using profile to set exclusive ownership`
            export FASTRTPS_DEFAULT_PROFILES_FILE=./install/vulcanexus_change_mutable_qos/profiles/subscriber_exclusive_ownership.xml
            ros2 run demo_nodes_cpp listener  `# Run minimal subscriber`

    .. tab-item:: Second terminal

        .. code-block:: bash

            source /opt/vulcanexus/jazzy/setup.bash
            cd ~/vulcanexus_ws
            source install/setup.bash
            `# Using profile to set large strenght value`
            export FASTRTPS_DEFAULT_PROFILES_FILE=./install/vulcanexus_change_mutable_qos/profiles/large_ownership_strength.xml
            ros2 run vulcanexus_change_mutable_qos change_mutable_qos_publisher Publisher_1     `# Run Publisher 1`

    .. tab-item:: Third terminal

        .. code-block:: bash

            source /opt/vulcanexus/jazzy/setup.bash
            cd ~/vulcanexus_ws
            source install/setup.bash
            `# Using profile to set small strenght value`
            export FASTRTPS_DEFAULT_PROFILES_FILE=./install/vulcanexus_change_mutable_qos/profiles/small_ownership_strength.xml
            ros2 run vulcanexus_change_mutable_qos change_mutable_qos_publisher Publisher_2     `# Run Publisher 2`


**Publisher 1 has higher ownership strength than Publisher 2**.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/qos/mutable/first_launch.mp4">
        Your browser does not support the video tag.
    </video>


Change mutable QoS via command line
-----------------------------------

In this last section, the `param set` command will be used to change the value of the node's parameter created earlier.
The parameter change will cause the parameter-changed callback to be called, which then results in a change in the ownership strength.
In another terminal, try the following code:

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash
    cd ~/vulcanexus_ws
    source install/setup.bash
    ros2 param set /Publisher_2_change_mutable_qos Publisher_2_ownership_strength 50


With that execution, the ownership strength of the Publisher 2 has changed to become larger than that of the Publisher 1.
**Now the Subscriber should be receiving the messages from the Publisher 2 and not from the Publisher 1**.

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/qos/mutable/param_change.mp4">
        Your browser does not support the video tag.
    </video>
