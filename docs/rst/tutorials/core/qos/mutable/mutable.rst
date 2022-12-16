.. _tutorials_qos_mutable_mutable:

Change mutable QoS through get native API
=========================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Fast DDS over Vulcanexus offers the possibility of not only configuring the QoS policy when creating ROS nodes, but it is also possible to modify them (some of them) after having created the ROS node.
The QoS that allow their modification after that, are called mutable QoS.

`Here <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/policy.html>`_ you can find reference of all supported QoS Policy, and you can check wether any QoS is mutable in these links:

    * `DataReader QoS <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/subscriber/dataReader/dataReader.html?highlight=mutable#datareaderqos>`_
    * `DataWriter QoS <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/dataWriter/dataWriter.html?highlight=mutable#datawriterqos>`_
    * `Topic QoS <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/topic/topic.html?highlight=mutable#topicqos>`_
    * `DomainParticipant QoS <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html?highlight=mutable#domainparticipantqos>`_
    * `Publisher QoS <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/publisher/publisher.html?highlight=mutable#publisherqos>`_
    * `Subscriber QoS <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/subscriber/subscriber/subscriber.html?highlight=mutable#subscriberqos>`_

Another :ref:`tutorial <tutorials_qos_ownership_ownership>` explained the use of the Ownership and Ownership Strength QoS (see `Ownership QoS Policy <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#ownershipqospolicy>`_) and how to configure it within the ROS 2 talker/listener demo.
This tutorial will show how to change Ownership Strength QoS in runtime, after all nodes have been already deployed.
We will see how three nodes (one subscriber and two publishers) interact: After creation the subscriber will only be receiving data from the publisher with bigger ownership strength (corresponding to exclusive ownership QoS), and after that we will change the ownership strength of the other one to become bigger, thus the subscriber will start to show the data of the latter.

This will be done creating a custom package, following similar steps as in this `ROS 2 tutorial <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html>`_ to be able to change a node's parameter, and respond to that change by changing the Partition QoS of the publisher.

The ROS MiddleWare (see `ROS 2 RMW <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Concepts/About-Different-Middleware-Vendors.html?highlight=RMW#default-rmw-implementation>`_) provides APIs to get handles to the objects of the inner DDS implementation.
That is needed to be able to change the mutable Qos.
Thus, this tutorial will also show how to use that powerful tool.
For another demo on how to access inner RMW entities, see `demo_nodes_cpp_native <https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native>`_.

Prerequisites
-------------

The first prerequisite is to have Vulcanexus Humble installed (see `Linux binary installation <https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html>`_ or `Linux installation from sources <https://docs.vulcanexus.org/en/latest/rst/installation/linux_source_installation.html>`_).

Also before starting this tutorial, you should be familiar with creating a workspace and creating a package, as well as familiar with parameters and their function in a ROS 2 system.
We recommend to first complete the following tutorials:

    * :ref:`Modifying Ownership and Ownership Strength QoS Policy <tutorials_qos_ownership_ownership>`

    * `Writing a simple publisher and subscriber (C++) <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>`_

    * `Understanding parameters <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html>`_

    * `Using parameters in a class (C++) <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html>`_

    * `Monitoring for parameter changes (C++) <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html>`_

Create a package
----------------

First, open a new terminal and source your ROS 2 installation so that ros2 commands will work.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash


We will create a package in a new workspace folder named `ros2_ws` in `~/`.

.. code-block:: bash

    mkdir ~/ros2_ws && mkdir ~/ros2_ws/src && cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake vulcanexus_change_mutable_qos --dependencies rclcpp


Make sure to add the description, maintainer email and name, and license information to the `package.xml` file of the created package, as explained in the aforementioned tutorials.

Write sources
-------------

First we will create the source file of the first publisher, the Publisher 1.
Inside the `ros2_ws/src/cpp_parameter_event_handler/src` directory, create a new file called `change_mutable_qos_pub1.cpp` and paste the following code within:

.. code-block:: c++

    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    #include "rmw_fastrtps_cpp/get_participant.hpp"
    #include "rmw_fastrtps_cpp/get_publisher.hpp"

    #include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

    using namespace std::chrono_literals;

    class Node_ChangeMutableQoS_Pub1 : public rclcpp::Node
    {
    public:
    Node_ChangeMutableQoS_Pub1()
    : Node("node1_change_mutable_qos")
    {
        // Chatter publisher callback
        auto publish =
        [this]() -> void
        {
            msg_ = std::make_unique<std_msgs::msg::String>();
            msg_->data = "Hello World: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "PUB1 Publishing: '%s'", msg_->data.c_str());
            pub_->publish(std::move(msg_));

            eprosima::fastdds::dds::DataWriterQos dw_qos;
            dw->get_qos(dw_qos);

            eprosima::fastdds::dds::OwnershipStrengthQosPolicy dw_os_qos;
            dw_os_qos = dw_qos.ownership_strength();


            RCLCPP_INFO(this->get_logger(), "Ownership Strength: '%d'", dw_os_qos.value);
        };
        // Chatter publisher timer
        timer_ = create_wall_timer(500ms, publish);
        // Chatter publisher creation
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

        // Access RMW and Fast DDS inner object handles
        rcl_pub = pub_->get_publisher_handle().get();
        rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
        dw = rmw_fastrtps_cpp::get_datawriter(rmw_pub);

        // Declare ROS parameter
        this->declare_parameter("pub1_ownership_strength", 100); // This is the parameter initialization. 100 is only to state it is int type

        // Create a parameter subscriber that can be used to monitor parameter changes
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Set a callback for this node's integer parameter, "pub1_ownership_strength"
        auto cb = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(
            this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int());

            eprosima::fastdds::dds::DataWriterQos dw_qos;
            dw->get_qos(dw_qos);

            eprosima::fastdds::dds::OwnershipStrengthQosPolicy dw_os_qos;
            dw_os_qos = dw_qos.ownership_strength();
            dw_os_qos.value = p.as_int();
            dw_qos.ownership_strength(dw_os_qos);

            dw->set_qos(dw_qos);
        };
        cb_handle_ = param_subscriber_->add_parameter_callback("pub1_ownership_strength", cb);
    }

    private:
        size_t count_ = 1;
        std::unique_ptr<std_msgs::msg::String> msg_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Pointers to RMW and Fast DDS inner object handles
        rcl_publisher_t * rcl_pub;
        rmw_publisher_t * rmw_pub;
        eprosima::fastdds::dds::DataWriter * dw;
    };

    int main(int argc, char ** argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Node_ChangeMutableQoS_Pub1>());
        rclcpp::shutdown();

        return 0;
    }


The code for the Publisher 2 is the same, just changing the Publisher 1 names for Publisher 2 names.
Inside the `ros2_ws/src/cpp_parameter_event_handler/src` directory, create a new file called `change_mutable_qos_pub2.cpp` and paste the following code within:

.. code-block:: c++

    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    #include "rmw_fastrtps_cpp/get_participant.hpp"
    #include "rmw_fastrtps_cpp/get_publisher.hpp"

    #include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

    using namespace std::chrono_literals;

    class Node_ChangeMutableQoS_Pub1 : public rclcpp::Node
    {
    public:
    Node_ChangeMutableQoS_Pub1()
    : Node("node2_change_mutable_qos")
    {
        // Chatter publisher callback
        auto publish =
        [this]() -> void
        {
            msg_ = std::make_unique<std_msgs::msg::String>();
            msg_->data = "Hello World: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "PUB2 Publishing: '%s'", msg_->data.c_str());
            pub_->publish(std::move(msg_));

            eprosima::fastdds::dds::DataWriterQos dw_qos;
            dw->get_qos(dw_qos);

            eprosima::fastdds::dds::OwnershipStrengthQosPolicy dw_os_qos;
            dw_os_qos = dw_qos.ownership_strength();


            RCLCPP_INFO(this->get_logger(), "Ownership strength: '%d'", dw_os_qos.value);
        };
        // Chatter publisher timer
        timer_ = create_wall_timer(500ms, publish);
        // Chatter publisher creation
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

        // Access RMW and Fast DDS inner object handles
        rcl_pub = pub_->get_publisher_handle().get();
        rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
        dw = rmw_fastrtps_cpp::get_datawriter(rmw_pub);

        // Declare ROS parameter
        this->declare_parameter("pub2_ownership_strength", 1); // This is the parameter initialization. 100 is only to state it is int type

        // Create a parameter subscriber that can be used to monitor parameter changes
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Set a callback for this node's integer parameter, "pub2_ownership_strength"
        auto cb = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(
            this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int());

            eprosima::fastdds::dds::DataWriterQos dw_qos;
            dw->get_qos(dw_qos);

            eprosima::fastdds::dds::OwnershipStrengthQosPolicy dw_os_qos;
            dw_os_qos = dw_qos.ownership_strength();
            dw_os_qos.value = p.as_int();
            dw_qos.ownership_strength(dw_os_qos);

            dw->set_qos(dw_qos);
        };
        cb_handle_ = param_subscriber_->add_parameter_callback("pub2_ownership_strength", cb);
    }

    private:
        size_t count_ = 1;
        std::unique_ptr<std_msgs::msg::String> msg_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Pointers to RMW and Fast DDS inner object handles
        rcl_publisher_t * rcl_pub;
        rmw_publisher_t * rmw_pub;
        eprosima::fastdds::dds::DataWriter * dw;
    };

    int main(int argc, char ** argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Node_ChangeMutableQoS_Pub1>());
        rclcpp::shutdown();

        return 0;
    }


The case of the subscriber is easier, as we only need a minimal subscriber for this tutorial.
Inside the `ros2_ws/src/cpp_parameter_event_handler/src` directory, create a new file called `change_mutable_qos_sub.cpp` and paste the following code within:

.. code-block:: c++

    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    using std::placeholders::_1;

    class Node_ChangeMutableQoS_Sub : public rclcpp::Node
    {
    public:
        Node_ChangeMutableQoS_Sub()
        : Node("minimal_subscriber")
        {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&Node_ChangeMutableQoS_Sub::topic_callback, this, _1));
        }

    private:
        void topic_callback(const std_msgs::msg::String & msg) const
        {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Node_ChangeMutableQoS_Sub>());
    rclcpp::shutdown();
    return 0;
    }


Explaining the source code
--------------------------

In the case of the Publishers, the code is analogous, so here the code is going to be explained just for Publisher 1.
For the case of the Subscriber, this tutorial is not going to explain it, as it is just the minimal subscriber, listening on the topic `/chatter`, already explained in the `Writing a simple publisher and subscriber (C++) <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>`_ tutorial.

For the Publisher, here not all the code is going to be explained, as the referred tutorials of the prerequisites section explain big part of it.
For instance, the `/chatter` temporized publisher is explained in the `Writing a simple publisher and subscriber (C++) <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>`_

.. code-block:: c++

    // Chatter publisher callback
        auto publish =
        [this]() -> void
        {
            msg_ = std::make_unique<std_msgs::msg::String>();
            msg_->data = "Hello World: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "PUB1 Publishing: '%s'", msg_->data.c_str());
            pub_->publish(std::move(msg_));

            eprosima::fastdds::dds::DataWriterQos dw_qos;
            dw->get_qos(dw_qos);

            eprosima::fastdds::dds::OwnershipStrengthQosPolicy dw_os_qos;
            dw_os_qos = dw_qos.ownership_strength();


            RCLCPP_INFO(this->get_logger(), "Ownership Strength: '%d'", dw_os_qos.value);
        };
        // Chatter publisher timer
        timer_ = create_wall_timer(500ms, publish);
        // Chatter publisher creation
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);


, and the mechanism to respond by means of a user callback to a change in a node's parameter is explained in `Monitoring for parameter changes (C++) <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html>`_.

.. code-block:: c++

    // Declare ROS parameter
        this->declare_parameter("pub1_ownership_strength", 100); // This is the parameter initialization. 100 is only to state it is int type

        // Create a parameter subscriber that can be used to monitor parameter changes
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Set a callback for this node's integer parameter, "pub1_ownership_strength"
        auto cb = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(
            this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int());

            eprosima::fastdds::dds::DataWriterQos dw_qos;
            dw->get_qos(dw_qos);

            eprosima::fastdds::dds::OwnershipStrengthQosPolicy dw_os_qos;
            dw_os_qos = dw_qos.ownership_strength();
            dw_os_qos.value = p.as_int();
            dw_qos.ownership_strength(dw_os_qos);

            dw->set_qos(dw_qos);
        };
        cb_handle_ = param_subscriber_->add_parameter_callback("pub1_ownership_strength", cb);


The `demo_nodes_cpp_native <https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native>`_ shows how to access inner RMW and Fast DDS entities, although it is not actually explained.
In this tutorial, that same mechanism is used.
In the private section of the `Node_ChangeMutableQoS_PubX` class, the pointers to the native handlers are declared:

.. code-block:: c++

    // Pointers to RMW and Fast DDS inner object handles
    rcl_publisher_t * rcl_pub;
    rmw_publisher_t * rmw_pub;
    eprosima::fastdds::dds::DataWriter * dw;


In the constructor, the pointers are populated by calling the APIs provided by the rmw and rmw_fastrtps_cpp, until obtaining the `eprosima::fastdds::dds::DataWriter` handle:

.. code-block:: c++

    // Access RMW and Fast DDS inner object handles
    rcl_pub = pub_->get_publisher_handle().get();
    rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
    dw = rmw_fastrtps_cpp::get_datawriter(rmw_pub);


When the `pubX_ownership_strength` is updated (for instance, via command line using `ros2 param set` command), the `cb` parameter callback is raised, and the `eprosima::fastdds::dds::DataWriter` handle is used to update its ownership strength.

.. code-block:: c++

    eprosima::fastdds::dds::DataWriterQos dw_qos;
    dw->get_qos(dw_qos);

    eprosima::fastdds::dds::OwnershipStrengthQosPolicy dw_os_qos;
    dw_os_qos = dw_qos.ownership_strength();
    dw_os_qos.value = p.as_int();
    dw_qos.ownership_strength(dw_os_qos);

    dw->set_qos(dw_qos);

In this case, as in the current version of Fast DDS the builtin statistics are enabled by default (see `DomainParticipantQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#domainparticipantqos>`_), it is needed to retrieve the internal QoS by means of `::get_qos()`, then perform the modifications and update the QoS by means of `::set_qos()`:
The value of the ownership strength is set from the value of the updated parameter.

Update CMakeLists.txt and package.xml
-------------------------------------

We need to add the instructions to compile the new source files, and to account for its dependencies both in CMakeLists.txt and package.xml files.

Make sure that the find_package lines in the CMakeLists.txt are the following, so substitute what you have for the following lines:

.. code-block:: cmake

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rmw_fastrtps_cpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(fastrtps REQUIRED)


Then add the following lines to compile and install each node:

.. code-block:: cmake

    add_executable(change_mutable_qos_pub1 src/change_mutable_qos_pub1.cpp)
    ament_target_dependencies(change_mutable_qos_pub1 rclcpp rmw rmw_fastrtps_cpp std_msgs fastrtps)

    install(TARGETS
    change_mutable_qos_pub1
    DESTINATION lib/${PROJECT_NAME}
    )

    add_executable(change_mutable_qos_pub2 src/change_mutable_qos_pub2.cpp)
    ament_target_dependencies(change_mutable_qos_pub2 rclcpp rmw rmw_fastrtps_cpp std_msgs fastrtps)

    install(TARGETS
    change_mutable_qos_pub2
    DESTINATION lib/${PROJECT_NAME}
    )

    add_executable(change_mutable_qos_sub src/change_mutable_qos_sub.cpp)
    ament_target_dependencies(change_mutable_qos_sub rclcpp rmw rmw_fastrtps_cpp std_msgs)

    install(TARGETS
    change_mutable_qos_sub
    DESTINATION lib/${PROJECT_NAME}
    )


Inside package.xml file, make sure that the <depend> tags, are the following, so substitute what you have for the following lines:

.. code-block:: xml

    <depend>rclcpp</depend>
    <depend>rmw_fastrtps_cpp</depend>
    <depend>fastrtps</depend>
    <depend>std_msgs</depend>


Configure initial QoS
---------------------

Ownership Strength Policy is mutable, but Ownership Policy is not. Then, we need to configure EXCLUSIVE_OWNERSHIP_POLICY to all participants before running the ROS nodes.
To do that, create a new xml file in the root of the workspace:

.. code-block:: bash

    cd ~/ros2_ws
    touch profiles1.xml

Open the newly created file with your preferred editor and paste the following xml code:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <data_writer profile_name="/chatter">
            <qos>
                <ownership>
                    <kind>EXCLUSIVE</kind>
                </ownership>
                <ownershipStrength>
                    <value>10</value>
                </ownershipStrength>
            </qos>
        </data_writer>

        <data_reader profile_name="/chatter">
            <qos>
                <ownership>
                    <kind>EXCLUSIVE</kind>
                </ownership>
            </qos>
        </data_reader>
    </profiles>


This xml includes one profile for a publisher (data writer) and one profile for a subscriber (data reader), and sets them to exclusive ownership, and ownership strength of value 10 for the publisher.
This will be applied to the Publisher 1 and to the Subscriber.
We need another profile in a separate file to assign a different ownership strength to the Publisher 2:

.. code-block:: bash

    touch profiles2.xml


.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <data_writer profile_name="/chatter">
            <qos>
                <ownership>
                    <kind>EXCLUSIVE</kind>
                </ownership>
                <ownershipStrength>
                    <value>2</value>
                </ownershipStrength>
            </qos>
        </data_writer>


This will assign an ownership strength of value 2 to the Publisher 2.

Build
-----

Now we are ready to build the package.
Change your directory to the workspace folder and build using colcon:

.. code-block:: bash

    cd ~/ros2_ws
    colcon build


Run
---

Open three terminals in the workspace folder.
On each you need to source Vulcanexus installation, as well as the package installation.
Then, export the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to point out to the corresponding profiles file.
And run the ROS 2 node.

First, in the first terminal, run the subscriber node, configured with the profiles1.xml file:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    . install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=./profiles1.xml
    ros2 run vulcanexus_change_mutable_qos change_mutable_qos_sub


Then, in another terminal, run the first publisher, configured also with the profiles1.xml file.
This Publisher will then be configured with ownership strength value of 10.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    . install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=./profiles1.xml
    ros2 run vulcanexus_change_mutable_qos change_mutable_qos_pub1


At this point you will be able to see that both nodes are communicating, and the messages from Publisher 1 can be seen in the Subscriber.

In the third terminal, run the second publisher, configured with the profiles2.xml file.
This Publisher will then be configured with ownership strength value of 2.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    . install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=./profiles2.xml
    ros2 run vulcanexus_change_mutable_qos change_mutable_qos_pub2


This Publisher 2 starts sending messages (you could see that the number of the message starts from 1 while the messages from Publisher 1 are already in a higher number), and the Subscriber is still receiving messages from Publisher 1 and not from Publisher 2.
This is because of the exclusive ownership.
Publisher 1 has higher ownership strength than Publisher 2.

Change mutable QoS via command line
-----------------------------------

Here we are going to use the ROS command `param set` to change the value of the node's parameter we have created earlier.
The parameter change will cause the parameter-change callback to be called, and then resulting in a change in the ownership strength.
In another terminal, try the following code:

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash
    ros2 param set /node2_change_mutable_qos pub2_ownership_strength 50


With that execution, we have changed the ownership strength of the Publisher 2 to become bigger than that of the Publisher 1.
You now should be watching the Subscriber receiving the messages from the Publisher 2 and not from the Publisher 1.
