.. _tutorials_ros2_introduction:

.. figure:: /rst/figures/tutorials/ros/biglogo.png
    :width: 500px
    :align: center

ROS2 PubSub example
===================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: none

Overview
--------

Writing a simple publisher and subscriber build it using Colcon.

Nodes are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.


Create a workspace
--------------------

A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

    .. code-block:: bash

        source /opt/ros/humble/setup.bash

        # Best practice is to create a new directory for every new workspace.
        mkdir -p ~/dev_ws/src
        # Another best practice is to put any packages in your workspace into the src directory.
        cd ~/dev_ws/src

Create a package
----------------

Recall that packages should be created in the src directory, not the root of the workspace. So, navigate into dev_ws/src, and run the package creation command:

    .. code-block:: bash

        ros2 pkg create --build-type ament_cmake cpp_pubsub

        # Recall that this is the directory in any CMake package where the source files containing executables belong
        cd cpp_pubsub/src
    
    .. code-block:: text

        cpp_pubsub/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include
        │   └── cpp_pubsub
        └── src

Create publisher_member_function.cpp
------------------------------------

    .. code-block:: bash

        # on ~/dev_ws/srccpp_pubsub/src
        touch publisher_member_function.cpp

    .. code-block:: text

        cpp_pubsub/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include
        │   └── cpp_pubsub
        └── src
            └── publisher_member_function.cpp

    .. code-block:: c++

        // node dependencies.
        #include <chrono>
        #include <functional>
        #include <memory>
        #include <string>

        // allows you to use the most common pieces of the ROS 2 system.
        #include "rclcpp/rclcpp.hpp"
        // includes the built-in message type you will use to publish data.
        #include "std_msgs/msg/string.hpp"

        using namespace std::chrono_literals;

        /* This example creates a subclass of Node and uses std::bind() to register a
        * member function as a callback from the timer. */

        // creates the node class MinimalPublisher by inheriting from rclcpp::Node. 
        // Every this in the code is referring to the node.
        class MinimalPublisher : public rclcpp::Node
        {
        public:

            MinimalPublisher()
                // initialize Node with node name.
                : Node("minimal_publisher")
                , count_(0)
            {
                // publisher of type std_msgs::msg::String on topic called topic and 
                // queue size to limit messages in the event of a backup.
                publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

                // timer_callback function will be executed twice a second.
                timer_ = this->create_wall_timer(
                    500ms, std::bind(&MinimalPublisher::timer_callback, this));
            }

        private:

            // The timer_callback function is where the message data is set and the messages are actually published.
            void timer_callback()
            {
                auto message = std_msgs::msg::String();
                message.data = "Hello, world! " + std::to_string(count_++);
                // The RCLCPP_INFO macro ensures every published message is printed to the console.
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                publisher_->publish(message);
            }

            // private attributes.
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
            size_t count_;
        };

        int main(
                int argc,
                char* argv[])
        {
            // initializes ROS 2.
            rclcpp::init(argc, argv);
            // starts processing data from the node, including callbacks from the timer.
            rclcpp::spin(std::make_shared<MinimalPublisher>());
            rclcpp::shutdown();
            return 0;
        }

Create subscriber_member_function.cpp
-------------------------------------

    .. code-block:: bash

        # on ~/dev_ws/srccpp_pubsub/src
        touch publisher_member_function.cpp

    .. code-block:: text

        cpp_pubsub/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include
        │   └── cpp_pubsub
        └── src
            ├── subscriber_member_function.cpp
            └── publisher_member_function.cpp

    topic name and message type used by the publisher and subscriber must match to allow them to communicate.

    .. code-block:: c++

        #include <memory>

        // allows you to use the most common pieces of the ROS 2 system.
        #include "rclcpp/rclcpp.hpp"
        // includes the built-in message type you will use to publish data.
        #include "std_msgs/msg/string.hpp"

        using std::placeholders::_1;

        class MinimalSubscriber : public rclcpp::Node
        {
        public:

            MinimalSubscriber()
                // initialize Node with node name.
                : Node("minimal_subscriber")
            {
                // subscriber of type std_msgs::msg::String on topic called topic and 
                // queue size to limit messages in the event of a backup and bind
                // topic_callback to be called on data.
                subscription_ = this->create_subscription<std_msgs::msg::String>(
                    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
            }

        private:
            // function receives the string message data published over the topic, 
            // and simply writes it to the console using the RCLCPP_INFO macro.
            void topic_callback(
                    const std_msgs::msg::String::SharedPtr msg) const
            {
                RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            }

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        };

        int main(
                int argc,
                char* argv[])
        {
            // initializes ROS 2.
            rclcpp::init(argc, argv);
            // preparing to receive messages whenever they come.
            rclcpp::spin(std::make_shared<MinimalSubscriber>());
            rclcpp::shutdown();
            return 0;
        }

Add dependencies
----------------

Open `package.xml` on `dev_ws/src/cpp_pubsub` directory

make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags:

    .. code-block:: xml

        <description>Examples of minimal publisher/subscriber using rclcpp</description>
        <maintainer email="you@email.com">Your Name</maintainer>
        <license>Apache License 2.0</license>

Add a new line after the `ament_cmake buildtool` dependency and paste the following dependencies corresponding to your node’s include statements:

    .. code-block:: xml

        <depend>rclcpp</depend>
        <depend>std_msgs</depend>

CMakeLists.txt
--------------

Replace `CMakeLists.txt` content with next example:
    
    .. code-block:: text

        cmake_minimum_required(VERSION 3.5)
        project(cpp_pubsub)

        if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
        endif()

        if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
        endif()

        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)

        add_executable(talker src/publisher_member_function.cpp)
        ament_target_dependencies(talker rclcpp std_msgs)

        add_executable(listener src/subscriber_member_function.cpp)
        ament_target_dependencies(listener rclcpp std_msgs)

        install(TARGETS
                talker
                listener
                DESTINATION lib/${PROJECT_NAME})

        ament_package()

Build
-----

You likely already have the rclcpp and std_msgs packages installed as part of your ROS 2 system. It’s good practice to run rosdep in the root of your workspace (dev_ws) to check for missing dependencies before building:

    .. code-block:: bash

        # ~/dev_ws
        rosdep install -i --from-path src --rosdistro humble -y

Still in the root of your workspace, dev_ws, build your new package:

    .. code-block:: bash

        # ~/dev_ws
        colcon build --packages-select cpp_pubsub


Run
-----

source setup.bash on your workspace
    
    .. code-block:: bash

        # ~/dev_ws
        . install/setup.bash

Run the talker in one terminal:

    .. code-block:: bash

        # ~/dev_ws
        ros2 run cpp_pubsub talker


Then run listener in another terminal:

    .. code-block:: bash

        # ~/dev_ws
        ros2 run cpp_pubsub listener

The listener will start printing messages to the console.