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