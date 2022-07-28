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
        // Initialize Node with node name.
        : Node("minimal_subscriber")
    {
        // Subscriber of type std_msgs::msg::String on topic called topic and
        // queue size to limit messages in the event of a backup and bind
        // topic_callback to be called on data.
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    // Function receives the string message data published over the topic,
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
    // Initializes ROS 2.
    rclcpp::init(argc, argv);
    // Preparing to receive messages whenever they come.
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}