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

        // Get and modify ownership strength
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
