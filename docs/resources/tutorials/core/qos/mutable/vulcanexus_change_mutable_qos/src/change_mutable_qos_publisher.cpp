// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file change_mutable_qos_publisher.cpp
 */
 
 #include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_publisher.hpp"

#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

using namespace std::chrono_literals;

class Node_ChangeMutableQoS_Publisher : public rclcpp::Node
{
public:
  Node_ChangeMutableQoS_Publisher(std::string node_name_prefix, std::string node_name)
  : Node(node_name),
    node_name_(node_name)
  {
    // Chatter publisher callback
    auto publish =
      [this]() -> void
      {
        message_ = std::make_unique<std_msgs::msg::String>();
        message_->data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "'%s' Publishing: '%s'", node_name_.c_str(), message_->data.c_str());
        publisher_->publish(std::move(message_));

        RCLCPP_INFO(this->get_logger(), "Ownership strength: '%d'", data_writer_->get_qos().ownership_strength().value);
      };
    // Chatter publisher timer
    timer_ = create_wall_timer(500ms, publish);
    // Chatter publisher creation
    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    // Access RMW and Fast DDS inner object handles
    rcl_publisher_ = publisher_->get_publisher_handle().get();
    rmw_publisher_ = rcl_publisher_get_rmw_handle(rcl_publisher_);
    data_writer_ = rmw_fastrtps_cpp::get_datawriter(rmw_publisher_);

    // Declare parameter
    std::string parameter_name = node_name_prefix + "_ownership_strength";
    this->declare_parameter(parameter_name, 100); // This is the parameter initialization. 100 is only to state it is int type

    // Create a parameter subscriber that can be used to monitor parameter changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Set a callback for this node's integer parameter, "Publisher_X_ownership_strength"
    auto callback = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "Callback: Received an update to parameter \"%s\" of type %s: \"%ld\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_int());

        eprosima::fastdds::dds::DataWriterQos dw_qos;
        data_writer_->get_qos(dw_qos);
        
        dw_qos.ownership_strength().value = p.as_int();
        data_writer_->set_qos(dw_qos);
      };
    callback_handle_ = param_subscriber_->add_parameter_callback(parameter_name, callback);
  }

private:
    size_t count_ = 1;
    std::unique_ptr<std_msgs::msg::String> message_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callback_handle_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Pointers to RMW and Fast DDS inner object handles
    rcl_publisher_t * rcl_publisher_;
    rmw_publisher_t * rmw_publisher_;
    eprosima::fastdds::dds::DataWriter * data_writer_;

    std::string node_name_;
};

int main(int argc, char ** argv)
{
    std::string node_name_prefix;
    if (argc == 2)
    {
        node_name_prefix = *(argv + 1);
    }
    else node_name_prefix = "Publisher_X";
    std::string node_name = node_name_prefix + "_change_mutable_qos";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Node_ChangeMutableQoS_Publisher>(node_name_prefix, node_name));
    rclcpp::shutdown();
    return 0;
}
