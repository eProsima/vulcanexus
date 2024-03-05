// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "demo_keys_cpp/msg/sensor_data_msg.hpp"


class Controller : public rclcpp::Node
{
public:

  explicit Controller()
  : Node("controller")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](demo_keys_cpp::msg::SensorDataMsg::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Received %s from sensor id %d", msg->data.c_str() ,(int)msg->sensor_id);
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<demo_keys_cpp::msg::SensorDataMsg>("chatter", 10, callback);
  }

private:
  rclcpp::Subscription<demo_keys_cpp::msg::SensorDataMsg>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
