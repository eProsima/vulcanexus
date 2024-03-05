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

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "demo_keys_cpp/msg/sensor_data_msg.hpp"

using namespace std::chrono_literals;

class RightSensor : public rclcpp::Node
{
public:

  explicit RightSensor()
  : Node("right_sensor")
  {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<demo_keys_cpp::msg::SensorDataMsg>();
        msg_->sensor_id = 0;
        msg_->data = "Data [" + std::to_string(count_) + "] from right sensor";
        RCLCPP_INFO(this->get_logger(), "%s", msg_->data.c_str());
        count_++;
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
      };

    rclcpp::QoS qos(rclcpp::KeepLast{1});
    pub_ = this->create_publisher<demo_keys_cpp::msg::SensorDataMsg>("chatter", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 0;
  std::unique_ptr<demo_keys_cpp::msg::SensorDataMsg> msg_;
  rclcpp::Publisher<demo_keys_cpp::msg::SensorDataMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightSensor>());
  rclcpp::shutdown();
  return 0;
}

