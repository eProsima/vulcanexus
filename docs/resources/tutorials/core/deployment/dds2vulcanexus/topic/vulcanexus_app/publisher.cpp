// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#include <memory>
#include <string>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dds2vulcanexus/idl/hello_world.hpp"

using namespace std::chrono_literals;

class HelloWorldPublisher : public rclcpp::Node
{
public:
  HelloWorldPublisher()
  : Node("helloworld_publisher")
  {
    sample_.index = 0;
    sample_.message = "Hello from Vulcanexus";

    publisher_ = this->create_publisher<dds2vulcanexus::idl::HelloWorld>("HelloWorld", 10);

    auto timer_callback =
      [this]() -> void {
        sample_.index++;
        RCLCPP_INFO(
          this->get_logger(), "Publishing: '%s %u'",
          sample_.message.c_str(), sample_.index);
        this->publisher_->publish(sample_);
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<dds2vulcanexus::idl::HelloWorld>::SharedPtr publisher_;
  dds2vulcanexus::idl::HelloWorld sample_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloWorldPublisher>());
  rclcpp::shutdown();
  return 0;
}
