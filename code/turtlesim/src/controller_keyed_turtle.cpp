// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file controller_keyed_turtle.cpp
 */

#include <stdexcept>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <docs_turtlesim/msg/keyed_twist.hpp>
#include <docs_turtlesim/msg/keyed_pose.hpp>

static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT = 0x44;
static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;
static constexpr char KEYCODE_q = 0x71;
static constexpr char KEYCODE_Q = 0x51;
static constexpr char KEYCODE_1 = 0x31;
static constexpr char KEYCODE_2 = 0x32;
static constexpr char KEYCODE_3 = 0x33;

#define SCALE_ANGULAR 2.0
#define SCALE_LINEAR 2.0


class KeyboardReader
{
public:
  KeyboardReader()
  {
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0)
    {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0)
    {
      throw std::runtime_error("Failed to set new console mode");
    }
  }

  char readOne()
  {
    char c = 0;

    int rc = read(0, &c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }

    return c;
  }

  ~KeyboardReader()
  {
    tcsetattr(0, TCSANOW, &cooked_);
  }

private:
  struct termios cooked_;
};


class ControlKeyedTurtle : public rclcpp::Node
{
public:
  ControlKeyedTurtle()
  : Node("controller_keyed_turtle")
  , turtle_id_(0)
  {
    // Create a publisher for the velocity
    velocity_pub_ = create_publisher<docs_turtlesim::msg::KeyedTwist>(
      "/cmd_vel",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    // Create a subscriber for the pose
    pose_sub_ = create_subscription<docs_turtlesim::msg::KeyedPose>(
      "/pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&ControlKeyedTurtle::callbackPose, this, std::placeholders::_1));

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Select turtle to control with 1, 2 or 3.");
    puts("Use arrow keys to move the turtle.");
    puts("'Q' to quit.");
  }

  int readKey()
  {
    char c;

    // get the next event from the keyboard
    try
    {
      c = input_.readOne();
    }
    catch (const std::runtime_error & e)
    {
      return -1;
    }

    double linear = 0.0;
    double angular = 0.0;

    RCLCPP_DEBUG(get_logger(), "value: 0x%02X\n", c);

    switch(c)
    {
    case KEYCODE_LEFT:
      RCLCPP_INFO(get_logger(), "LEFT");
      angular = 1.0;
      break;
    case KEYCODE_RIGHT:
      RCLCPP_INFO(get_logger(), "RIGHT");
      angular = -1.0;
      break;
    case KEYCODE_UP:
      RCLCPP_INFO(get_logger(), "UP");
      linear = 1.0;
      break;
    case KEYCODE_DOWN:
      RCLCPP_INFO(get_logger(), "DOWN");
      linear = -1.0;
      break;
    case KEYCODE_q:
    case KEYCODE_Q:
      RCLCPP_INFO(get_logger(), "Quit");
      return 0;
    case KEYCODE_1:
      RCLCPP_INFO(get_logger(), "Selecting turtle 1");
      turtle_id_ = 1;
      break;
    case KEYCODE_2:
      RCLCPP_INFO(get_logger(), "Selecting turtle 2");
      turtle_id_ = 2;
      break;
    case KEYCODE_3:
      RCLCPP_INFO(get_logger(), "Selecting turtle 3");
      turtle_id_ = 3;
      break;
    default:
      // This can happen if the read returned when there was no data, or
      // another key was pressed.  In these cases, just silently ignore the
      // press.
      break;
    }

    if (linear != 0.0 || angular != 0.0)
    {
      if (turtle_id_ == 0)
      {
        RCLCPP_INFO(get_logger(), "No turtle selected.");
      }
      else
      {
        docs_turtlesim::msg::KeyedTwist twist;
        twist.turtle_id = turtle_id_;
        twist.angular.z = SCALE_ANGULAR * angular;
        twist.linear.x = SCALE_LINEAR * linear;
        velocity_pub_->publish(twist);
      }
    }

    return 1;
  }

private:

  void callbackPose(const docs_turtlesim::msg::KeyedPose::SharedPtr pose) const
  {
    RCLCPP_INFO(get_logger(), "Turtle [%d] is at x: %f, y: %f, theta: %f",
      pose->turtle_id, pose->x, pose->y, pose->theta);
  }

  rclcpp::Publisher<docs_turtlesim::msg::KeyedTwist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<docs_turtlesim::msg::KeyedPose>::SharedPtr pose_sub_;

  long turtle_id_;
  KeyboardReader input_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto control_node = std::make_shared<ControlKeyedTurtle>();

  while (rclcpp::ok()) {

    int rc = control_node->readKey();
    if (rc != 1) {
      break;
    }

    rclcpp::spin_some(control_node);
  }

  rclcpp::shutdown();

  return 0;
}
