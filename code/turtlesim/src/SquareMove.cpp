#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <math.h>

#define PI 3.1415926

class SquareMove : public rclcpp::Node
{
public:

    SquareMove()
        : Node("turtlesim_square_move")
        , state_(State::INIT)
        , first_pose_received_(false)
        , first_goal_set_(false)
    {
        // Create subscription on the turtle1/pose topic to keep track of the turtle pose
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            1, std::bind(&SquareMove::pose_callback_, this, std::placeholders::_1));

        // Create a publisher on the turtle1/cmd_vel topic to publish the movement commands to the turtle
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);

        // Time to check the current state of the turtle
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SquareMove::timer_callback_, this));

        // Create the service client to reset the state of the turtlesim_node
        reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");
    }

private:

    enum State
    {
        INIT,
        FORWARD,
        STOP_FORWARD,
        TURN,
        STOP_TURN,
    };

    void pose_callback_(
            const turtlesim::msg::Pose& pose)
    {
        // Update the turtle pose
        pose_ = pose;
        if (!first_pose_received_)
        {
            first_pose_received_ = true;
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            reset_client_->async_send_request(empty_request);
            RCLCPP_INFO(rclcpp::get_logger(
                        "turtlesim_square_move"), "First pose received [%f %f, %f]", pose_.x, pose_.y, pose_.theta);
        }
    }

    void next_goal_()
    {
        switch (state_)
        {
            case State::FORWARD:
                goal_.x = cos(pose_.theta) * 2 + pose_.x;
                goal_.y = sin(pose_.theta) * 2 + pose_.y;
                goal_.theta = pose_.theta;
                break;
            case State::TURN:
                goal_.x = pose_.x;
                goal_.y = pose_.y;
                goal_.theta = fmod(pose_.theta + static_cast<float>(PI) / 2.0f, 2.0f * static_cast<float>(PI));
                // wrap goal_.theta to [-pi, pi)
                if (goal_.theta >= static_cast<float>(PI))
                {
                    goal_.theta -= 2.0f * static_cast<float>(PI);
                }
                break;
            default:
                break;
        }
    }

    bool reached_goal_()
    {
        return fabsf(pose_.x - goal_.x) < 0.1 &&
               fabsf(pose_.y - goal_.y) < 0.1 &&
               fabsf(pose_.theta - goal_.theta) < 0.01;
    }

    bool stopped_()
    {
        return pose_.angular_velocity < 0.0001 && pose_.linear_velocity < 0.0001;
    }

    void print_goal_()
    {
        RCLCPP_INFO(
            rclcpp::get_logger("turtlesim_square_move"), "New goal [%f, %f, %f]", goal_.x, goal_.y, goal_.theta);
    }

    void command_turtle_(
            float linear,
            float angular)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear;
        twist.angular.z = angular;
        twist_publisher_->publish(twist);
    }

    void init_()
    {
        if (!first_pose_received_)
        {
            return;
        }

        if (!first_goal_set_)
        {
            first_goal_set_ = true;
            state_ = FORWARD;
            next_goal_();
            print_goal_();
        }
    }

    void forward_()
    {
        if (reached_goal_())
        {
            state_ = STOP_FORWARD;
            command_turtle_(0, 0);
        }
        else
        {
            command_turtle_(0.5f, 0);
        }
    }

    void stop_forward_()
    {
        if (stopped_())
        {
            RCLCPP_INFO(rclcpp::get_logger("turtlesim_square_move"), "Reached goal!");
            state_ = TURN;
            next_goal_();
            print_goal_();
        }
        else
        {
            command_turtle_(0, 0);
        }
    }

    void turn_()
    {
        if (reached_goal_())
        {
            state_ = STOP_TURN;
            command_turtle_(0, 0);
        }
        else
        {
            command_turtle_(0, 0.2f);
        }
    }

    void stop_turn_()
    {
        if (stopped_())
        {
            RCLCPP_INFO(rclcpp::get_logger("turtlesim_square_move"), "Reached goal!");
            state_ = FORWARD;
            next_goal_();
            print_goal_();
        }
        else
        {
            command_turtle_(0, 0);
        }
    }

    void timer_callback_()
    {
        switch (state_)
        {
            case State::INIT:
                init_();
                break;
            case State::FORWARD:
                forward_();
                break;
            case State::STOP_FORWARD:
                stop_forward_();
                break;
            case State::TURN:
                turn_();
                break;
            case State::STOP_TURN:
                stop_turn_();
                break;
            default:
                break;
        }
    }

    // Current state of the turtle (moving, turning, stopped_)
    State state_;
    // Check the initial position of the turtle
    bool first_pose_received_;
    // Set the firts goal of the turtle
    bool first_goal_set_;
    // Current pose of the turtle
    turtlesim::msg::Pose pose_;
    // Current goal of the turtle
    turtlesim::msg::Pose goal_;

    // Timer to check the goal and perform some actions
    rclcpp::TimerBase::SharedPtr timer_;
    // Publisher in the Twist topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    // Subscription on the turtle pose topic
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    // Client to reset the state of the turtlesim_node
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
};


int main(
        int argc,
        char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareMove>());
    rclcpp::shutdown();
    return 0;
}
