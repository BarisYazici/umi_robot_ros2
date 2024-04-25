#pragma once

#include "umi_robot_msgs/msg/joint_states.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>

class ServoControlClient : public rclcpp::Node
{
public:
    ServoControlClient(const rclcpp::NodeOptions& options) : rclcpp::Node("servo_control_client", options)
    {
        publisher_ = this->create_publisher<umi_robot_msgs::msg::JointStates>("/set_servo_angle", 1000);
    }

    void send_command(const std::vector<double>& angles)
    {
        auto message = umi_robot_msgs::msg::JointStates();

        message.motor_1 = angles[0];
        message.motor_2 = angles[1];
        message.motor_3 = angles[2];
        message.motor_4 = angles[3];

        publisher_->publish(message);
    }

private:
    rclcpp::Publisher<umi_robot_msgs::msg::JointStates>::SharedPtr publisher_;
};