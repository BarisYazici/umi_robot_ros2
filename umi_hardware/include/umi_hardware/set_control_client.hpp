#pragma once

#include "umi_robot_msgs/srv/control_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

class ServoControlClient : public rclcpp::Node
{
public:
    ServoControlClient(const rclcpp::NodeOptions& options) : rclcpp::Node("servo_control_client", options)
    {
        client_ = create_client<umi_robot_msgs::srv::ControlServo>("/set_servo_angle");
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }
    }

    void send_command(const std::vector<double>& angles)
    {
        auto request = std::make_shared<umi_robot_msgs::srv::ControlServo::Request>();

        request->motor_1 = angles[0];
        request->motor_2 = angles[1];
        request->motor_3 = angles[2];
        request->motor_4 = angles[3];

        auto result_future = client_->async_send_request(request);
        // blocking wait
        // result_future.wait();
    }

private:
    rclcpp::Client<umi_robot_msgs::srv::ControlServo>::SharedPtr client_;
};
