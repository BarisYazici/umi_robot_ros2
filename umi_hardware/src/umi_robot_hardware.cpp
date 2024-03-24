#include "umi_hardware/umi_robot_hardware.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace umi_hardware{

hardware_interface::CallbackReturn UmiRobotPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (uint i = 0; i < servo_positions_.size(); i++)
  {
    servo_positions_[i] = 0;
    servo_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("UmiRobotPositionOnlyHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn UmiRobotPositionOnlyHardware::on_init(const hardware_interface::HardwareInfo& info)
{

  RCLCPP_INFO(rclcpp::get_logger("UmiRobotPositionOnlyHardware"), "Initializing ...please wait...");
  info_ = info;


  servo_control_client_ = std::make_shared<ServoControlClient>(rclcpp::NodeOptions());

  executor_ = std::make_shared<UmiExecutor>();
  executor_->add_node(servo_control_client_);


  RCLCPP_INFO(rclcpp::get_logger("UmiRobotPositionOnlyHardware"), "Successfully initialized!");

  servo_positions_.resize(info_.joints.size());
  servo_commands_.resize(info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
UmiRobotPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &servo_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
UmiRobotPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &servo_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn UmiRobotPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("UmiRobotPositionOnlyHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UmiRobotPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UmiRobotPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  servo_positions_ = servo_commands_; 

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UmiRobotPositionOnlyHardware::write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period)
{
    std::vector<double> servo_commands_in_degree(4);
    for (int i = 0; i < 4; i++)
    {
        servo_commands_in_degree[i] = servo_commands_[i] * 180 / M_PI;
    }
    servo_control_client_->send_command(servo_commands_in_degree);
    return hardware_interface::return_type::OK;
}
}  // namespace umi_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(umi_hardware::UmiRobotPositionOnlyHardware,
                       hardware_interface::SystemInterface)
