#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "umi_hardware/set_control_client.hpp"
#include "umi_hardware/umi_executor.hpp"

namespace umi_hardware
{
class UmiRobotPositionOnlyHardware: public hardware_interface::SystemInterface
{
public:
    // UmiHardwareInterface();
  RCLCPP_SHARED_PTR_DEFINITIONS(UmiRobotPositionOnlyHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<ServoControlClient> servo_control_client_;
    
    std::vector<double> servo_positions_;
    std::vector<double> servo_commands_;
    std::shared_ptr<UmiExecutor> executor_;

};
}  // namespace umi_hardware