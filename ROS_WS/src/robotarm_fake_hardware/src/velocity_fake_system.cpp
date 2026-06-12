#include "robotarm_fake_hardware/velocity_fake_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robotarm_fake_hardware
{

hardware_interface::CallbackReturn VelocityFakeSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();

  positions_.resize(n, 0.0);
  velocities_.resize(n, 0.0);
  velocity_commands_.resize(n, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
VelocityFakeSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
VelocityFakeSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type VelocityFakeSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  for (size_t i = 0; i < positions_.size(); ++i)
  {
    velocities_[i] = velocity_commands_[i];
    positions_[i] += velocities_[i] * dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VelocityFakeSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(
  robotarm_fake_hardware::VelocityFakeSystem,
  hardware_interface::SystemInterface)
