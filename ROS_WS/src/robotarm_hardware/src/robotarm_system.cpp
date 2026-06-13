#include "robotarm_hardware/robotarm_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <algorithm>
#include <iostream>

namespace robotarm_hardware
{

hardware_interface::CallbackReturn RobotArmSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();

  position_.assign(n, 0.0);
  velocity_.assign(n, 0.0);
  command_.assign(n, 0.0);

  encoder_ticks_.assign(n, 0.0);
  last_encoder_ticks_.assign(n, 0.0);

  forward_gpio_ = {23, 25, 13};
  backward_gpio_ = {22, 24, 12};
  encoder_gpio_ = {5, 16, 18};

  gear_ratio_ = {1.0, 1.0, 1.0};
  direction_ = {1.0, 1.0, 1.0};
  max_pwm_ = {0.6, 0.6, 0.6};

  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmSystem"),
    "RobotArmSystem initialized with 3 joints"
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobotArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &command_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type RobotArmSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // TODO: hier später echte Encoder-Ticks lesen.
    // Aktuell Simulations-Fallback:
    encoder_ticks_[i] +=
    	command_[i] * dt *
  	counts_per_motor_output_rev_ *
  	gear_ratio_[i] /
  	(2.0 * M_PI);

    const double delta_ticks =
  	encoder_ticks_[i] - last_encoder_ticks_[i];
    const double counts_per_joint_rev =
      counts_per_motor_output_rev_ * gear_ratio_[i];

    const double delta_rad =
      direction_[i] *
      static_cast<double>(delta_ticks) /
      counts_per_joint_rev *
      2.0 * M_PI;

    position_[i] += delta_rad;

    if (dt > 0.0)
    {
      velocity_[i] = delta_rad / dt;
    }

    last_encoder_ticks_[i] = encoder_ticks_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double pwm = command_[i];

    pwm *= direction_[i];
    pwm = std::clamp(pwm, -max_pwm_[i], max_pwm_[i]);

    set_motor(i, pwm);
  }

  return hardware_interface::return_type::OK;
}

void RobotArmSystem::set_motor(size_t i, double pwm)
{
  // TODO: hier später echte GPIO/PWM setzen.
  //
  // if pwm > 0:
  //   forward_gpio_[i] HIGH/PWM
  //   backward_gpio_[i] LOW
  //
  // if pwm < 0:
  //   forward_gpio_[i] LOW
  //   backward_gpio_[i] HIGH/PWM
  //
  // if pwm == 0:
  //   both LOW

  (void) pwm;
  (void) i;
}

void RobotArmSystem::stop_all()
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    set_motor(i, 0.0);
  }
}

}

PLUGINLIB_EXPORT_CLASS(
  robotarm_hardware::RobotArmSystem,
  hardware_interface::SystemInterface
)
