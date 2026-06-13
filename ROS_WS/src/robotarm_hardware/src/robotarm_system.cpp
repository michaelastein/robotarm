#include "robotarm_hardware/robotarm_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

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

  chip_ = gpiod_chip_open_by_name("gpiochip4");

  if (!chip_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmSystem"),
      "Could not open gpiochip4"
    );
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < n; ++i)
  {
    gpiod_line * fwd = gpiod_chip_get_line(chip_, forward_gpio_[i]);
    gpiod_line * bwd = gpiod_chip_get_line(chip_, backward_gpio_[i]);

    if (!fwd || !bwd)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmSystem"),
        "Could not get GPIO lines"
      );
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (
      gpiod_line_request_output(fwd, "robotarm_hardware", 0) < 0 ||
      gpiod_line_request_output(bwd, "robotarm_hardware", 0) < 0
    )
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmSystem"),
        "Could not request GPIO outputs"
      );
      return hardware_interface::CallbackReturn::ERROR;
    }

    forward_lines_.push_back(fwd);
    backward_lines_.push_back(bwd);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmSystem"),
    "RobotArmSystem initialized with GPIO outputs"
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
      &position_[i]
    );

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_[i]
    );
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
      &command_[i]
    );
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
    // Temporary simulation until real encoder reading is added.
    encoder_ticks_[i] +=
      command_[i] *
      dt *
      counts_per_motor_output_rev_ *
      gear_ratio_[i] /
      (2.0 * M_PI);

    const double delta_ticks =
      encoder_ticks_[i] - last_encoder_ticks_[i];

    const double counts_per_joint_rev =
      counts_per_motor_output_rev_ * gear_ratio_[i];

    const double delta_rad =
      direction_[i] *
      delta_ticks /
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
  if (i >= forward_lines_.size() || i >= backward_lines_.size())
  {
    return;
  }

  const double deadband = 0.03;

  if (pwm > deadband)
  {
    gpiod_line_set_value(forward_lines_[i], 1);
    gpiod_line_set_value(backward_lines_[i], 0);
  }
  else if (pwm < -deadband)
  {
    gpiod_line_set_value(forward_lines_[i], 0);
    gpiod_line_set_value(backward_lines_[i], 1);
  }
  else
  {
    gpiod_line_set_value(forward_lines_[i], 0);
    gpiod_line_set_value(backward_lines_[i], 0);
  }
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
