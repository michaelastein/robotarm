#include "robotarm_hardware/robotarm_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace robotarm_hardware
{

RobotArmSystem::RobotArmSystem()
: chip_(nullptr),
  software_pwm_frequency_hz_(25.0),
  pwm_deadband_(0.002),
  velocity_deadband_rad_s_(0.003),
  velocity_filter_alpha_(0.25),
  pwm_start_time_(std::chrono::steady_clock::now()),
  serial_fd_(-1),
  serial_device_("/dev/ttyACM0"),
  arduino_counts_initialized_(false),
  coast_time_after_command_s_(0.20)
{
  last_arduino_counts_.fill(0);
}

RobotArmSystem::~RobotArmSystem()
{
  stop_all();
  release_gpio();
  close_arduino_serial();
}

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
  ticks_per_joint_rev_.assign(n, 0.0);

  forward_gpio_.assign(n, 0);
  backward_gpio_.assign(n, 0);
  arduino_channel_.assign(n, 0);

  last_motion_sign_.assign(n, 0.0);
  last_valid_motion_sign_.assign(n, 0.0);
  last_active_command_time_.assign(n, rclcpp::Time(0, 0, RCL_ROS_TIME));

  direction_.assign(n, 1.0);
  max_pwm_.assign(n, 0.08);
  min_pwm_.assign(n, 0.0);
  velocity_to_pwm_gain_.assign(n, 0.0);
  velocity_kp_.assign(n, 0.0);
  min_command_velocity_.assign(n, 0.08);
  max_joint_velocity_.assign(n, 0.15);

  lower_limit_.assign(n, 0.0);
  upper_limit_.assign(n, 0.0);

  for (size_t i = 0; i < n; ++i)
  {
    const auto & name = info_.joints[i].name;

    if (name == "base_joint")
    {
      forward_gpio_[i] = 23;
      backward_gpio_[i] = 22;
      arduino_channel_[i] = 0;

      ticks_per_joint_rev_[i] = 595.0;

      direction_[i] = 1.0;
      max_pwm_[i] = 0.08;

      min_pwm_[i] = 0.010;
      velocity_to_pwm_gain_[i] = 0.006;
      velocity_kp_[i] = 0.002;

      min_command_velocity_[i] = 0.08;
      max_joint_velocity_[i] = 0.20;

      lower_limit_[i] = -3.14159265;
      upper_limit_[i] = 3.14159265;
    }
    else if (name == "shoulder_joint")
    {
      forward_gpio_[i] = 25;
      backward_gpio_[i] = 24;
      arduino_channel_[i] = 1;

      ticks_per_joint_rev_[i] = 3000.0;

      direction_[i] = 1.0;
      max_pwm_[i] = 0.08;

      min_pwm_[i] = 0.015;
      velocity_to_pwm_gain_[i] = 0.001;
      velocity_kp_[i] = 0.001;

      min_command_velocity_[i] = 0.08;
      max_joint_velocity_[i] = 0.20;

      lower_limit_[i] = -0.52359878;
      upper_limit_[i] = 1.39626340;
    }
    else if (name == "elbow_joint")
    {
      forward_gpio_[i] = 13;
      backward_gpio_[i] = 12;
      arduino_channel_[i] = 2;

      ticks_per_joint_rev_[i] = 2400.0;

      direction_[i] = 1.0;
      max_pwm_[i] = 0.08;

      min_pwm_[i] = 0.05;
      velocity_to_pwm_gain_[i] = 0.060;
      velocity_kp_[i] = 0.006;

      min_command_velocity_[i] = 0.08;
      max_joint_velocity_[i] = 0.30;

      lower_limit_[i] = -0.69813170;
      upper_limit_[i] = 2.44346095;
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmSystem"),
        "Unknown joint name in hardware config: %s",
        name.c_str()
      );

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmSystem"),
    "URDF hardware joint count = %zu",
    n
  );

  for (size_t i = 0; i < n; ++i)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RobotArmSystem"),
      "Joint %zu: name=%s fwd=%d bwd=%d arduino_channel=%d "
      "ticks_per_rev=%.1f dir=%.1f min_pwm=%.3f max_pwm=%.3f "
      "vel_to_pwm=%.3f vel_kp=%.3f min_cmd_vel=%.3f max_vel=%.3f",
      i,
      info_.joints[i].name.c_str(),
      forward_gpio_[i],
      backward_gpio_[i],
      arduino_channel_[i],
      ticks_per_joint_rev_[i],
      direction_[i],
      min_pwm_[i],
      max_pwm_[i],
      velocity_to_pwm_gain_[i],
      velocity_kp_[i],
      min_command_velocity_[i],
      max_joint_velocity_[i]
    );
  }

  if (!open_arduino_serial())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmSystem"),
      "Could not open Arduino serial device: %s",
      serial_device_.c_str()
    );

    return hardware_interface::CallbackReturn::ERROR;
  }

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
        "Could not get GPIO motor lines for joint %zu",
        i
      );

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (gpiod_line_request_output(fwd, "robotarm_hardware", 0) < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmSystem"),
        "Could not request forward GPIO output for joint %zu",
        i
      );

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (gpiod_line_request_output(bwd, "robotarm_hardware", 0) < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmSystem"),
        "Could not request backward GPIO output for joint %zu",
        i
      );

      return hardware_interface::CallbackReturn::ERROR;
    }

    gpiod_line_set_value(fwd, 0);
    gpiod_line_set_value(bwd, 0);

    forward_lines_.push_back(fwd);
    backward_lines_.push_back(bwd);

    RCLCPP_INFO(
      rclcpp::get_logger("RobotArmSystem"),
      "GPIO ready for %s: fwd=%d bwd=%d",
      info_.joints[i].name.c_str(),
      forward_gpio_[i],
      backward_gpio_[i]
    );
  }

  pwm_start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmSystem"),
    "RobotArmSystem initialized without debug logs"
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

bool RobotArmSystem::open_arduino_serial()
{
  serial_fd_ = open(
    serial_device_.c_str(),
    O_RDONLY | O_NOCTTY | O_NONBLOCK
  );

  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmSystem"),
      "open(%s) failed: %s",
      serial_device_.c_str(),
      std::strerror(errno)
    );

    return false;
  }

  termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmSystem"),
      "tcgetattr failed: %s",
      std::strerror(errno)
    );

    close_arduino_serial();
    return false;
  }

  cfmakeraw(&tty);

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmSystem"),
      "tcsetattr failed: %s",
      std::strerror(errno)
    );

    close_arduino_serial();
    return false;
  }

  tcflush(serial_fd_, TCIFLUSH);

  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmSystem"),
    "Opened Arduino serial device %s at 115200 baud",
    serial_device_.c_str()
  );

  return true;
}

void RobotArmSystem::close_arduino_serial()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool RobotArmSystem::parse_arduino_line(
  const std::string & line,
  std::array<long, 4> & counts)
{
  std::stringstream ss(line);
  std::string item;
  std::array<long, 4> parsed;
  parsed.fill(0);

  for (size_t i = 0; i < 4; ++i)
  {
    if (!std::getline(ss, item, ','))
    {
      return false;
    }

    try
    {
      parsed[i] = std::stol(item);
    }
    catch (...)
    {
      return false;
    }
  }

  counts = parsed;
  return true;
}

bool RobotArmSystem::read_arduino_counts(std::array<long, 4> & counts)
{
  if (serial_fd_ < 0)
  {
    return false;
  }

  bool got_valid_line = false;
  std::array<long, 4> latest_counts = counts;

  char buffer[256];

  while (true)
  {
    const ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));

    if (n > 0)
    {
      serial_buffer_.append(buffer, static_cast<size_t>(n));

      size_t newline_pos = std::string::npos;

      while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos)
      {
        std::string line = serial_buffer_.substr(0, newline_pos);
        serial_buffer_.erase(0, newline_pos + 1);

        if (!line.empty() && line.back() == '\r')
        {
          line.pop_back();
        }

        std::array<long, 4> parsed_counts;

        if (parse_arduino_line(line, parsed_counts))
        {
          latest_counts = parsed_counts;
          got_valid_line = true;
        }
      }

      if (serial_buffer_.size() > 1024)
      {
        serial_buffer_.clear();
      }

      continue;
    }

    if (n == 0)
    {
      break;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      break;
    }

    RCLCPP_WARN(
      rclcpp::get_logger("RobotArmSystem"),
      "Arduino serial read error: %s",
      std::strerror(errno)
    );

    break;
  }

  if (got_valid_line)
  {
    counts = latest_counts;
  }

  return got_valid_line;
}

hardware_interface::return_type RobotArmSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  std::array<long, 4> current_counts = last_arduino_counts_;

  if (!read_arduino_counts(current_counts))
  {
    return hardware_interface::return_type::OK;
  }

  if (!arduino_counts_initialized_)
  {
    last_arduino_counts_ = current_counts;
    arduino_counts_initialized_ = true;

    RCLCPP_WARN(
      rclcpp::get_logger("RobotArmSystem"),
      "Arduino counts initialized: [%ld, %ld, %ld, %ld]",
      current_counts[0],
      current_counts[1],
      current_counts[2],
      current_counts[3]
    );

    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const int channel = arduino_channel_[i];

    if (channel < 0 || channel >= 4)
    {
      continue;
    }

    long raw_delta_ticks =
      current_counts[channel] - last_arduino_counts_[channel];

    if (raw_delta_ticks < 0)
    {
      raw_delta_ticks = 0;
    }

    double sign_for_ticks = 0.0;

    if (raw_delta_ticks > 0)
    {
      if (last_motion_sign_[i] != 0.0)
      {
        sign_for_ticks = last_motion_sign_[i];
      }
      else
      {
        sign_for_ticks = 0.0;

        if (raw_delta_ticks > 5)
        {
          RCLCPP_WARN(
            rclcpp::get_logger("RobotArmSystem"),
            "Ignoring encoder ticks for %s without active command: raw_delta=%ld",
            info_.joints[i].name.c_str(),
            raw_delta_ticks
          );
        }
      }
    }

    const double signed_delta_ticks =
      sign_for_ticks * static_cast<double>(raw_delta_ticks);

    encoder_ticks_[i] += signed_delta_ticks;

    const double delta_rad =
      signed_delta_ticks /
      ticks_per_joint_rev_[i] *
      2.0 * M_PI;

    position_[i] += delta_rad;

    position_[i] = std::clamp(
      position_[i],
      lower_limit_[i],
      upper_limit_[i]
    );

    double measured_velocity = 0.0;

    if (dt > 0.0)
    {
      measured_velocity = delta_rad / dt;
    }

    velocity_[i] =
      velocity_filter_alpha_ * measured_velocity +
      (1.0 - velocity_filter_alpha_) * velocity_[i];
  }

  last_arduino_counts_ = current_counts;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmSystem::write(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double desired_velocity = command_[i];

    desired_velocity = std::clamp(
      desired_velocity,
      -max_joint_velocity_[i],
      max_joint_velocity_[i]
    );

    if (position_[i] <= lower_limit_[i] && desired_velocity < 0.0)
    {
      desired_velocity = 0.0;
    }

    if (position_[i] >= upper_limit_[i] && desired_velocity > 0.0)
    {
      desired_velocity = 0.0;
    }

    if (std::abs(desired_velocity) <= velocity_deadband_rad_s_)
    {
      last_motion_sign_[i] = 0.0;
      set_motor(i, 0.0);
      continue;
    }

    const double sign = desired_velocity > 0.0 ? 1.0 : -1.0;

    double desired_speed = std::abs(desired_velocity);

    if (desired_speed < min_command_velocity_[i])
    {
      desired_speed = min_command_velocity_[i];
    }

    const double measured_speed_along_direction = velocity_[i] * sign;

    const double velocity_error =
      desired_speed - measured_speed_along_direction;

    double pwm_abs =
      min_pwm_[i] +
      velocity_to_pwm_gain_[i] * desired_speed +
      velocity_kp_[i] * velocity_error;

    if (pwm_abs < 0.0)
    {
      pwm_abs = 0.0;
    }

    pwm_abs = std::clamp(
      pwm_abs,
      0.0,
      max_pwm_[i]
    );

    const double pwm_command = sign * pwm_abs;
    const double physical_pwm_command = pwm_command * direction_[i];

    last_motion_sign_[i] = sign;
    last_valid_motion_sign_[i] = sign;
    last_active_command_time_[i] = time;

    set_motor(i, physical_pwm_command);
  }

  return hardware_interface::return_type::OK;
}

void RobotArmSystem::set_motor(size_t i, double pwm_command)
{
  if (
    i >= forward_lines_.size() ||
    i >= backward_lines_.size() ||
    i >= max_pwm_.size() ||
    i >= last_motion_sign_.size())
  {
    return;
  }

  if (std::abs(pwm_command) <= pwm_deadband_)
  {
    gpiod_line_set_value(forward_lines_[i], 0);
    gpiod_line_set_value(backward_lines_[i], 0);

    last_motion_sign_[i] = 0.0;
    return;
  }

  const double clamped_command =
    std::clamp(pwm_command, -max_pwm_[i], max_pwm_[i]);

  const double duty =
    std::clamp(std::abs(clamped_command) / max_pwm_[i], 0.0, 1.0);

  const auto now = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = now - pwm_start_time_;

  const double pwm_period = 1.0 / software_pwm_frequency_hz_;
  const double phase = std::fmod(elapsed.count(), pwm_period);

  const bool pwm_on = phase < duty * pwm_period;

  if (!pwm_on)
  {
    gpiod_line_set_value(forward_lines_[i], 0);
    gpiod_line_set_value(backward_lines_[i], 0);

    return;
  }

  if (clamped_command > 0.0)
  {
    gpiod_line_set_value(forward_lines_[i], 1);
    gpiod_line_set_value(backward_lines_[i], 0);
  }
  else
  {
    gpiod_line_set_value(forward_lines_[i], 0);
    gpiod_line_set_value(backward_lines_[i], 1);
  }
}

void RobotArmSystem::stop_all()
{
  for (size_t i = 0; i < forward_lines_.size(); ++i)
  {
    if (i < backward_lines_.size())
    {
      gpiod_line_set_value(forward_lines_[i], 0);
      gpiod_line_set_value(backward_lines_[i], 0);
    }
  }
}

void RobotArmSystem::release_gpio()
{
  for (auto * line : forward_lines_)
  {
    if (line)
    {
      gpiod_line_release(line);
    }
  }

  for (auto * line : backward_lines_)
  {
    if (line)
    {
      gpiod_line_release(line);
    }
  }

  forward_lines_.clear();
  backward_lines_.clear();

  if (chip_)
  {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
}

}  // namespace robotarm_hardware

PLUGINLIB_EXPORT_CLASS(
  robotarm_hardware::RobotArmSystem,
  hardware_interface::SystemInterface
)
