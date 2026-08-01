/**
 * @file robotarm_system.cpp
 *
 * @brief ros2_control hardware interface for a three-joint GPIO-driven arm.
 *
 * This implementation:
 *   - exports position and velocity state interfaces,
 *   - exports velocity command interfaces,
 *   - reads encoder counts from an Arduino over a nonblocking serial port,
 *   - estimates signed joint motion from encoder magnitude and commanded motor
 *     direction,
 *   - converts requested joint velocities into bounded PWM targets,
 *   - enforces software joint limits,
 *   - produces software PWM on paired forward/backward GPIO lines,
 *   - and releases all hardware resources during shutdown.
 *
 * Important configuration values are initialized in RobotArmSystem::on_init()
 * according to each URDF joint name.
 *
 * Serial encoder format:
 *   Each complete line must contain four comma-separated integer counts:
 *
 *     count0,count1,count2,count3
 *
 * Encoder counts are treated as cumulative magnitudes. Since the encoders do
 * not provide direction, the sign is inferred from the active or most recently
 * valid motor-command direction.
 *
 * Velocity-to-PWM control:
 *   pwm = min_pwm
 *       + velocity_to_pwm_gain * desired_speed
 *       + velocity_kp * velocity_error
 *
 * The result is clamped to the configured per-joint maximum PWM value.
 *
 * Software PWM:
 *   PWM targets are copied under a mutex by a dedicated thread. At the start of
 *   each period, active motor outputs are enabled and then disabled at their
 *   individual duty-cycle deadlines.
 */

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
#include <mutex>
#include <thread>
#include <tuple>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace robotarm_hardware
{

/**
 * @brief Initialize default hardware, serial, filter, and PWM state.
 *
 * Per-joint arrays are sized and configured later in on_init(), after the
 * ros2_control hardware description is available.
 */
RobotArmSystem::RobotArmSystem()
: chip_(nullptr),
  software_pwm_frequency_hz_(100.0),
  pwm_deadband_(0.002),
  velocity_deadband_rad_s_(0.020),
  velocity_filter_alpha_(0.25),
  pwm_start_time_(std::chrono::steady_clock::now()),
  serial_fd_(-1),
  serial_device_("/dev/ttyACM0"),
  arduino_counts_initialized_(false),
  coast_time_after_command_s_(0.20),
  pwm_thread_running_(false)
{
  last_arduino_counts_.fill(0);
}

/**
 * @brief Stop motor output and release all hardware resources.
 */
RobotArmSystem::~RobotArmSystem()
{
  stop_pwm_thread();
  stop_all();
  release_gpio();
  close_arduino_serial();
}

/**
 * @brief Initialize ros2_control interfaces and physical hardware.
 *
 * @param info
 *   Parsed ros2_control hardware description from the robot configuration.
 *
 * @return
 *   SUCCESS when all joints are recognized, serial communication opens, GPIO
 *   lines are claimed, and the PWM thread starts. Otherwise ERROR.
 */
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
  pwm_targets_.assign(n, 0.0);

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

      ticks_per_joint_rev_[i] = 575.1;

      direction_[i] = 1.0;
      max_pwm_[i] = 0.50;

      min_pwm_[i] = 0.16;
      velocity_to_pwm_gain_[i] = 0.30;
      velocity_kp_[i] = 0.06;

      min_command_velocity_[i] = 0.03;
      max_joint_velocity_[i] = 0.30;

      lower_limit_[i] = -3.14159265;
      upper_limit_[i] = 3.14159265;
    }
    else if (name == "shoulder_joint")
    {
      forward_gpio_[i] = 25;
      backward_gpio_[i] = 24;
      arduino_channel_[i] = 1;

      ticks_per_joint_rev_[i] = 2556.0;

      direction_[i] = 1.0;
      max_pwm_[i] = 0.40;

      min_pwm_[i] = 0.12;
      velocity_to_pwm_gain_[i] = 0.35;
      velocity_kp_[i] = 0.08;

      min_command_velocity_[i] = 0.035;
      max_joint_velocity_[i] = 0.18;

      lower_limit_[i] = -0.52359878;
      upper_limit_[i] = 1.39626340;
    }
    else if (name == "elbow_joint")
    {
      forward_gpio_[i] = 13;
      backward_gpio_[i] = 12;
      arduino_channel_[i] = 2;

      ticks_per_joint_rev_[i] = 2556.0;

      direction_[i] = 1.0;
      max_pwm_[i] = 0.40;

      min_pwm_[i] = 0.14;
      velocity_to_pwm_gain_[i] = 0.30;
      velocity_kp_[i] = 0.08;

      min_command_velocity_[i] = 0.035;
      max_joint_velocity_[i] = 0.22;

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
  }

  pwm_start_time_ = std::chrono::steady_clock::now();
  start_pwm_thread();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
/**
 * @brief Export position and velocity state interfaces for every joint.
 *
 * @return
 *   Vector containing one position and one velocity interface per joint.
 */
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
/**
 * @brief Export one velocity command interface for every joint.
 *
 * @return
 *   Vector containing the writable velocity command interfaces.
 */
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

/**
 * @brief Open and configure the Arduino encoder serial device.
 *
 * @return
 *   True when the device opens and termios configuration succeeds.
 */
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

  return true;
}

/**
 * @brief Close the Arduino serial file descriptor when open.
 */
void RobotArmSystem::close_arduino_serial()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

/**
 * @brief Parse one comma-separated Arduino encoder line.
 *
 * @param line
 *   Input line containing four integer encoder counts.
 *
 * @param counts
 *   Output array updated only when all four values parse successfully.
 *
 * @return
 *   True when exactly four integer fields are read successfully.
 */
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

/**
 * @brief Read all currently available serial data and keep the newest valid line.
 *
 * @param counts
 *   Input/output encoder-count array. It is replaced by the newest complete
 *   valid line when one is received.
 *
 * @return
 *   True when at least one valid encoder line is parsed.
 */
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

/**
 * @brief Update joint position and velocity state from Arduino encoder counts.
 *
 * @param time
 *   Current ros2_control update time. Unused by this implementation.
 *
 * @param period
 *   Time elapsed since the previous read cycle.
 *
 * @return
 *   hardware_interface::return_type::OK.
 */
hardware_interface::return_type RobotArmSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  std::array<long, 4> current_counts = last_arduino_counts_;

  if (!read_arduino_counts(current_counts))
  {
    for (size_t i = 0; i < velocity_.size(); ++i)
    {
      velocity_[i] *= (1.0 - velocity_filter_alpha_);

      if (std::abs(velocity_[i]) < 1e-6)
      {
        velocity_[i] = 0.0;
      }
    }

    return hardware_interface::return_type::OK;
  }

  if (!arduino_counts_initialized_)
  {
    last_arduino_counts_ = current_counts;
    arduino_counts_initialized_ = true;

    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const int channel = arduino_channel_[i];

    if (channel < 0 || channel >= 4)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmSystem"),
        "Invalid Arduino encoder channel %d for %s",
        channel,
        info_.joints[i].name.c_str()
      );
      continue;
    }

    const long previous_count = last_arduino_counts_[channel];
    const long current_count = current_counts[channel];
    long raw_delta_ticks = current_count - previous_count;

    if (raw_delta_ticks < 0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("RobotArmSystem"),
        "Encoder counter reset/wrap for %s: previous=%ld current=%ld",
        info_.joints[i].name.c_str(),
        previous_count,
        current_count
      );
      raw_delta_ticks = 0;
    }

    /*
     * The encoders provide tick magnitude only, without direction.
     * Therefore use the active motor-command direction, or the most
     * recently known direction while the motor coasts or settles.
     * No positive encoder delta is discarded.
     */
    double sign_for_ticks = last_motion_sign_[i];

    if (sign_for_ticks == 0.0)
    {
      sign_for_ticks = last_valid_motion_sign_[i];
    }

    if (sign_for_ticks == 0.0)
    {
      sign_for_ticks = 1.0;
    }

    const double signed_delta_ticks =
      sign_for_ticks * static_cast<double>(raw_delta_ticks);

    encoder_ticks_[i] += signed_delta_ticks;

    const double delta_rad =
      signed_delta_ticks /
      ticks_per_joint_rev_[i] *
      2.0 * M_PI;

    position_[i] += delta_rad;

    double measured_velocity = 0.0;

    if (dt > 1e-9)
    {
      measured_velocity = delta_rad / dt;
    }

    velocity_[i] =
      velocity_filter_alpha_ * measured_velocity +
      (1.0 - velocity_filter_alpha_) * velocity_[i];

    if (std::abs(velocity_[i]) < 1e-6)
    {
      velocity_[i] = 0.0;
    }
  }

  last_arduino_counts_ = current_counts;

  return hardware_interface::return_type::OK;
}

/**
 * @brief Convert velocity commands into protected per-joint PWM targets.
 *
 * @param time
 *   Current ros2_control update time, stored for active command tracking.
 *
 * @param period
 *   Time elapsed since the previous write cycle. Unused here.
 *
 * @return
 *   hardware_interface::return_type::OK.
 */
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

    double joint_velocity_deadband =
      velocity_deadband_rad_s_;

    if (info_.joints[i].name == "base_joint")
    {
      joint_velocity_deadband = 0.028;
    }

    if (std::abs(desired_velocity) <= joint_velocity_deadband)
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

/**
 * @brief Store one bounded signed PWM target for the software PWM thread.
 *
 * @param i
 *   Joint index.
 *
 * @param pwm_command
 *   Signed PWM request. Positive and negative values select opposite motor
 *   directions. Values inside pwm_deadband_ become zero.
 */
void RobotArmSystem::set_motor(size_t i, double pwm_command)
{
  if (i >= pwm_targets_.size() || i >= max_pwm_.size())
  {
    return;
  }

  double target = 0.0;

  if (std::abs(pwm_command) > pwm_deadband_)
  {
    target = std::clamp(
      pwm_command,
      -max_pwm_[i],
      max_pwm_[i]
    );
  }

  std::lock_guard<std::mutex> lock(pwm_mutex_);
  pwm_targets_[i] = target;
}

/**
 * @brief Start the dedicated software-PWM worker thread once.
 */
void RobotArmSystem::start_pwm_thread()
{
  if (pwm_thread_running_.exchange(true))
  {
    return;
  }

  pwm_thread_ = std::thread(&RobotArmSystem::pwm_loop, this);
}

/**
 * @brief Stop and join the software-PWM worker thread.
 */
void RobotArmSystem::stop_pwm_thread()
{
  if (!pwm_thread_running_.exchange(false))
  {
    return;
  }

  pwm_condition_.notify_all();

  if (pwm_thread_.joinable())
  {
    pwm_thread_.join();
  }
}

/**
 * @brief Generate software PWM for every configured motor output.
 *
 * Each cycle enables the selected direction output, schedules per-joint
 * turn-off events according to duty cycle, and waits until the next period.
 */
void RobotArmSystem::pwm_loop()
{
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;

  const Duration period(
    1.0 / std::max(1.0, software_pwm_frequency_hz_)
  );

  auto next_period = Clock::now();

  while (pwm_thread_running_.load())
  {
    next_period +=
      std::chrono::duration_cast<Clock::duration>(period);

    std::vector<double> targets;

    {
      std::lock_guard<std::mutex> lock(pwm_mutex_);
      targets = pwm_targets_;
    }

    /*
     * Every tuple contains:
     *   off deadline, joint index, direction
     *
     * All active outputs are switched on at the beginning of the period.
     * They are then switched off at their individual duty-cycle deadline.
     */
    std::vector<std::tuple<Clock::time_point, size_t, int>> off_events;
    off_events.reserve(targets.size());

    const auto period_start = Clock::now();

    for (size_t i = 0; i < targets.size(); ++i)
    {
      if (
        i >= forward_lines_.size() ||
        i >= backward_lines_.size() ||
        i >= max_pwm_.size())
      {
        continue;
      }

      const double command = targets[i];

      if (
        std::abs(command) <= pwm_deadband_ ||
        max_pwm_[i] <= 0.0)
      {
        gpiod_line_set_value(forward_lines_[i], 0);
        gpiod_line_set_value(backward_lines_[i], 0);
        continue;
      }

      const double duty = std::clamp(
        std::abs(command) / max_pwm_[i],
        0.0,
        1.0
      );

      const int direction = command > 0.0 ? 1 : -1;

      if (direction > 0)
      {
        gpiod_line_set_value(forward_lines_[i], 1);
        gpiod_line_set_value(backward_lines_[i], 0);
      }
      else
      {
        gpiod_line_set_value(forward_lines_[i], 0);
        gpiod_line_set_value(backward_lines_[i], 1);
      }

      if (duty < 1.0)
      {
        const auto on_time =
          std::chrono::duration_cast<Clock::duration>(
            period * duty
          );

        off_events.emplace_back(
          period_start + on_time,
          i,
          direction
        );
      }
    }

    std::sort(
      off_events.begin(),
      off_events.end(),
      [](const auto & a, const auto & b)
      {
        return std::get<0>(a) < std::get<0>(b);
      }
    );

    for (const auto & event : off_events)
    {
      if (!pwm_thread_running_.load())
      {
        break;
      }

      const auto deadline = std::get<0>(event);
      const size_t joint = std::get<1>(event);

      std::unique_lock<std::mutex> wait_lock(pwm_wait_mutex_);

      pwm_condition_.wait_until(
        wait_lock,
        deadline,
        [this]()
        {
          return !pwm_thread_running_.load();
        }
      );

      wait_lock.unlock();

      if (!pwm_thread_running_.load())
      {
        break;
      }

      if (
        joint < forward_lines_.size() &&
        joint < backward_lines_.size())
      {
        gpiod_line_set_value(forward_lines_[joint], 0);
        gpiod_line_set_value(backward_lines_[joint], 0);
      }
    }

    if (!pwm_thread_running_.load())
    {
      break;
    }

    std::unique_lock<std::mutex> wait_lock(pwm_wait_mutex_);

    pwm_condition_.wait_until(
      wait_lock,
      next_period,
      [this]()
      {
        return !pwm_thread_running_.load();
      }
    );
  }

  for (size_t i = 0; i < forward_lines_.size(); ++i)
  {
    if (i < backward_lines_.size())
    {
      gpiod_line_set_value(forward_lines_[i], 0);
      gpiod_line_set_value(backward_lines_[i], 0);
    }
  }
}

/**
 * @brief Clear every PWM target and drive all motor GPIO outputs low.
 */
void RobotArmSystem::stop_all()
{
  {
    std::lock_guard<std::mutex> lock(pwm_mutex_);
    std::fill(pwm_targets_.begin(), pwm_targets_.end(), 0.0);
  }

  for (size_t i = 0; i < forward_lines_.size(); ++i)
  {
    if (i < backward_lines_.size())
    {
      gpiod_line_set_value(forward_lines_[i], 0);
      gpiod_line_set_value(backward_lines_[i], 0);
    }
  }
}

/**
 * @brief Release all claimed GPIO lines and close the GPIO chip.
 */
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