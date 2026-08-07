#ifndef ROBOTARM_HARDWARE__ROBOTARM_SYSTEM_HPP_
#define ROBOTARM_HARDWARE__ROBOTARM_SYSTEM_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <gpiod.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace robotarm_hardware
{

class RobotArmSystem : public hardware_interface::SystemInterface
{
public:
  RobotArmSystem();
  ~RobotArmSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool open_arduino_serial();
  void close_arduino_serial();
  rclcpp::Time base_last_encoder_tick_time_;
  double base_startup_boost_;
  bool parse_arduino_line(
    const std::string & line,
    std::array<long, 4> & counts);

  bool read_arduino_counts(
    std::array<long, 4> & counts);

  void set_motor(size_t i, double pwm_command);
  void start_pwm_thread();
  void stop_pwm_thread();
  void pwm_loop();
  void stop_all();
  void release_gpio();
  std::mutex pwm_mutex_;
  std::vector<double> pwm_targets_;

  std::thread pwm_thread_;
  std::atomic<bool> pwm_thread_running_;

  std::mutex pwm_wait_mutex_;
  std::condition_variable pwm_condition_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> command_;

  std::vector<double> encoder_ticks_;
  std::vector<double> ticks_per_joint_rev_;

  std::vector<int> forward_gpio_;
  std::vector<int> backward_gpio_;
  std::vector<int> arduino_channel_;

  std::vector<gpiod_line *> forward_lines_;
  std::vector<gpiod_line *> backward_lines_;
  std::vector<double> last_motion_sign_;
  std::vector<double> last_valid_motion_sign_;
  std::vector<rclcpp::Time> last_active_command_time_;
  double coast_time_after_command_s_;
  std::vector<double> direction_;
  std::vector<double> max_pwm_;
  std::vector<double> min_pwm_;

  std::vector<double> velocity_to_pwm_gain_;
  std::vector<double> velocity_kp_;

  std::vector<double> min_command_velocity_;
  std::vector<double> max_joint_velocity_;

  std::vector<double> lower_limit_;
  std::vector<double> upper_limit_;

  gpiod_chip * chip_;

  double software_pwm_frequency_hz_;
  double pwm_deadband_;
  double velocity_deadband_rad_s_;
  double velocity_filter_alpha_;

  std::chrono::steady_clock::time_point pwm_start_time_;

  int serial_fd_;
  std::string serial_device_;
  std::string serial_buffer_;

  bool arduino_counts_initialized_;
  std::array<long, 4> last_arduino_counts_;
};

}  // namespace robotarm_hardware

#endif  // ROBOTARM_HARDWARE__ROBOTARM_SYSTEM_HPP_
