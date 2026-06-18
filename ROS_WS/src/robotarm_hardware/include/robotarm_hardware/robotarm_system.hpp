#ifndef ROBOTARM_HARDWARE__ROBOTARM_SYSTEM_HPP_
#define ROBOTARM_HARDWARE__ROBOTARM_SYSTEM_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

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
  void set_motor(size_t i, double command);
  void stop_all();
  void release_gpio();

  bool open_arduino_serial();
  void close_arduino_serial();
  bool read_arduino_counts(std::array<long, 4> & counts);
  bool parse_arduino_line(const std::string & line, std::array<long, 4> & counts);

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> command_;

  std::vector<double> encoder_ticks_;
  std::vector<double> ticks_per_joint_rev_;

  std::vector<int> forward_gpio_;
  std::vector<int> backward_gpio_;

  std::vector<gpiod_line *> forward_lines_;
  std::vector<gpiod_line *> backward_lines_;

  // Arduino channel index:
  // Arduino prints: base,shoulder,elbow,extra
  std::vector<int> arduino_channel_;

  std::array<long, 4> last_arduino_counts_;
  bool arduino_counts_initialized_;
  std::string serial_buffer_;
  int serial_fd_;
  std::string serial_device_;

  // Sign of the last actual physical motor movement:
  // +1 = forward GPIO active
  // -1 = backward GPIO active
  //  0 = no known movement yet
  std::vector<double> last_motion_sign_;

  std::vector<double> direction_;
  std::vector<double> max_pwm_;

  std::vector<double> lower_limit_;
  std::vector<double> upper_limit_;

  gpiod_chip * chip_;

  double software_pwm_frequency_hz_;
  double deadband_;

  std::chrono::steady_clock::time_point pwm_start_time_;
};

}  // namespace robotarm_hardware

#endif  // ROBOTARM_HARDWARE__ROBOTARM_SYSTEM_HPP_
