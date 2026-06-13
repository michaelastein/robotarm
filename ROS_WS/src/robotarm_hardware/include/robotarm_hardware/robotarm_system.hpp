#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <string>

namespace robotarm_hardware
{

class RobotArmSystem : public hardware_interface::SystemInterface
{
public:
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
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> command_;

  std::vector<int> forward_gpio_;
  std::vector<int> backward_gpio_;
  std::vector<int> encoder_gpio_;

  std::vector<double> gear_ratio_;
  std::vector<double> direction_;
  std::vector<double> max_pwm_;

  std::vector<double> encoder_ticks_;
  std::vector<double> last_encoder_ticks_;

  double counts_per_motor_output_rev_ = 127.8;

  void stop_all();
  void set_motor(size_t i, double pwm);
};

}
