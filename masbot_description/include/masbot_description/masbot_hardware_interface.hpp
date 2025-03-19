#ifndef MASBOT_HARDWARE_INTERFACE_HPP_
#define MASBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace masbot_hardware_interface
{

class MasbotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MasbotHardwareInterface);

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
  // Parameters for the MasBot hardware
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  
  // Encoder counts per revolution
  int encoder_counts_per_rev_;
  
  // Motor driver pins
  int left_motor_pwm_pin_;
  int left_motor_dir_pin_;
  int right_motor_pwm_pin_;
  int right_motor_dir_pin_;
  
  // Encoder pins
  int left_encoder_a_pin_;
  int left_encoder_b_pin_;
  int right_encoder_a_pin_;
  int right_encoder_b_pin_;
  
  // Hardware states
  double hw_left_wheel_pos_;
  double hw_right_wheel_pos_;
  double hw_left_wheel_vel_;
  double hw_right_wheel_vel_;
  double hw_left_wheel_cmd_;
  double hw_right_wheel_cmd_;
  
  // Methods to communicate with the hardware
  bool connect_to_hardware();
  void disconnect_from_hardware();
  bool read_encoders();
  void write_motor_commands();
};

}  // namespace masbot_hardware_interface

#endif  // MASBOT_HARDWARE_INTERFACE_HPP_
