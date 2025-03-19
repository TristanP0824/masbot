#include "masbot_description/masbot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace masbot_hardware_interface
{

hardware_interface::CallbackReturn MasbotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  hw_left_wheel_pos_ = 0.0;
  hw_right_wheel_pos_ = 0.0;
  hw_left_wheel_vel_ = 0.0;
  hw_right_wheel_vel_ = 0.0;
  hw_left_wheel_cmd_ = 0.0;
  hw_right_wheel_cmd_ = 0.0;

  // Get joint names from the URDF
  left_wheel_name_ = info_.hardware_parameters["left_wheel_name"];
  right_wheel_name_ = info_.hardware_parameters["right_wheel_name"];
  
  // Get encoder parameters
  encoder_counts_per_rev_ = std::stoi(info_.hardware_parameters["encoder_counts_per_rev"]);
  
  // Get motor driver pins
  left_motor_pwm_pin_ = std::stoi(info_.hardware_parameters["left_motor_pwm_pin"]);
  left_motor_dir_pin_ = std::stoi(info_.hardware_parameters["left_motor_dir_pin"]);
  right_motor_pwm_pin_ = std::stoi(info_.hardware_parameters["right_motor_pwm_pin"]);
  right_motor_dir_pin_ = std::stoi(info_.hardware_parameters["right_motor_dir_pin"]);
  
  // Get encoder pins
  left_encoder_a_pin_ = std::stoi(info_.hardware_parameters["left_encoder_a_pin"]);
  left_encoder_b_pin_ = std::stoi(info_.hardware_parameters["left_encoder_b_pin"]);
  right_encoder_a_pin_ = std::stoi(info_.hardware_parameters["right_encoder_a_pin"]);
  right_encoder_b_pin_ = std::stoi(info_.hardware_parameters["right_encoder_b_pin"]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasbotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Connect to hardware
  if (!connect_to_hardware()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Successfully configured!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MasbotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export position and velocity state interfaces for both wheels
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_POSITION, &hw_left_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_left_wheel_vel_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_POSITION, &hw_right_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_right_wheel_vel_));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MasbotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Export velocity command interfaces for both wheels
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_left_wheel_cmd_));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_right_wheel_cmd_));
  
  return command_interfaces;
}

hardware_interface::CallbackReturn MasbotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Activating hardware interface...");
  
  // Reset commands and states
  hw_left_wheel_pos_ = 0.0;
  hw_right_wheel_pos_ = 0.0;
  hw_left_wheel_vel_ = 0.0;
  hw_right_wheel_vel_ = 0.0;
  hw_left_wheel_cmd_ = 0.0;
  hw_right_wheel_cmd_ = 0.0;
  
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Hardware interface activated!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MasbotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Deactivating hardware interface...");
  
  // Stop motors
  hw_left_wheel_cmd_ = 0.0;
  hw_right_wheel_cmd_ = 0.0;
  write_motor_commands();
  
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Hardware interface deactivated!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MasbotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read encoder values from hardware
  if (!read_encoders()) {
    RCLCPP_ERROR(rclcpp::get_logger("MasbotHardwareInterface"), "Failed to read encoder values!");
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MasbotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write velocity commands to hardware
  write_motor_commands();
  
  return hardware_interface::return_type::OK;
}

bool MasbotHardwareInterface::connect_to_hardware()
{
  // In a real implementation, this would initialize GPIO pins, 
  // set up communication with motor drivers, etc.
  
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Connecting to MasBot hardware...");
  RCLCPP_INFO(
    rclcpp::get_logger("MasbotHardwareInterface"),
    "Left motor pins: PWM=%d, DIR=%d", left_motor_pwm_pin_, left_motor_dir_pin_);
  RCLCPP_INFO(
    rclcpp::get_logger("MasbotHardwareInterface"),
    "Right motor pins: PWM=%d, DIR=%d", right_motor_pwm_pin_, right_motor_dir_pin_);
  RCLCPP_INFO(
    rclcpp::get_logger("MasbotHardwareInterface"),
    "Left encoder pins: A=%d, B=%d", left_encoder_a_pin_, left_encoder_b_pin_);
  RCLCPP_INFO(
    rclcpp::get_logger("MasbotHardwareInterface"),
    "Right encoder pins: A=%d, B=%d", right_encoder_a_pin_, right_encoder_b_pin_);
  
  // For demonstration purposes, we'll assume connection is successful
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Successfully connected to hardware!");
  
  return true;
}

void MasbotHardwareInterface::disconnect_from_hardware()
{
  // In a real implementation, this would clean up GPIO pins, 
  // close communication with motor drivers, etc.
  
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Disconnecting from hardware...");
  
  // For demonstration purposes, we'll assume disconnection is successful
  RCLCPP_INFO(rclcpp::get_logger("MasbotHardwareInterface"), "Successfully disconnected from hardware!");
}

bool MasbotHardwareInterface::read_encoders()
{
  // In a real implementation, this would read encoder values from hardware
  // and update position and velocity
  
  // For demonstration purposes, we'll simulate encoder readings
  // In a real implementation, you would read actual encoder values
  // and convert them to radians and rad/s
  
  // Simulate wheel movement based on commands
  static double last_left_pos = 0.0;
  static double last_right_pos = 0.0;
  
  // Simple simulation: position changes based on commanded velocity
  hw_left_wheel_pos_ += hw_left_wheel_cmd_ * 0.01;  // Assuming 10ms update rate
  hw_right_wheel_pos_ += hw_right_wheel_cmd_ * 0.01;
  
  // Calculate velocities (simple derivative)
  hw_left_wheel_vel_ = (hw_left_wheel_pos_ - last_left_pos) / 0.01;
  hw_right_wheel_vel_ = (hw_right_wheel_pos_ - last_right_pos) / 0.01;
  
  last_left_pos = hw_left_wheel_pos_;
  last_right_pos = hw_right_wheel_pos_;
  
  return true;
}

void MasbotHardwareInterface::write_motor_commands()
{
  // In a real implementation, this would convert velocity commands to PWM values
  // and send them to the motor drivers
  
  // For demonstration purposes, we'll just log the commands
  RCLCPP_DEBUG(
    rclcpp::get_logger("MasbotHardwareInterface"),
    "Writing motor commands: left=%.2f, right=%.2f", hw_left_wheel_cmd_, hw_right_wheel_cmd_);
  
  // In a real implementation, you would:
  // 1. Convert rad/s to PWM values
  // 2. Set direction pins based on sign of command
  // 3. Set PWM pins to control motor speed
}

}  // namespace masbot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  masbot_hardware_interface::MasbotHardwareInterface,
  hardware_interface::SystemInterface
)
