#include "slambot_control/diffbot_system.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream> // For stringstream

using namespace std::chrono_literals;

namespace slambot_control
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.ticks_per_rev = std::stof(info_.hardware_parameters["ticks_per_rev"]);

  // Initialize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());



  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Finished initialization.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Replace the old on_configure function with this new one

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  // Set the baud rate for the serial port
  LibSerial::BaudRate baud_rate;
  switch (cfg_.baud_rate)
  {
    case 115200:
      baud_rate = LibSerial::BaudRate::BAUD_115200;
      break;
    default:
      RCLCPP_FATAL(rclcpp::get_logger("DiffBotSystemHardware"), "Unsupported baud rate: %d", cfg_.baud_rate);
      return hardware_interface::CallbackReturn::ERROR;
  }

  // open the serial port
  try {
    serial_port_.Open(cfg_.device);
    serial_port_.SetBaudRate(baud_rate); // Use the correct enum value
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("DiffBotSystemHardware"), "Failed to open serial port %s: %s", cfg_.device.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured and opened serial port.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Wheel Joints
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "position", &hw_positions_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "velocity", &hw_velocities_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, "position", &hw_positions_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, "velocity", &hw_velocities_[1]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, "velocity", &hw_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, "velocity", &hw_commands_[1]));
  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");
  // Set some default values
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");
  serial_port_.Close();
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  try {
    std::string response;
    // Read all available lines from the serial port
    while (serial_port_.IsDataAvailable()) {
      serial_port_.ReadLine(response, '\n', cfg_.timeout_ms);

      // Parse encoder data: "e <left_ticks> <right_ticks>"
      if (response.rfind("e ", 0) == 0) {
        long tick_l, tick_r;
        sscanf(response.c_str(), "e %ld %ld", &tick_l, &tick_r);

        double delta_pos_l = (tick_l / cfg_.ticks_per_rev) * (2 * M_PI);
        double delta_pos_r = (tick_r / cfg_.ticks_per_rev) * (2 * M_PI);

        // This assumes the encoder values from ESP32 are total accumulated ticks
        // If they reset on each read, the logic would be hw_positions_[0] += delta_pos_l;
        hw_positions_[0] = delta_pos_l;
        hw_positions_[1] = delta_pos_r;

        // Naive velocity calculation, can be improved with filters
        hw_velocities_[0] = (hw_positions_[0] - hw_velocities_[0]) / period.seconds();
        hw_velocities_[1] = (hw_positions_[1] - hw_velocities_[1]) / period.seconds();
      }

    }
  } catch (const LibSerial::ReadTimeout &) {
    // This is expected if no data is available
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Send velocity commands to the ESP32
  std::stringstream ss;
  ss << "v " << hw_commands_[0] << " " << hw_commands_[1] << "\n";
  try {
    serial_port_.Write(ss.str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Failed to write to serial port: %s", e.what());
  }
  return hardware_interface::return_type::OK;
}
}  // namespace my_robot_bringup

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(slambot_control::DiffBotSystemHardware, hardware_interface::SystemInterface)