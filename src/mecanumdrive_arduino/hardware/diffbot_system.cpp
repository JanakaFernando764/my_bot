// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mecanumdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecanumdrive_arduino
{
hardware_interface::CallbackReturn MecDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)

{

  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "=== Hardware parameters ===");
    for (const auto& param : info.hardware_parameters) {
        RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), 
                    "  %s = '%s'", param.first.c_str(), param.second.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "=========================");

    
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_fl_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_fr_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_bl_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_br_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fl_.name, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fr_.name, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_bl_.name, hardware_interface::HW_IF_POSITION, &wheel_bl_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_bl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_bl_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_br_.name, hardware_interface::HW_IF_POSITION, &wheel_br_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_br_.name, hardware_interface::HW_IF_VELOCITY, &wheel_br_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_bl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_bl_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_br_.name, hardware_interface::HW_IF_VELOCITY, &wheel_br_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn MecDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MecDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("MecDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_fl_.enc,wheel_fr_.enc,wheel_bl_.enc,wheel_br_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_fl_.pos;
  wheel_fl_.pos = wheel_fl_.calc_enc_angle();
  wheel_fl_.vel = (wheel_fl_.pos - pos_prev) / delta_seconds;
  
  pos_prev = wheel_fr_.pos;
  wheel_fr_.pos = wheel_fr_.calc_enc_angle();
  wheel_fr_.vel = (wheel_fr_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_bl_.pos;
  wheel_bl_.pos = wheel_bl_.calc_enc_angle();
  wheel_bl_.vel = (wheel_bl_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_br_.pos;
  wheel_br_.pos = wheel_br_.calc_enc_angle();
  wheel_br_.vel = (wheel_br_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type mecanumdrive_arduino ::MecDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_fl_counts_per_loop = wheel_fl_.cmd / wheel_fl_.rads_per_count / cfg_.loop_rate;
  int motor_fr_counts_per_loop = wheel_fr_.cmd / wheel_fr_.rads_per_count / cfg_.loop_rate;
  int motor_bl_counts_per_loop = wheel_bl_.cmd / wheel_bl_.rads_per_count / cfg_.loop_rate;
  int motor_br_counts_per_loop = wheel_br_.cmd / wheel_br_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_fl_counts_per_loop, motor_fr_counts_per_loop,motor_bl_counts_per_loop,motor_br_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace mecanumdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanumdrive_arduino::MecDriveArduinoHardware, hardware_interface::SystemInterface)
