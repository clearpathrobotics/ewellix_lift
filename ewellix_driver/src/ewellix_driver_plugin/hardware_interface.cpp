/**
Software License Agreement (BSD)

\file      hardware_interface.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "ewellix_driver/ewellix_driver_plugin/hardware_interface.hpp"

namespace ewellix_driver
{

EwellixHardwareInterface::~EwellixHardwareInterface()
{
  // On clean up
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn
EwellixHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = system_info;
  joint_count_ = 0;
  in_motion_ = false;
  activated_ = false;

  // Check joint command and state interfaces
  for (const hardware_interface::ComponentInfo& joint: info_.joints)
  {
    if(joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("EwellixHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if(joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("EwellixHardwareInterface"),
                   "Joint '%s' has %s command interface listed as first. '%s' expected.", joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("EwellixHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("EwellixHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("EwellixHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
    encoder_positions_.push_back(0);
    encoder_commands_.push_back(0);
    speed_.push_back(0);
    speed_commands_.push_back(0);
    positions_.push_back(0);
    position_commands_.push_back(0);
    old_positions_.push_back(0);
    velocities_.push_back(0);
    efforts_.push_back(0);
    joint_count_++;
  }
  state_.actual_positions = {0, 0, 0, 0, 0, 0};
  state_.remote_positions = {0, 0, 0, 0, 0, 0};
  state_.speeds = {0, 0, 0, 0, 0, 0};
  state_.currents = {0, 0, 0, 0, 0, 0};
  state_.status1 = {0, 0, 0, 0, 0, 0};
  state_.errors = {0, 0, 0, 0, 0};
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EwellixHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EwellixHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
EwellixHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Configuring...");

  // Port
  const std::string port = info_.hardware_parameters["port"];
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "%s", port.c_str());

  // Baud Rate
  const int baud = std::stoi(info_.hardware_parameters["baud"]);
  // Timeout
  const int timeout = std::stoi(info_.hardware_parameters["timeout"]);
  // Encoder Conversion
  conversion_ = std::stof(info_.hardware_parameters["conversion"]);
  // Rated Effort
  rated_effort_ = std::stof(info_.hardware_parameters["rated_effort"]);
  // Tolerance
  tolerance_ = std::stof(info_.hardware_parameters["tolerance"]);

  // Create EwellixSerial
  ewellix_serial_ = std::make_unique<EwellixSerial>(
    port, baud, timeout);

  // Open Serial
  if(!ewellix_serial_->open())
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to open EwellixSerial.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EwellixHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Activating...");

  // Activate Communication
  if(!ewellix_serial_->activate())
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to activate EwellixSerial remote control.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Cycle Communication to keep alive
  if(!ewellix_serial_->cycle2(encoder_commands_, data_))
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to cycle2 EwellixSerial port.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  state_ = ewellix_serial_->convertCycle2(data_);
  // Initialize position command with current position.
  for(int i = 0; i < joint_count_; i++)
  {
    encoder_commands_[i] = state_.actual_positions[i];
    position_commands_[i] = encoder_commands_[i] / conversion_;
  }

  // Stop command to clear flags
  if(!ewellix_serial_->stopAll())
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to send stop.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  activated_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EwellixHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Deactivating...");

  // Deactivate Communication
  if(!ewellix_serial_->deactivate())
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to deactivate EwellixSerial remote control.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  activated_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EwellixHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Cleanup...");

  // Close communication
  // if(!ewellix_serial_->close())
  // {
  //   RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to close EwellixSerial port.");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  ewellix_serial_->close();

  ewellix_serial_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EwellixHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Shutdown...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EwellixHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Error!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
EwellixHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Cycle
  if (activated_)
  {
    for(int i = 0; i < joint_count_; i++)
    {
      encoder_commands_[i] = position_commands_[i] * conversion_;
    }
    // Cycle Communication to keep alive
    if(!ewellix_serial_->cycle2(encoder_commands_, data_))
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to cycle2 EwellixSerial port.");
      return hardware_interface::return_type::ERROR;
    }
    state_ = ewellix_serial_->convertCycle2(data_);
  }
  for(int i = 0; i < joint_count_; i++)
  {
    // Read
    encoder_positions_[i] = state_.actual_positions[i];
    encoder_commands_[i] = state_.remote_positions[i];
    // Store previous positions
    old_positions_[i] = positions_[i];
    // Convert encoder ticks to meters
    positions_[i] = encoder_positions_[i] / conversion_;
    // Calculate velocity
    velocities_[i] = (positions_[i] - old_positions_[i]) / period.seconds();
    // Calculate effort
    efforts_[i] = state_.speeds[i] / 100 * rated_effort_;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
EwellixHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if(activated_)
  {
    // Stop
    if(in_motion_ && !outOfPosition())
    {
      RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Stop!");
      if(!ewellix_serial_->stopAll())
      {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to send stop.");
        return hardware_interface::return_type::ERROR;
      }
      in_motion_ = false;
    }

    // Execute Motion
    if(!in_motion_ && outOfPosition())
    {
      RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Moving!");

      if(!ewellix_serial_->executeAllRemote())
      {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to send execute command.");
        return hardware_interface::return_type::ERROR;
      }
      in_motion_ = true;
    }
  }
  return hardware_interface::return_type::OK;
}

bool
EwellixHardwareInterface::outOfPosition()
{
  bool inBounds = true;
  for(int i; i < joint_count_; i++)
  {
    inBounds &= (abs(encoder_positions_[i] - encoder_commands_[i]) <= (tolerance_* conversion_));
  }
  return !inBounds;
}


} // namespace ewellix_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ewellix_driver::EwellixHardwareInterface, hardware_interface::SystemInterface)
