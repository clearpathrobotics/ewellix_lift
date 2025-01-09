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

/**
 * EwellixHardwareInterface destructor
 */
EwellixHardwareInterface::~EwellixHardwareInterface()
{
  // On clean up
  on_cleanup(rclcpp_lifecycle::State());
}

/**
 * On Initialization

 * Parse system information to determine number of joints.
 * Initialize the control variables with the appropriate number of entries.
 */
hardware_interface::CallbackReturn
EwellixHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = system_info;
  joint_count_ = 0;
  activated_ = false;
  async_error_ = false;
  async_thread_shutdown_ = false;

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
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * Export State Interfaces
 *
 * Link the positions, velocities, and effort variables to each hardware state
 * interface.
 */
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

/**
 * Export Command Interfaces
 *
 * Link the position commands to each hardware command interface.
 */
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

/**
 * On Configure
 *
 * Parse the hardware parameters and attempt to open the given port.
 */
hardware_interface::CallbackReturn
EwellixHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Configuring...");

  // Port
  const std::string port = info_.hardware_parameters["port"];
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Port: %s", port.c_str());

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

  // Create asynchronous reading thread
  async_thread_ = std::make_shared<std::thread>(&EwellixHardwareInterface::asyncThread, this);

  return hardware_interface::CallbackReturn::SUCCESS;
}


/**
 * On Activate
 *
 * Send the activate command to the lift to enable remote communication.
 * Send the cycle command to keep the communication alive and to determine the
 * initial setup.
 * Send a stop command to clear all motion flags.
 */
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
  if(!ewellix_serial_->setCyclicObject2())
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to set CyclicObject2.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Cycle Communication to keep alive
  if(!ewellix_serial_->cycle2(encoder_commands_, data_))
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to cycle2.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse state from data
  state_.setFromData(data_);

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

/**
 * On Deactivate
 *
 * Send abort command to stop remote communication mode.
 */
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

/**
 * On Clean-up
 *
 * Close the port.
 */
hardware_interface::CallbackReturn
EwellixHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Cleanup...");

  if(async_thread_)
  {
    async_thread_shutdown_ = true;
    async_thread_->join();
    async_thread_.reset();
  }

  ewellix_serial_->close();

  ewellix_serial_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * On Shutdown
 */
hardware_interface::CallbackReturn
EwellixHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Shutdown...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * On Error
 */
hardware_interface::CallbackReturn
EwellixHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("EwellixHardwareInterface"), "Error!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * Read from hardware
 *
 * Use the cycle2 to sent remote positions and retrieve state of the lift.
 * Check if the lift is at the commanded position and stop.
 * Check if the lift is not at the commanded position and move.
 */
hardware_interface::return_type
EwellixHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point end;

  if(async_error_)
  {
    return hardware_interface::return_type::ERROR;
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

  end = std::chrono::steady_clock::now();
  std::string elapsed = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count());

  return hardware_interface::return_type::OK;
}


/**
 * Write to hardware interface
 * Write and read happens in read command.
 */
hardware_interface::return_type
EwellixHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

/**
 * Cycle to update state
 */
bool
EwellixHardwareInterface::updateState()
{
  // Convert commands
  convertCommands();

  // Cycle Communication to keep alive
  if(!ewellix_serial_->cycle2(encoder_commands_, data_))
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to cycle2 EwellixSerial port.");
    return false;
  }

  // Update state from response data
  state_.setFromData(data_);

  return true;
}

/**
 * Execute command
 */
bool
EwellixHardwareInterface::executeCommand()
{
  // Execute Motion
  if(outOfPosition() && !inMotion())
  {
    RCLCPP_DEBUG(rclcpp::get_logger("EwellixHardwareInterface"), "Stop!");
    if(!ewellix_serial_->stopAll())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to send stop.");
      return false;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("EwellixHardwareInterface"), "Moving!");
    if(!ewellix_serial_->executeAllRemote())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Failed to send execute command.");
      return false;
    }
    // Wait for motion
    usleep(50000);
  }

  return true;
}

/**
 * Async communication thread
 */
void
EwellixHardwareInterface::asyncThread()
{
  async_thread_shutdown_ = false;
  while(!async_thread_shutdown_)
  {
    if(activated_)
    {
      // Update
      if(!updateState())
      {
        async_error_ = true;
      }
      // Error handling
      if(errorTriggered())
      {
        async_error_ = true;
      }
      // Command
      if(!executeCommand())
      {
        async_error_ = true;
      }
    }
  }
}


/**
 * Out of Position
 *
 * Check if all joints are out of the tolerance boundaries.
 */
bool
EwellixHardwareInterface::outOfPosition()
{
  for(int i = 0; i < joint_count_; i++)
  {
    if((abs(encoder_positions_[i] - encoder_commands_[i]) > (tolerance_ * conversion_)))
    {
      return true;
    }
  }
  return false;
}

/**
 * Check if speed readings are above zero
 */
bool
EwellixHardwareInterface::inMotion()
{
  bool moving = false;
  for(int i = 0; i < joint_count_; i++)
  {
    moving |= state_.speeds[i] > 0;
  }
  return moving;
}

/**
 * Convert position commands to encoder commands
 */
void
EwellixHardwareInterface::convertCommands()
{
  for(int i = 0; i < joint_count_; i++)
  {
    encoder_commands_[i] = position_commands_[i] * conversion_;
    if (encoder_commands_[i] < EwellixSerial::EncoderLimit::LOWER)
    {
      encoder_commands_[i] = EwellixSerial::EncoderLimit::LOWER;
    }
    if (encoder_commands_[i] > EwellixSerial::EncoderLimit::UPPER)
    {
      encoder_commands_[i] = EwellixSerial::EncoderLimit::UPPER;
    }
  }
}

/**
 * Check errors in state
 *
 * @return true if there are errors.
 */
bool
EwellixHardwareInterface::errorTriggered()
{
  EwellixSerial::SCUError scu_error = state_.errors[0];
  if(scu_error.code == 0 && async_error_ == false)
  {
    return false;
  }
  if(scu_error.fault_ram)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "CRC error with ROM test. Faulty ROM. Motions are stopped and the control unit carries out a reset.");
  }
  if(scu_error.fault_rom)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Error with RAM test. Faulty RAM. Motions are stopped and the control unit carries out a reset.");
  }
  if(scu_error.fault_cpu)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Error with CPU test. Faulty CPU. Motions are stopped and the control unit carries out a reset.");
  }
  if(scu_error.stack_overrun)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "STACK overrun detected. Motions are stopped (fast stop) and the control unit carries out a reset.");
  }
  if(scu_error.sequence_error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Program sequence error. Watchdog reset. Motions are stopped (fast stop) and the control unit carries out a reset.");
  }
  if(scu_error.hand_switch_short)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Error with hand switch test. Short detected in hand switch. Only occurs if hand switch is parameterized as 'safe'. Motions are stopped (fast stop).");
  }
  if(scu_error.binary_inputs_short)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Error with binary inputs. Short detected between binary inputs. Only occurs if binary inputs are parameterized as safe and no analogue input is parameterized. Motions are stopped (fast stop).");
  }
  if(scu_error.faulty_relay)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Error with relay and FET tests. Faulty relay or FET. Test performed at start of motion. Motion not executed.");
  }
  if(scu_error.move_enable_comms)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Error with communication with MoveEnable controller. No reply form MoveEnable controller. Motions stopped (fast stop).");
  }
  if(scu_error.move_enable_incorrect)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Error with MoveEnable output test. The MoveEnable controller output is incorrect. Motions stopped (fast stop).");
  }
  if(scu_error.over_temperature)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Over-temperature detected at rectifier or FET. Motions stopped (fast stop).");
  }
  if(scu_error.battery_discharge)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Switching off due to excessive discharge of battery. Motions stopped (fast stop). Control unit switches itself off.");
  }
  if(scu_error.over_current)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Total current is exceeded. Occurs if motion in process. Motions stopped (fast stop). Bit reset in the next motion.");
  }
  if(scu_error.drive_1_error || scu_error.drive_2_error || scu_error.drive_3_error || scu_error.drive_4_error || scu_error.drive_5_error || scu_error.drive_6_error )
  {
    int drive = 0;
    drive |= scu_error.drive_1_error * (1 << 1);
    drive |= scu_error.drive_2_error * (1 << 2);
    drive |= scu_error.drive_3_error * (1 << 3);
    drive |= scu_error.drive_4_error * (1 << 4);
    drive |= scu_error.drive_5_error * (1 << 5);
    drive |= scu_error.drive_6_error * (1 << 6);
    RCLCPP_WARN(rclcpp::get_logger("EwellixNode"), "Error with drive #%d. Occurs when peak current reached, short circuit current, sensor monitor, over current or timeout. Drive stopped (fast stop). Bit reset on next motion.", int(std::sqrt(drive)));
    RCLCPP_WARN(rclcpp::get_logger("EwellixNode"), "Attempting to recover...");
    if(!ewellix_serial_->stopAll())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Failed to send stop.");
      return true;
    }
    if(!ewellix_serial_->executeAllOut())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Failed to send execute command.");
      return true;
    }
    async_error_ = false;
    return false;
  }
  if(scu_error.position_difference)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Position between drives too great. Only if synchronized parallel run is parameterized. Motion not started. If motion ins progress the motion is stopped (fast stop). Bit reset on next motion.");
  }
  if(scu_error.remote_timeout)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Remote communication timeout. Depends on SafetyMode set at activation.");
  }
  if(scu_error.lockbox_comm_error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Locking box I2C communication error. Only if locking box parameterized as 'safe'. Motion not performed or stopped");
  }
  if(scu_error.ram_config_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "RAM copy of EEPROM configuration data indicates incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_user_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "RAM copy of EEPROM user data indicates incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_lockbox_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "EEPROM locking box data indicates incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_dynamic_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "RAM copy of EEPROM dynamic data indicate incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_calib_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "RAM copy of EEPROM calibration data indicate incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_hw_settings_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "RAM copy of EEPROM HW settings indicate incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.io_test)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "IO test performed if no motion is active. Motion not performed.");
  }
  if(scu_error.idf_opsys_error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "IDF operating system error. Motion not performed or stopped.");
  }
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixHardwareInterface"), "Try to move lift with remote. If not moving, reset lift by power-cycling and holding both UP and DOWN buttons for 5+ seconds.");
  return true;
}

} // namespace ewellix_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ewellix_driver::EwellixHardwareInterface, hardware_interface::SystemInterface)
