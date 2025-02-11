/**
Software License Agreement (BSD)

\file      ewellix_node.cpp
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

#include "ewellix_driver/ewellix_node/ewellix_node.hpp"

namespace ewellix_driver
{

EwellixNode::EwellixNode(const std::string node_name)
: Node(node_name)
{
  // Declare Parameters
  this->declare_parameter("joint_count", 2);
  this->declare_parameter("port", "/dev/ttyUSB0");
  this->declare_parameter("baud", 38400);
  this->declare_parameter("timeout", 1000);
  this->declare_parameter("conversion", 3225.0);
  this->declare_parameter("rated_effort", 2000.0);
  this->declare_parameter("tolerance", 0.005);
  this->declare_parameter("frequency", 10.0);

  // Get Parameters
  this->get_parameter("joint_count", joint_count_);
  this->get_parameter("port", port_);
  this->get_parameter("baud", baud_);
  this->get_parameter("timeout", timeout_);
  this->get_parameter("conversion", conversion_);
  this->get_parameter("rated_effort", rated_effort_);
  this->get_parameter("tolerance", tolerance_);
  this->get_parameter("frequency", frequency_);

  RCLCPP_INFO(this->get_logger(),
  "\nParameters:\n  joint_count: %d\n  port: %s\n  baud: %d\n  timeout: %d\n  conversion: %f\n  rated_effort: %f\n  tolerance: %f\n  frequency: %f", joint_count_, port_.c_str(), baud_, timeout_, conversion_, rated_effort_, tolerance_, frequency_
  );

  // Initialize Variables
  encoder_positions_ = std::vector<int>(joint_count_, 0);
  encoder_commands_ = std::vector<int>(joint_count_, 0);
  speed_ = std::vector<uint16_t>(joint_count_, 0);
  speed_commands_ = std::vector<uint16_t>(joint_count_, 0);
  positions_ = std::vector<double>(joint_count_, 0);
  position_commands_ = std::vector<double>(joint_count_, 0);
  old_positions_ = std::vector<double>(joint_count_, 0);
  velocities_ = std::vector<double>(joint_count_, 0);
  efforts_ = std::vector<double>(joint_count_, 0);
  activated_ = false;
  async_error_ = false;
  async_thread_shutdown_ = false;

  // Create serial port
  ewellix_serial_ = std::make_unique<EwellixSerial>(port_, baud_, timeout_);

  // Open Serial
  if(!ewellix_serial_->open())
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to open port communication.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Successfully opened port.");

  // Activate communication
  if (!ewellix_serial_->activate())
  {
    RCLCPP_INFO(this->get_logger(), "Failed to activate. Trying again...");
    if (!ewellix_serial_->activate())
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to activate remote communication.");
      exit(1);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Successfully activated remote communication.");

  // Initial Cycle
  if (!ewellix_serial_->cycle())
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to cycle remote communication.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Successfully cycled remote communication.");

  // Setup CyclicObject2 to send and receive lift state
  if (!ewellix_serial_->setCyclicObject2())
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to set CyclicObject2.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Successfully set CyclicObject2");

  // Stop to clear movement flags.
  if (!ewellix_serial_->stopAll())
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to stop all actuators.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Successfully stopped all actuators.");

  // Setup ROS Interfaces
  subCommand_ = this->create_subscription<ewellix_interfaces::msg::Command>(
    "command",
    10,
    std::bind(&EwellixNode::commandCallback, this, std::placeholders::_1)
  );

  pubState_ = this->create_publisher<ewellix_interfaces::msg::State>("state", 10);

  // Publish loop
  run_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(1000/frequency_)), std::bind(&EwellixNode::run, this));

  // Start thread
  activated_ = true;

  async_thread_ = std::make_shared<std::thread>(&EwellixNode::asyncThread, this);
}

void
EwellixNode::commandCallback(const ewellix_interfaces::msg::Command &msg)
{
  float command;
  if(msg.ticks > 0)
  {
    command = msg.ticks / conversion_;
  }
  else if(msg.meters > 0)
  {
    command = msg.meters;
  }
  else
  {
    command = 0;
  }
  for (int i = 0; i < joint_count_; i++)
  {
    position_commands_[i] = command;
  }
}

void
EwellixNode::run()
{
  ewellix_interfaces::msg::State msg_state;
  msg_state.actual_positions = state_.actual_positions;
  msg_state.remote_positions = state_.remote_positions;
  msg_state.speeds = state_.speeds;
  msg_state.currents = state_.currents;
  for (size_t i = 0; i < state_.status.size(); i++)
  {
    msg_state.status.push_back(state_.status[i].code);
  }
  for (size_t i = 0; i < state_.errors.size(); i++)
  {
    msg_state.errors.push_back(state_.errors[i].code);
  }
  pubState_->publish(msg_state);
}

/**
 * Cycle to update state
 */
bool
EwellixNode::updateState()
{
  // Convert commands
  convertCommands();

  // Cycle Communication to keep alive
  if(!ewellix_serial_->cycle2(encoder_commands_, data_))
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Failed to cycle2 EwellixSerial port.");
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
EwellixNode::executeCommand()
{
  // Execute Motion
  if(outOfPosition() && !inMotion())
  {
    RCLCPP_DEBUG(rclcpp::get_logger("EwellixNode"), "Moving!");
    if(!ewellix_serial_->stopAll())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Failed to send stop.");
      return false;
    }
    if(!ewellix_serial_->executeAllRemote())
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Failed to send execute command.");
      return false;
    }
    // Sleep to Allow Motion to begin
    usleep(100000);
  }

  return true;
}

/**
 * Async communication thread
 */
void
EwellixNode::asyncThread()
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
      // Error
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
EwellixNode::outOfPosition()
{
  for(int i = 0; i < joint_count_; i++)
  {
    if((abs(state_.actual_positions[i] - encoder_commands_[i]) > (tolerance_ * conversion_)))
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
EwellixNode::inMotion()
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
EwellixNode::convertCommands()
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
EwellixNode::errorTriggered()
{
  EwellixSerial::SCUError scu_error = state_.errors[0];
  if(scu_error.code == 0 && async_error_ == false)
  {
    return false;
  }
  if(scu_error.fault_ram)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "CRC error with ROM test. Faulty ROM. Motions are stopped and the control unit carries out a reset.");
  }
  if(scu_error.fault_rom)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Error with RAM test. Faulty RAM. Motions are stopped and the control unit carries out a reset.");
  }
  if(scu_error.fault_cpu)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Error with CPU test. Faulty CPU. Motions are stopped and the control unit carries out a reset.");
  }
  if(scu_error.stack_overrun)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "STACK overrun detected. Motions are stopped (fast stop) and the control unit carries out a reset.");
  }
  if(scu_error.sequence_error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Program sequence error. Watchdog reset. Motions are stopped (fast stop) and the control unit carries out a reset.");
  }
  if(scu_error.hand_switch_short)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Error with hand switch test. Short detected in hand switch. Only occurs if hand switch is parameterized as 'safe'. Motions are stopped (fast stop).");
  }
  if(scu_error.binary_inputs_short)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Error with binary inputs. Short detected between binary inputs. Only occurs if binary inputs are parameterized as safe and no analogue input is parameterized. Motions are stopped (fast stop).");
  }
  if(scu_error.faulty_relay)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Error with relay and FET tests. Faulty relay or FET. Test performed at start of motion. Motion not executed.");
  }
  if(scu_error.move_enable_comms)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Error with communication with MoveEnable controller. No reply form MoveEnable controller. Motions stopped (fast stop).");
  }
  if(scu_error.move_enable_incorrect)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Error with MoveEnable output test. The MoveEnable controller output is incorrect. Motions stopped (fast stop).");
  }
  if(scu_error.over_temperature)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Over-temperature detected at rectifier or FET. Motions stopped (fast stop).");
  }
  if(scu_error.battery_discharge)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Switching off due to excessive discharge of battery. Motions stopped (fast stop). Control unit switches itself off.");
  }
  if(scu_error.over_current)
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("EwellixNode"), "Total current is exceeded. Occurs if motion in process. Motions stopped (fast stop). Bit reset in the next motion.");
  }
  if(scu_error.drive_1_error | scu_error.drive_2_error | scu_error.drive_3_error | scu_error.drive_4_error | scu_error.drive_5_error | scu_error.drive_6_error)
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
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Position between drives too great. Only if synchronized parallel run is parameterized. Motion not started. If motion ins progress the motion is stopped (fast stop). Bit reset on next motion.");
  }
  if(scu_error.remote_timeout)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Remote communication timeout. Depends on SafetyMode set at activation.");
  }
  if(scu_error.lockbox_comm_error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Locking box I2C communication error. Only if locking box parameterized as 'safe'. Motion not performed or stopped");
  }
  if(scu_error.ram_config_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "RAM copy of EEPROM configuration data indicates incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_user_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "RAM copy of EEPROM user data indicates incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_lockbox_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "EEPROM locking box data indicates incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_dynamic_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "RAM copy of EEPROM dynamic data indicate incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_calib_data_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "RAM copy of EEPROM calibration data indicate incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.ram_hw_settings_crc)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "RAM copy of EEPROM HW settings indicate incorrect CRC. Motion not performed or stopped.");
  }
  if(scu_error.io_test)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "IO test performed if no motion is active. Motion not performed.");
  }
  if(scu_error.idf_opsys_error)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "IDF operating system error. Motion not performed or stopped.");
  }
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("EwellixNode"), "Try to move lift with remote. If not moving, reset lift by power-cycling and holding both UP and DOWN buttons for 5+ seconds.");
  return true;
}

}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<ewellix_driver::EwellixNode> ewellix_node =
    std::make_shared<ewellix_driver::EwellixNode>("ewellix_node");

  exe.add_node(ewellix_node);
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
