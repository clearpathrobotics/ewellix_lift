/**
 *
 *  \file
 *  \brief      Ewellix hardware interface class
 *  \author     Luis Camero <lcamero@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2024, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */
#ifndef EWELLIX_DRIVER__EWELLIX_HARDWARE_INTERFACE_HPP_
#define EWELLIX_DRIVER__EWELLIX_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "ewellix_driver/ewellix_serial/ewellix_serial.hpp"
#include "ewellix_interfaces/msg/state.hpp"


namespace ewellix_driver
{

class EwellixHardwareInterface
: public hardware_interface::SystemInterface
{
  public:
  EwellixHardwareInterface(){};
  ~EwellixHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() final;

  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) final;

  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) final;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type
  read(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  hardware_interface::return_type
  write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  bool
  updateState();

  bool
  executeCommand();

  void
  asyncThread();

  bool
  outOfPosition();

  bool
  inMotion();

  void
  convertCommands();

  bool
  errorTriggered();

  protected:
  int joint_count_;
  bool activated_;
  bool in_motion_;
  float conversion_;
  float rated_effort_;
  float tolerance_;
  std::vector<int>encoder_positions_, encoder_commands_;
  std::vector<uint16_t>speed_, speed_commands_;
  std::vector<double>positions_, position_commands_, old_positions_;
  std::vector<double>velocities_;
  std::vector<double>efforts_;

  std::unique_ptr<EwellixSerial> ewellix_serial_;

  std::atomic_bool async_error_;
  std::atomic_bool async_thread_shutdown_;
  std::shared_ptr<std::thread> async_thread_;

  std::vector<uint8_t> data_;
  EwellixSerial::Cycle2Data state_;
};

} // namespace ewellix_driver

#endif // EWELLIX_DRIVER__EWELLIX_HARDWARE_INTERFACE_HPP_
