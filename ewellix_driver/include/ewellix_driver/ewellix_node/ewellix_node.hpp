/**
 *
 *  \file       ewellix_node.hpp
 *  \brief      Ewellix ROS 2 node
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

#ifndef EWELLIX_DRIVER__EWELLIX_NODE_HPP_
#define EWELLIX_DRIVER__EWELLIX_NODE_HPP_

#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ewellix_driver/ewellix_serial/ewellix_serial.hpp"

namespace ewellix_driver
{

class EwellixNode
: public rclcpp::Node
{
public:
  EwellixNode(const std::string node_name);

  void run();
  void executeCallback(const std_msgs::msg::Int32 &msg);

private:
  rclcpp::TimerBase::SharedPtr run_timer_;
  EwellixSerial * ewellix_serial_;
  int position_a1_;
  int position_a2_;
  bool moving_;
  bool execute_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subExecute_;
};

} // namespace ewellix_driver

#endif // EWELLIX_DRIVER__EWELLIX_NODE_HPP_
