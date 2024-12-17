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
    run_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250), std::bind(&EwellixNode::run, this));
    ewellix_serial_ = new EwellixSerial("/dev/ftdi_FT61IMGY", 38400, 1000);
    ewellix_serial_->open();
    if (!ewellix_serial_->activate())
    {
      RCLCPP_INFO(this->get_logger(), "Failed to activate remote communication.");
      exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Successfully activated remote communication.");

    if (!ewellix_serial_->cycle())
    {
      RCLCPP_INFO(this->get_logger(), "Failed to cycle remote communication.");
      exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Successfully cycled remote communication.");

    if (!ewellix_serial_->setCyclicObject2())
    {
      RCLCPP_INFO(this->get_logger(), "Failed to set CyclicObject2.");
      exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Successfully set CyclicObject2");

    position_a1_ = 0;
    position_a2_ = 0;
    execute_ = false;
    moving_ = false;

    subExecute_ = this->create_subscription<std_msgs::msg::Int32>(
      "execute",
      10,
      std::bind(&EwellixNode::executeCallback, this, std::placeholders::_1)
    );

    ewellix_serial_->stopAll();
  }

  void
  EwellixNode::executeCallback(const std_msgs::msg::Int32 &msg)
  {
    position_a1_ = msg.data;
    position_a2_ = msg.data;
    execute_ = true;
  }

  void
  EwellixNode::run()
  {
    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;
    std::string elapsed;
    std::vector<uint8_t> data;

    // cycle2
    start = std::chrono::steady_clock::now();
    if(!ewellix_serial_->cycle2({position_a1_, position_a2_}, data))
    {
      RCLCPP_INFO(this->get_logger(), "Failed to cycle2 remote communication.");
      exit(1);
    }
    end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Successfully cycled remote communication.");

    EwellixSerial::Cycle2Data data_cycle2(data);

    elapsed = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count());

    RCLCPP_INFO(this->get_logger(), std::string(
      " elapsed: " + elapsed + "ms"
      " remote_a1: " + std::to_string(data_cycle2.remote_positions[0]) +
      " remote_a2: " + std::to_string(data_cycle2.remote_positions[1]) +
      " pose_a1:" + std::to_string(data_cycle2.actual_positions[0]) +
      " pose_a2:" + std::to_string(data_cycle2.actual_positions[1]) +
      " speed_a1:" + std::to_string(data_cycle2.speeds[0]) +
      " speed_a2:" + std::to_string(data_cycle2.speeds[1]) +
      " status1_a1:" + std::to_string(data_cycle2.status[0].code) +
      " status1_a2:" + std::to_string(data_cycle2.status[1].code) +
      " error1: " + std::to_string(data_cycle2.errors[0].code) +
      " error2: " + std::to_string(data_cycle2.errors[1].code) +
      " error3: " + std::to_string(data_cycle2.errors[2].code) +
      " error4: " + std::to_string(data_cycle2.errors[3].code) +
      " error5: " + std::to_string(data_cycle2.errors[4].code)
    ).c_str());

    if(moving_ &&
      data_cycle2.speeds[0] == 0 && data_cycle2.speeds[1] == 0 &&
      abs(data_cycle2.actual_positions[0] - position_a1_) < 10 &&
      abs(data_cycle2.actual_positions[1] - position_a2_) < 10)
    {
      moving_ = false;
      start = std::chrono::steady_clock::now();
      ewellix_serial_->stopAll();
      end = std::chrono::steady_clock::now();
      elapsed = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count());
      RCLCPP_INFO(this->get_logger(), "STOP! elapsed %s ms", elapsed.c_str());
    }

    if(!moving_ && execute_)
    {
      start = std::chrono::steady_clock::now();
      moving_ = ewellix_serial_->executeAllRemote();
      end = std::chrono::steady_clock::now();
      elapsed = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count());
      execute_ = false;
      RCLCPP_INFO(this->get_logger(), "EXECUTE! elapsed %s ms", elapsed.c_str());
    }

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
