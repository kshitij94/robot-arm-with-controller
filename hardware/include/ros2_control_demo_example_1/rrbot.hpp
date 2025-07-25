// Copyright 2020 ros2_control Development Team
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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ros2_control_demo_example_1
{
class RRBotSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

    void sendCommandToBot(const std::string& jointName);

      int openSerialPort(const char* portname);

// Function to configure the serial port
bool configureSerialPort(int fd, int speed);
// Function to read data from the serial port
int readFromSerialPort(int fd, char* buffer, size_t size);


// Function to write data to the serial port
int writeToSerialPort(int fd, const char* buffer,
                      size_t size);

// Function to close the serial port
void closeSerialPort(int fd) ;

std::string extractJointNumber(const std::string& input);

bool shouldProcess(std::string jointId, int degree);

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;
  int fd;
  int joint_2_Position_;
  int joint_5_Position_;
  double arduino_serial_port_;
  std::map<std::string, int> last_joint_command_to_arduino_;
};

}  // namespace ros2_control_demo_example_1

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_
