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

#include "ros2_control_demo_example_1/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std;

namespace ros2_control_demo_example_1
{

  int RRBotSystemPositionOnlyHardware::openSerialPort(const char* portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        cerr << "Error opening " << portname << ": "
             << strerror(errno) << endl;
        return -1;
    }
    return fd;
}

// Function to configure the serial port
bool RRBotSystemPositionOnlyHardware::configureSerialPort(int fd, int speed)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        cerr << "Error from tcgetattr: " << strerror(errno)
             << endl;
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag
        = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no
                     // canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF
                     | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag
        |= (CLOCAL | CREAD); // ignore modem controls,
                             // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "Error from tcsetattr: " << strerror(errno)
             << endl;
        return false;
    }
    return true;
}

// Function to read data from the serial port
int RRBotSystemPositionOnlyHardware::readFromSerialPort(int fd, char* buffer, size_t size)
{
    return ::read(fd, buffer, size);
}

// Function to write data to the serial port
int RRBotSystemPositionOnlyHardware::writeToSerialPort(int fd, const char* buffer,
                      size_t size)
{
    return ::write(fd, buffer, size);
}

// Function to close the serial port
void RRBotSystemPositionOnlyHardware::closeSerialPort(int fd) { close(fd); }


hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

    const char* portname = "/dev/ttyUSB0";
  hw_slowdown_ = RRBotSystemPositionOnlyHardware::openSerialPort(portname);
  if (hw_slowdown_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open port : %s", portname);
  }

  RCLCPP_INFO(get_logger(), "Robot hardware_component update_rate is %dHz", info_.rw_rate);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  if (!configureSerialPort(hw_slowdown_, B9600)) {
      RCLCPP_FATAL(get_logger(), "Failed to configureSerialPort : B9600");
  }

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  RRBotSystemPositionOnlyHardware::closeSerialPort(hw_slowdown_);

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }


  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";

  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    // Simulate RRBot's movement
    auto new_value = get_state(name) + (get_command(name) - get_state(name)) / 100;
    set_state(name, new_value);
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_state(name) << " for joint '" << name << "'";
  }
      char buffer[100];

  // END: This part here is for exemplary purposes - Please do not copy to your production code
    int n = readFromSerialPort(hw_slowdown_, buffer, sizeof(buffer));
    if (n < 0) {
        std::stringstream ss;
        ss << "errro while reading from serial port:"
             << strerror(errno) << endl;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    }     else {
        ss << "Read from serial port: "
             << std::string(buffer, n) << endl;
    }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  if (hw_slowdown_ < 0) {
      return hardware_interface::return_type::OK;
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    std::stringstream ss;
    ss << "Writing commands:";
    auto jointPositionInRadian = get_command(name);
    int degrees = jointPositionInRadian * (180.0 / M_PI); 
    std::string jointid = "2";
    if (name.find("joint2") != std::string::npos) {
      if (joint_2_Position_==degrees) {
        ss << "Ignoring duplicate command for joint2";
        continue;
      }
      joint_2_Position_=degrees;
    } else {
      if (joint_5_Position_==degrees) {
        ss << "Ignoring duplicate command for joint2";
        continue;
      }
      jointid="5";
      joint_5_Position_=degrees;
    }
    std::string temp_message = "<";
    temp_message = temp_message + std::to_string(degrees) + "," + jointid+">";
    const char* message = temp_message.c_str();
    RRBotSystemPositionOnlyHardware::writeToSerialPort(hw_slowdown_, message, strlen(message));
    ss << temp_message;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  }
  return hardware_interface::return_type::OK;
}

void RRBotSystemPositionOnlyHardware::sendCommandToBot(const std::string& jointName) {
  
    auto jointPositionInRadian = get_command(jointName);
    std::stringstream tmp;
    int degrees = jointPositionInRadian * (180.0 / M_PI); 
    auto jointNameTmp = jointName;    
    jointNameTmp.erase(jointNameTmp.find("/position"), std::string("/position").length());
  
    tmp << std::fixed << std::setprecision(2) << std::endl
       << "\t Joint " << jointNameTmp << " target position is in radians is "<< jointPositionInRadian
       << " in degree is "<<degrees;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", tmp.str().c_str());
    
    std::string temp_message = "<" + jointNameTmp + " "+std::to_string(degrees) + ">";

// Then, get a const char* from the std::string using .c_str()
    const char* message = temp_message.c_str();


  RRBotSystemPositionOnlyHardware::writeToSerialPort(hw_slowdown_, message, strlen(message));
}

}  // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_1::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)
