/*!********************************************************************************
 * \brief     move_at_speed implementation
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef MOVE_AT_SPEED_CONTROL_H
#define MOVE_AT_SPEED_CONTROL_H

// System
#include <string>
#include "math.h"
#include "stdlib.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <fstream>
// ROS
#include "std_srvs/Empty.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>

// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/FlightActionCommand.h>
#include "aerostack_msgs/FlightState.h"
#include <geometry_msgs/TwistStamped.h>
// Aerostack libraries
#include <behavior_execution_controller.h>
#include <yaml-cpp/yaml.h>


namespace quadrotor_motion_with_platform_control
{
class BehaviorMoveAtSpeedWithPlatformControl : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorMoveAtSpeedWithPlatformControl();
  ~BehaviorMoveAtSpeedWithPlatformControl();

private:
  // Congfig variables
  std::string flight_action_str;
  std::string motion_reference_speed_str;
  std::string status_str;

  ros::NodeHandle node_handle;
  std::string nspace; 

  // Subscriber
  ros::Subscriber status_sub;
  //Publishers
  ros::Publisher flight_action_pub;
  ros::Publisher motion_reference_speed_pub;
  
  // Messages
  aerostack_msgs::FlightState status_msg;
  aerostack_msgs::FlightActionCommand flight_action_command;
  geometry_msgs::TwistStamped motion_reference_speed;

private:
  // BehaviorExecutionController
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  // Callbacks
  void statusCallBack(const aerostack_msgs::FlightState &msg);
};
}

#endif
