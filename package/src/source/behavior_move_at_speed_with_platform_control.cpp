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

#include "../include/behavior_move_at_speed_with_platform_control.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorMoveAtSpeedWithPlatformControl behavior;
  behavior.start();
  return 0;
}

BehaviorMoveAtSpeedWithPlatformControl::BehaviorMoveAtSpeedWithPlatformControl() : BehaviorExecutionManager() { 
  setName("move_at_speed_with_platform_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorMoveAtSpeedWithPlatformControl::~BehaviorMoveAtSpeedWithPlatformControl() {}

void BehaviorMoveAtSpeedWithPlatformControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();  

  ros::param::get("~flight_action_topic", flight_action_str);
  ros::param::get("~motion_reference_speed_topic", motion_reference_speed_str);
  ros::param::get("~flight_state_topic", status_str);

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorMoveAtSpeedWithPlatformControl::statusCallBack, this);
}

bool BehaviorMoveAtSpeedWithPlatformControl::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.state != aerostack_msgs::FlightState::LANDED){
    return true;
  }else{
    setErrorMessage("Error: Drone is landed");
    return false;
  }
}

void BehaviorMoveAtSpeedWithPlatformControl::checkGoal(){}


void BehaviorMoveAtSpeedWithPlatformControl::checkProgress() {
  if (status_msg.state == aerostack_msgs::FlightState::LANDED){
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
}

void BehaviorMoveAtSpeedWithPlatformControl::checkProcesses() 
{ 
 
}

void BehaviorMoveAtSpeedWithPlatformControl::onExecute() 
{ 
 
}

void BehaviorMoveAtSpeedWithPlatformControl::onActivate()
{
  //Publishers
  flight_action_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+flight_action_str, 1, true);
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/"+motion_reference_speed_str,1, true);

  //Get arguments
  std::string arguments=getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  
  std::string direction;

  if(config_file["direction"].IsDefined()){
    direction = config_file["direction"].as<std::string>();
  }else{
    ROS_ERROR("Behavior move at speed was called without a direction");
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::INTERRUPTED);
    return;
  }

  double speed;

  if(config_file["speed"].IsDefined()){
    speed = config_file["speed"].as<double>();
  }else{
    speed = 1; //Default value
  }

  if(direction == "LEFT")
  {
    motion_reference_speed.twist.linear.x=-speed;
    motion_reference_speed.twist.linear.y=0;
    motion_reference_speed.twist.linear.z=0;

  }
  else if(direction == "RIGHT")
  {

    motion_reference_speed.twist.linear.x=speed;
    motion_reference_speed.twist.linear.y=0;
    motion_reference_speed.twist.linear.z=0;

  }
  else if(direction == "FORWARD")
  {

    motion_reference_speed.twist.linear.x=0;
    motion_reference_speed.twist.linear.y=speed;
    motion_reference_speed.twist.linear.z=0;
  }
  else if(direction == "BACKWARD")
  {

    motion_reference_speed.twist.linear.x=0;
    motion_reference_speed.twist.linear.y=-speed;
    motion_reference_speed.twist.linear.z=0;
  }
  motion_reference_speed.header.stamp = ros::Time::now();
  motion_reference_speed_pub.publish(motion_reference_speed);
  flight_action_command.header.stamp = ros::Time::now();
  flight_action_command.action = aerostack_msgs::FlightActionCommand::MOVE;
  flight_action_pub.publish(flight_action_command); 
}

void BehaviorMoveAtSpeedWithPlatformControl::onDeactivate()
{
  aerostack_msgs::FlightActionCommand msg;
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  flight_action_pub.publish(msg);

  flight_action_pub.shutdown();
  motion_reference_speed_pub.shutdown();
}

void BehaviorMoveAtSpeedWithPlatformControl::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}
