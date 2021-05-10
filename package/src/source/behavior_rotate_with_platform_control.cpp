/*!********************************************************************************
 * \brief     rotate implementation
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

#include "../include/behavior_rotate_with_platform_control.h"

namespace quadrotor_motion_with_platform_control
{
BehaviorRotateWithPlatformControl::BehaviorRotateWithPlatformControl() : BehaviorExecutionController() { 
  setName("rotate_with_platform_control"); 
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorRotateWithPlatformControl::~BehaviorRotateWithPlatformControl() {}

void BehaviorRotateWithPlatformControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();

  ros::param::get("~imu_topic", imu_str);
  ros::param::get("~flight_action_topic", flight_action_str);
  ros::param::get("~motion_reference_pose_topic", motion_reference_pose_str);
  ros::param::get("~flight_state_topic", status_str);

  //Subscribers
  imu_sub = node_handle.subscribe("/" + nspace + "/"+imu_str, 1, &BehaviorRotateWithPlatformControl::imuCallBack, this);
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorRotateWithPlatformControl::statusCallBack, this);
  started = false;
}

bool BehaviorRotateWithPlatformControl::checkSituation()
{
  if (status_msg.state != aerostack_msgs::FlightState::LANDED){
    return true;
  }else{
    setErrorMessage("Error: Drone is landed");
    return false;
  }
}

void BehaviorRotateWithPlatformControl::checkGoal(){ 
  if (started){
      current_angle = 0;
      angle2 = 0;
      if(!(imu_msg.orientation.w == 0 && imu_msg.orientation.x == 0 && imu_msg.orientation.y == 0 && imu_msg.orientation.z == 0)){
          current_angle = atan2(2.0 * (imu_msg.orientation.z * imu_msg.orientation.w + imu_msg.orientation.x * imu_msg.orientation.y) , 
                              - 1.0 + 2.0 * (imu_msg.orientation.w * imu_msg.orientation.w + imu_msg.orientation.x * imu_msg.orientation.x));    
          angle2 = atan2(2.0 * (reference_pose.pose.orientation.z * reference_pose.pose.orientation.w + reference_pose.pose.orientation.x * reference_pose.pose.orientation.y) , 
                              - 1.0 + 2.0 * (reference_pose.pose.orientation.w * reference_pose.pose.orientation.w + reference_pose.pose.orientation.x * reference_pose.pose.orientation.x));    
      } 
      if (abs(abs(angle2) - abs(current_angle)) < 0.5) BehaviorExecutionController::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  }
}

void BehaviorRotateWithPlatformControl::checkProgress() {
}

void BehaviorRotateWithPlatformControl::checkProcesses() 
{ 
 
}


void BehaviorRotateWithPlatformControl::onActivate()
{
  //Publishers
  flight_action_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+flight_action_str, 1, true);
  motion_reference_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/"+motion_reference_pose_str, 1,true);

  reference_pose.pose.position.x = 0; reference_pose.pose.position.y = 0;reference_pose.pose.position.z = 0;
  // Extract target yaw
  std::string arguments=getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["relative_angle"].IsDefined())
  {
    angle=config_file["relative_angle"].as<double>() * M_PI/180;
    q_rot.setRPY(0, 0, angle);
    reference_pose.pose.orientation.w = q_rot.getW();
    reference_pose.pose.orientation.x = q_rot.getX();
    reference_pose.pose.orientation.y = q_rot.getY();
    reference_pose.pose.orientation.z = q_rot.getZ();
  }
  else
  {
    if(config_file["angle"].IsDefined())
    {
      angle=config_file["angle"].as<double>() * M_PI/180;
      if(!(imu_msg.orientation.w == 0 && imu_msg.orientation.x == 0 && imu_msg.orientation.y == 0 && imu_msg.orientation.z == 0)){
          angle = angle - atan2(2.0 * (imu_msg.orientation.z * imu_msg.orientation.w + imu_msg.orientation.x * imu_msg.orientation.y) , 
                              - 1.0 + 2.0 * (imu_msg.orientation.w * imu_msg.orientation.w + imu_msg.orientation.x * imu_msg.orientation.x));    
      }
      if(angle > 2* M_PI || angle < -2*M_PI)
      {
        angle=angle*180/M_PI;
        angle= fmod(angle,360);
        angle=angle*M_PI/180;
      }
      q_rot.setRPY(0, 0, angle);
      reference_pose.pose.orientation.w = q_rot.getW();
      reference_pose.pose.orientation.x = q_rot.getX();
      reference_pose.pose.orientation.y = q_rot.getY();
      reference_pose.pose.orientation.z = q_rot.getZ();
    }
  }
  motion_reference_pose_pub.publish(reference_pose);
  flight_action_command.action = aerostack_msgs::FlightActionCommand::MOVE;
  flight_action_pub.publish(flight_action_command);

  started = true;
}

void BehaviorRotateWithPlatformControl::onDeactivate()
{
  started = false; 
  flight_action_pub.shutdown();
  motion_reference_pose_pub.shutdown();
}

void BehaviorRotateWithPlatformControl::onExecute()
{
}

void BehaviorRotateWithPlatformControl::imuCallBack(const sensor_msgs::Imu &msg){
  imu_msg = msg;
}
void BehaviorRotateWithPlatformControl::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}

}
PLUGINLIB_EXPORT_CLASS(quadrotor_motion_with_platform_control::BehaviorRotateWithPlatformControl, nodelet::Nodelet)
