/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   ros_control hardware interface layer for Baxter
*/

#include <baxter_control/baxter_hardware_interface.h>

namespace baxter_control
{

BaxterHardwareInterface::BaxterHardwareInterface(bool in_simulation)
  : in_simulation_(in_simulation)
{
  if( in_simulation_ )
  {
    ROS_INFO_STREAM_NAMED("hardware_interface","Running in simulation mode");
    right_arm_hw_.reset(new baxter_control::ArmSimulatorInterface("right"));
    left_arm_hw_.reset(new baxter_control::ArmSimulatorInterface("left"));
  }
  else
  {
    ROS_INFO_STREAM_NAMED("hardware_interface","Running in hardware mode");
    right_arm_hw_.reset(new baxter_control::ArmHardwareInterface("right"));
    left_arm_hw_.reset(new baxter_control::ArmHardwareInterface("left"));
  }
  // Initialize right arm
  right_arm_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_);
  left_arm_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_);

  ROS_ERROR_STREAM_NAMED("temp","done loading right and left arm_hw_");
  right_arm_hw_->modeSwitch(VELOCITY);  
  left_arm_hw_->modeSwitch(VELOCITY);  
  ROS_ERROR_STREAM_NAMED("temp","done mode switch");

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&pj_interface_);

  // Enable baxter
  bool enabled = false;
  while(!enabled)
  {
    if( !baxter_util_.enableBaxter() )
    {
      ROS_WARN_STREAM_NAMED("hardware_interface","Unable to enable Baxter, retrying...");
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
    else
    {
      enabled = true;
    }
  }

  // Create the controller manager
  ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

  double hz = 100;
  ros::Duration update_freq = ros::Duration(1.0/hz);
  non_realtime_loop_ = nh_.createTimer(update_freq, &BaxterHardwareInterface::update, this);

  ROS_INFO_NAMED("hardware_interface", "Loaded baxter_hardware_interface.");
}

BaxterHardwareInterface::~BaxterHardwareInterface()
{
  baxter_util_.disableBaxter();
}

void BaxterHardwareInterface::update(const ros::TimerEvent& e)
{
   // Input
  right_arm_hw_->read();
  left_arm_hw_->read();

  // Control
  controller_manager_->update(ros::Time::now(), ros::Duration(e.current_real - e.last_real) );

  // Output
  right_arm_hw_->write();
  left_arm_hw_->write();
 }

void BaxterHardwareInterface::armModeSwitch(BaxterControlMode mode)
{
  ROS_ERROR_STREAM_NAMED("temp","here armmodeswitch " << mode);
  // Pass command down to arm hardware interfaces
  //right_arm_hw_->modeSwitch(mode);
  //left_arm_hw_->modeSwitch(mode);
}

} // namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface","Starting hardware interface...");

  ros::init(argc, argv, "baxter_hardware_interface");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  bool in_simulation = false;

  // Parse command line arguments
  for (std::size_t i = 0; i < argc; ++i)
  {
    if( std::string(argv[i]).compare("--simulation") == 0 )
    {
      ROS_INFO_STREAM_NAMED("main","Gripper action server in simulation mode");
      in_simulation = true;
    }
  }  

  baxter_control::BaxterHardwareInterface baxter(in_simulation);

  ros::spin();

  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

  return 0;
}



