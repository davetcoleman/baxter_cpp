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
  : in_simulation_(in_simulation),
    joint_mode_(1)
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

  // Set the joint mode interface data
  jm_interface_.registerHandle(hardware_interface::JointModeHandle("joint_mode", &joint_mode_));

  // Start the shared joint state subscriber
  sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1,
                     &BaxterHardwareInterface::stateCallback, this);

  // Wait for first state message to be recieved if we are not in simulation
  if (!in_simulation_)
  {
    // Loop until we find a joint_state message from Baxter
    do
    {
      // Loop until we get our first joint_state message
      while(ros::ok() && state_msg_timestamp_.toSec() == 0)
      {
        ROS_INFO_STREAM_NAMED("hardware_interface","Waiting for first state message to be recieved");
        ros::spinOnce();
        ros::Duration(0.25).sleep();
      }
    } while (state_msg_->name.size() != NUM_BAXTER_JOINTS);
  }

  // Initialize right arm
  right_arm_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_, &joint_mode_, state_msg_);
  left_arm_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_, &joint_mode_, state_msg_);

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&jm_interface_);
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

  // Set callback for Baxter being disabled
  baxter_util_.setDisabledCallback(boost::bind( &ArmInterface::robotDisabledCallback, right_arm_hw_ ));
  baxter_util_.setDisabledCallback(boost::bind( &ArmInterface::robotDisabledCallback,  left_arm_hw_ ));

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
  //baxter_util_.disableBaxter();
}

bool BaxterHardwareInterface::stateExpired()
{
  // Check that we have a non-expired state message
  // \todo lower the expiration duration
  if( ros::Time::now() > state_msg_timestamp_ + ros::Duration(STATE_EXPIRED_TIMEOUT)) // check that the message timestamp is no older than 1 second
  {

    ROS_WARN_STREAM_THROTTLE_NAMED(1,"hardware_interface","State expired. Last recieved state " << (ros::Time::now() - state_msg_timestamp_).toSec() << " seconds ago." );
    return true;
  }
  return false;
}

void BaxterHardwareInterface::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  // Check if this message has the correct number of joints
  if( msg->name.size() != NUM_BAXTER_JOINTS )
  {
    //ROS_WARN_STREAM_NAMED("temp","unrecognized joint state message: " << *msg);
    return;
  }

  // Copy the latest message into a buffer
  state_msg_ = msg;
  state_msg_timestamp_ = ros::Time::now();
}

void BaxterHardwareInterface::update(const ros::TimerEvent& e)
{
  // Check if state msg from Baxter is expired
  if( stateExpired() )
    return;

  // Input
  right_arm_hw_->read(state_msg_);
  left_arm_hw_->read(state_msg_);

  // Control
  controller_manager_->update(ros::Time::now(), ros::Duration(e.current_real - e.last_real) );

  // Output
  right_arm_hw_->write();
  left_arm_hw_->write();
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
      ROS_INFO_STREAM_NAMED("main","Baxter Hardware Interface in simulation mode");
      in_simulation = true;
    }
  }

  baxter_control::BaxterHardwareInterface baxter(in_simulation);

  ros::spin();

  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

  return 0;
}



