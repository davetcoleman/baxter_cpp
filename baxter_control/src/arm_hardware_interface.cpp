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
   Desc:   ros_control hardware interface for a Baxter arm
*/

#include <baxter_control/arm_hardware_interface.h>

namespace baxter_control
{

ArmHardwareInterface::ArmHardwareInterface(const std::string &arm_name)
  : arm_name_(arm_name),
    state_msg_(new sensor_msgs::JointState())
{
  // Populate joints in this arm
  joint_names_.push_back(arm_name_+"_e0");
  joint_names_.push_back(arm_name_+"_e1");
  joint_names_.push_back(arm_name_+"_s0");
  joint_names_.push_back(arm_name_+"_s1");
  joint_names_.push_back(arm_name_+"_w0");
  joint_names_.push_back(arm_name_+"_w1");
  joint_names_.push_back(arm_name_+"_w2");
  n_dof_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  output_command_msg_.angles.resize(n_dof_);
  output_command_msg_.names.resize(n_dof_);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = 0.0;
    joint_velocity_[i] = 0.0;
    joint_effort_[i] = 0.0;
    joint_position_command_[i] = 0.0;
    joint_effort_command_[i] = 0.0;
    joint_velocity_command_[i] = 0.0;
  }
}

ArmHardwareInterface::~ArmHardwareInterface()
{
}

bool ArmHardwareInterface::init(
  hardware_interface::JointStateInterface&    js_interface,
  hardware_interface::EffortJointInterface&   ej_interface,
  hardware_interface::VelocityJointInterface& vj_interface,
  hardware_interface::PositionJointInterface& pj_interface)
{
  for (std::size_t i = 0; i < n_dof_; ++i)
  {

    // Debug
    ROS_DEBUG_STREAM_NAMED(arm_name_,"Loading joint '" << joint_names_[i]
      << "' of type '" << HARDWARE_INTERFACE << "'");

    // Create joint state interface for all joints
    js_interface.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    // Create position joint interface
    pj_interface.registerHandle(hardware_interface::JointHandle(
        js_interface.getHandle(joint_names_[i]),&joint_position_command_[i]));
  }

  // Start publishers and subscribers
  pub_position_command_ = nh_.advertise<baxter_msgs::JointPositions>("/robot/limb/"+arm_name_+
                          "/command_joint_angles",10);

  sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/robot/limb/" + arm_name_ +
                     "/joint_states", 1, &ArmHardwareInterface::stateCallback, this);

  // Wait for first state message to be recieved
  while(ros::ok() && state_msg_timestamp_.toSec() == 0)
  {
    ROS_INFO_STREAM_NAMED(arm_name_,"Waiting for first state message to be recieved");
    ros::spinOnce();
    ros::Duration(0.25).sleep();
  }

  // Make sure Rethink hasn't changed anything with the ordering in which joints are published in
  // joint state message
  if( state_msg_->name[0].compare("e0") != 0 || state_msg_->name[1].compare("e1") != 0 ||
    state_msg_->name[2].compare("s0") != 0 || state_msg_->name[3].compare("s1") != 0 ||
    state_msg_->name[4].compare("w0") != 0 || state_msg_->name[5].compare("w1") != 0 ||
    state_msg_->name[6].compare("w2") != 0)
  {
    ROS_ERROR_STREAM_NAMED(arm_name_,"It seems the joint state message for arm " << arm_name_ <<
      " has changed its ordering. The order has been hard coded into the arm_hardware_interface and "
      << "now it needs to be fixed");
    return false;
  }

  // Set the initial command values based on current state
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_command_[i] = state_msg_->position[i];
    ROS_DEBUG_STREAM_NAMED("temp","set joint " << joint_names_[i] << " to position " << joint_position_command_[i]);

    // Pre-load the joint names into the output messages just once
    output_command_msg_.names[i] = joint_names_[i];

    ROS_DEBUG_STREAM_NAMED("temp","done here too");
  }

  ROS_INFO_NAMED(arm_name_, "Loaded baxter_hardware_interface.");
  return true;
}

void ArmHardwareInterface::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  // Copy the latest message into a buffer
  state_msg_ = msg;
  state_msg_timestamp_ = ros::Time::now();
}

/*
  void ArmHardwareInterface::loadJointStateNameMap()
  {
  ROS_DEBUG_STREAM_NAMED(arm_name_,"Loading joint state map using state msg:\n"
  << *state_msg_);

  int controlled_joints_count = 0;
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
  // Find the name in the state message that corresponds to the transmission index
  std::vector<std::string>::const_iterator name_it;
  name_it = std::find(state_msg_->name.begin(), state_msg_->name.end(), joint_names_[i]);

  // Error check
  if( name_it == state_msg_->name.end() )
  {
  ROS_DEBUG_STREAM_NAMED(arm_name_,"No mapping for '" <<
  joint_names_[i] << "' in Baxter state message");
  }ppp
  else
  {
  // We will use this in our published command message - keep track of size of vector
  controlled_joints_count++;
  }

  // Copy index of found joint name
  joint_interface_to_joint_state_[i] = name_it - state_msg_->name.begin() - 1;
  }

  std::copy(joint_interface_to_joint_state_.begin(), joint_interface_to_joint_state_.end(),
  std::ostream_iterator<double>(std::cout, "\n"));

  // Resize the output publisher message to match input message
  output_command_msg_.angles.resize(controlled_joints_count);
  output_command_msg_.names.resize(controlled_joints_count);

  ROS_DEBUG_STREAM_NAMED(arm_name_,"Matched " << controlled_joints_count <<
  " joints from state message and URDF transmissions");

  // \temp
  sleep(2);
  }
*/

bool ArmHardwareInterface::stateExpired()
{
  // Check that we have a non-expired state message
  // \todo lower the expiration duration
  if( ros::Time::now() > state_msg_timestamp_ + ros::Duration(STATE_EXPIRED_TIMEOUT)) // check that the message timestamp is no older than 1 second
  {
    ROS_WARN_STREAM_NAMED(arm_name_,"State expired. \n" << ros::Time::now() << "\n" <<
      state_msg_timestamp_ + ros::Duration(STATE_EXPIRED_TIMEOUT) << "\n" << " State: \n" << *state_msg_ );
    return true;
  }
  return false;
}

void ArmHardwareInterface::read()
{
  if( stateExpired() )
    return;

  // Copy state message to our datastructures
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    //ROS_INFO_STREAM_NAMED("read","on index " << i << " which maps to " << i);
    //ROS_INFO_STREAM_NAMED("read","and the max message is " << state_msg_->position.size() );
    //ROS_INFO_STREAM_NAMED("read","joint_position size is " << joint_position_.size());
    //ROS_INFO_STREAM_NAMED("read","state_msg position size is " << state_msg_->position.size());
    //ROS_INFO_STREAM_NAMED("read","position is " << state_msg_->position[i]);

    joint_position_[i] = state_msg_->position[i];
    joint_velocity_[i] = state_msg_->velocity[i];
    joint_effort_[i] = state_msg_->effort[i];
  }
}

void ArmHardwareInterface::write()
{
  if( stateExpired() )
    return;


  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    //ROS_INFO_STREAM_NAMED("write","id = "<<i);
    //ROS_INFO_STREAM_NAMED("write","name = " << joint_names_[i]);
    //ROS_INFO_STREAM_NAMED("write","mapping = " << i);

    output_command_msg_.angles[i] = joint_position_command_[i];
  }

  pub_position_command_.publish(output_command_msg_);
}

} // namespace
