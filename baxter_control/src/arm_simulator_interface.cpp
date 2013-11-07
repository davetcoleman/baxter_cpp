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
   Desc:   ros_control hardware interface layer for Baxter in simulation
*/

#include <baxter_control/arm_simulator_interface.h>

namespace baxter_control
{

ArmSimulatorInterface::ArmSimulatorInterface(const std::string &arm_name)
  : ArmInterface(arm_name)
{
  // Populate joints in this arm
  joint_names_.push_back(arm_name_+"_e0");
  joint_names_.push_back(arm_name_+"_e1");
  joint_names_.push_back(arm_name_+"_s0");
  joint_names_.push_back(arm_name_+"_s1");
  joint_names_.push_back(arm_name_+"_w0");
  joint_names_.push_back(arm_name_+"_w1");
  joint_names_.push_back(arm_name_+"_w2");
  joint_names_.push_back(arm_name_+"_gripper_l_finger_joint");
  joint_names_.push_back(arm_name_+"_gripper_r_finger_joint");
  // hack for head_pan
  if(arm_name_.compare("right"))
      joint_names_.push_back("head_pan");

  n_dof_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

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

ArmSimulatorInterface::~ArmSimulatorInterface()
{
}

bool ArmSimulatorInterface::init(
  hardware_interface::JointStateInterface&    js_interface,
  hardware_interface::JointModeInterface&     jm_interface,
  hardware_interface::EffortJointInterface&   ej_interface,
  hardware_interface::VelocityJointInterface& vj_interface,
  hardware_interface::PositionJointInterface& pj_interface)
{
  for (std::size_t i = 0; i < n_dof_; ++i)
  {

    // Create joint state interface for all joints
    js_interface.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    // Create position joint interface
    pj_interface.registerHandle(hardware_interface::JointHandle(
        js_interface.getHandle(joint_names_[i]),&joint_position_command_[i]));
  }

  // Set the initial command values to 0
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_command_[i] = 0;

    joint_position_[i] = 0;
    joint_velocity_[i] = 0;
    joint_effort_[i] = 0;
  }

  ROS_INFO_NAMED(arm_name_, "Loaded baxter_hardware_interface.");
  return true;
}

void ArmSimulatorInterface::read()
{
  /*
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = joint_position_
    joint_velocity_[i] = 0;
    joint_effort_[i] = 0;
  }
  */
}

void ArmSimulatorInterface::write()
{
  double error;
  
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    error = joint_position_command_[i] - joint_position_[i];
    //ROS_DEBUG_STREAM_NAMED("temp","error " << error << " command="<<joint_position_command_[i] 
    //  << " joint position=" <<  joint_position_[i]);
    joint_position_[i] += error * 0.1;
  }
}

} // namespace
