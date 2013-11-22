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

ArmSimulatorInterface::ArmSimulatorInterface(const std::string &arm_name, double loop_hz)
  : ArmInterface(arm_name, loop_hz)
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

  // Start arms in gravity-neutral position
  joint_position_[0] = -0.00123203;
  joint_position_[1] = 0.49262;
  joint_position_[2] = -0.272659;
  joint_position_[3] = 1.04701;
  joint_position_[4] = -0.0806423;
  joint_position_[5] = -0.0620532;
  joint_position_[6] = 0.0265941;
  joint_position_[7] = 0;
  joint_position_[8] = 0;

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_velocity_[i] = 0.0;
    joint_effort_[i] = 0.0;

    // Initial COmmands
    joint_position_command_[i] = joint_position_[i]; // set command to the gravity-neutral position
    joint_effort_command_[i] = 0.0;
    joint_velocity_command_[i] = 0.0;
  }

}

ArmSimulatorInterface::~ArmSimulatorInterface()
{
}

bool ArmSimulatorInterface::init(
  hardware_interface::JointStateInterface&    js_interface,
  hardware_interface::EffortJointInterface&   ej_interface,
  hardware_interface::VelocityJointInterface& vj_interface,
  hardware_interface::PositionJointInterface& pj_interface,
  int* joint_mode,
  sensor_msgs::JointStateConstPtr state_msg)
{
  joint_mode_ = joint_mode;

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    js_interface.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    // Create position joint interface
    pj_interface.registerHandle(hardware_interface::JointHandle(
        js_interface.getHandle(joint_names_[i]),&joint_position_command_[i]));

    // Create velocity joint interface
    vj_interface.registerHandle(hardware_interface::JointHandle(
        js_interface.getHandle(joint_names_[i]),&joint_velocity_command_[i]));

    // Create effort joint interface
    ej_interface.registerHandle(hardware_interface::JointHandle(
        js_interface.getHandle(joint_names_[i]),&joint_effort_command_[i]));
  }

  ROS_INFO_NAMED(arm_name_, "Loaded baxter_hardware_interface.");
  return true;
}

void ArmSimulatorInterface::read( sensor_msgs::JointStateConstPtr &state_msg )
{
  // Not used for visualization
}

void ArmSimulatorInterface::write(ros::Duration elapsed_time)
{
  // Convert to seconds
  elapsed_time_sec = elapsed_time.toSec();

  // Send commands to baxter in different modes

  // Move all the states to the commanded set points slowly
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    switch (*joint_mode_)
    {
      case hardware_interface::MODE_POSITION:
        // Position
        p_error_ = joint_position_command_[i] - joint_position_[i];
        // scale the rate it takes to achieve position by a factor that is invariant to the feedback loop
        joint_position_[i] += p_error_ * POSITION_STEP_FACTOR / loop_hz_;
        break;

      case hardware_interface::MODE_VELOCITY:
        // Position
        joint_position_[i] += joint_velocity_[i] * elapsed_time_sec;

        // Velocity
        v_error_ = joint_velocity_command_[i] - joint_velocity_[i];
        // scale the rate it takes to achieve velocity by a factor that is invariant to the feedback loop
        joint_velocity_[i] += v_error_ * VELOCITY_STEP_FACTOR / loop_hz_;

        /*
        if (arm_name_.compare("right") == 0 )
          ROS_INFO_STREAM_NAMED("temp",
            " Position " << joint_position_[i] <<
            " Velocity " << joint_velocity_[i] <<
            " Velocity Err " << v_error_ <<
            " Effort " << joint_effort_[i]);
        */

        break;
      case hardware_interface::MODE_EFFORT:
        break;
    }
  }


}

void ArmSimulatorInterface::robotDisabledCallback()
{
  // not implemented for simulation
}


} // namespace
