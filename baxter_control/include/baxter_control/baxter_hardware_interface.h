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

#ifndef BAXTER_CONTROL__BAXTER_HARDWARE_INTERFACE_
#define BAXTER_CONTROL__BAXTER_HARDWARE_INTERFACE_

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Baxter
#include <baxter_control/baxter_utilities.h>
#include <baxter_control/arm_interface.h>
#include <baxter_control/arm_hardware_interface.h>
#include <baxter_control/arm_simulator_interface.h>

namespace baxter_control
{

class BaxterHardwareInterface : public hardware_interface::RobotHW
{
private:

  // Node Handles
  ros::NodeHandle nh_; // no namespace

  // Timing
  ros::Duration control_period_;
  ros::Time last_sim_time_ros_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::PositionJointInterface pj_interface_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  // sub-hardware interfaces
  ArmInterfacePtr right_arm_hw_;
  ArmInterfacePtr left_arm_hw_;

  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Timer non_realtime_loop_;

  bool in_simulation_;

public:

  /**
   * \brief Constructor/Descructor
   */
  BaxterHardwareInterface(bool in_simulation);
  ~BaxterHardwareInterface();

  void update(const ros::TimerEvent& e);

  /**
   * \brief Call to switch the arm hardware between different interfaces - position or velocity
   * \param mode - which mode to call
   */
  void armModeSwitch(BaxterControlMode mode);

};

} // namespace

#endif
