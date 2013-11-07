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

#ifndef BAXTER_CONTROL__ARM_INTERFACE_
#define BAXTER_CONTROL__ARM_INTERFACE_

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_mode_interface.h>

// Baxter
#include <baxter_msgs/JointPositions.h>
#include <baxter_msgs/JointVelocities.h>

namespace baxter_control
{

enum BaxterControlMode { POSITION, VELOCITY, TORQUE };

class ArmInterface
{
protected:

  // Node Handles
  ros::NodeHandle nh_; // no namespace

  // Number of joints we are using
  unsigned int n_dof_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_velocity_command_;
  
  // Track current hardware interface mode we are in
  int* joint_mode_;

  // Name of this arm
  std::string arm_name_;

public:

  /**
   * \brief Constructor/Descructor
   */
  ArmInterface(const std::string &arm_name)
    : arm_name_(arm_name)
  {};

  ~ArmInterface()
  {};

  /**
   * \brief Initialice hardware interface
   * \return false if an error occurred during initialization
   */
  virtual bool init(
    hardware_interface::JointStateInterface&    js_interface,
    hardware_interface::EffortJointInterface&   ej_interface,
    hardware_interface::VelocityJointInterface& vj_interface,
    hardware_interface::PositionJointInterface& pj_interface,
    int* joint_mode
  ) 
  { return true; };

  /**
   * \brief Copy the joint state message into our hardware interface datastructures
   */
  virtual void read()
  {};

  /**
   * \brief Publish our hardware interface datastructures commands to Baxter hardware
   */
  virtual void write()
  {};

};

typedef boost::shared_ptr<baxter_control::ArmInterface> ArmInterfacePtr;
typedef boost::shared_ptr<const baxter_control::ArmInterface> ArmInterfaceConstPtr;

} // namespace

#endif
