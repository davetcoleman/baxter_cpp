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

#ifndef BAXTER_CONTROL__ARM_SIMULATOR_INTERFACE_
#define BAXTER_CONTROL__ARM_SIMULATOR_INTERFACE_

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

// Baxter
#include <baxter_msgs/JointPositions.h>

// Parent class
#include <baxter_control/arm_interface.h>

namespace baxter_control
{

class ArmSimulatorInterface : public ArmInterface
{
private:

public:

  /**
   * \brief Constructor/Descructor
   */
  ArmSimulatorInterface(const std::string &arm_name);
  ~ArmSimulatorInterface();

  /**
   * \brief Initialice hardware interface
   * \return false if an error occurred during initialization
   */
  bool init(
    hardware_interface::JointStateInterface&    js_interface,
    hardware_interface::EffortJointInterface&   ej_interface,
    hardware_interface::VelocityJointInterface& vj_interface,
    hardware_interface::PositionJointInterface& pj_interface,
    int* joint_mode
  );

  /**
   * \brief Buffers joint state info from Baxter ROS topic
   * \param
   */
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  /**
   * \brief Checks if the state message from Baxter is out of date
   * \return true if expired
   */
  bool stateExpired();

  /**
   * \brief Copy the joint state message into our hardware interface datastructures
   */
  void read();

  /**
   * \brief Publish our hardware interface datastructures commands to Baxter hardware
   */
  void write();

  /**
   * \brief This is called when Baxter is disabled, so that we can update the desired positions
   */
  void robotDisabledCallback();

};

} // namespace

#endif
