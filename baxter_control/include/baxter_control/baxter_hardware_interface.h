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

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// baxter
#include <baxter_msgs/JointPositions.h>

#include <sensor_msgs/JointState.h>

namespace baxter_control
{

class BaxterHardwareInterface : public hardware_interface::RobotHW
{
private:

  // Node Handles
  ros::NodeHandle nh_; // no namespace
  ros::NodeHandle model_nh_; // namespaces to robot name

  // Strings
  std::string robot_namespace_;
  std::string robot_description_;

  // Transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // Timing
  ros::Duration control_period_;
  ros::Time last_sim_time_ros_;

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::PositionJointInterface pj_interface_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_velocity_command_;

  // Publishers
  ros::Publisher pub_position_command_;
  baxter_msgs::JointPositions output_msg_;

  // Subscriber
  ros::Subscriber sub_joint_state_;

  // Buffer of joint states
  sensor_msgs::JointStateConstPtr state_msg_;

  std::vector<int> joint_interface_to_joint_state_;
  bool has_joint_interface_to_joint_state_;

public:

  BaxterHardwareInterface();
  ~BaxterHardwareInterface();

  /**
   * \brief Initialice hardware interface
   * \return false if an error occurred during initialization
   */
  bool init();
  
  /**
   * \brief Buffers joint state info from Baxter ROS topic
   * \param 
   */
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  /**
   * \brief At startup, maps the baxter message joint_names vector into our URDF-parsed vector
   */
  void loadJointStateNameMap();

  /**
   * \brief Copy the joint state message into our hardware interface datastructures
   */
  void read();

  /**
   * \brief Publish our hardware interface datastructures commands to Baxter hardware
   */
  void write();


  // Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const;

  // Get Transmissions from the URDF
  bool parseTransmissionsFromURDF();

};


}
