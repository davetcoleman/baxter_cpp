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
   Desc:   Sends commands to individual baxter joints and records results to csv file
*/

#ifndef BAXTER_CONTROL__BAXTER_TO_CSV_
#define BAXTER_CONTROL__BAXTER_TO_CSV_

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

// basic file operations
#include <iostream>
#include <fstream>

// Baxter
#include <baxter_msgs/JointPositions.h>
#include <baxter_msgs/DigitalIOState.h>
#include <baxter_msgs/JointVelocities.h>

namespace baxter_control
{

static const int RECORD_TIME = 30; // how long to run experiment
static const double STATE_EXPIRED_TIMEOUT = 1.0; // throw error when no message recieved in this many sec
static const double RECORD_RATE_HZ = 100.0; // times per second to record
static const double COMMAND_CHANGE_HZ = 20; // times per second to send new command
static const double SAFETY_LIMIT = 0.5; // the max closeness baxter will get to its hard limits
static const double SINE_SPEED = 1; // how fast teh sine wave controls the arm

/*
/robot/left_w1_velocity_controller/state/set_point
/robot/left_w1_velocity_controller/state/process_value

/robot/limb/left/command_joint_velocities/velocities[5]
/robot/limb/left/joint_states/velocity[5]
 */

class BaxterToCSV
{
private:

  // Node Handles
  ros::NodeHandle nh_; // no namespace

  // Timing
  ros::Duration control_period_;
  ros::Time last_sim_time_ros_;

  ros::Timer non_realtime_loop_;

  // Subscriber
  ros::Subscriber sub_joint_state_;
  ros::Subscriber sub_command_;

  // Buffer of joint states and commands
  sensor_msgs::JointState state_msg_;
  ros::Time state_msg_timestamp_;
  baxter_msgs::JointVelocities cmd_velocity_msg_;
  baxter_msgs::JointPositions cmd_position_msg_;


  // Which arm and joint are we testing
  std::string arm_name_;
  std::string joint_name_;
  bool position_cmd_mode_;

  // For writing CSV file
  std::string file_name_;

  // Buffer of joint state data
  std::vector<sensor_msgs::JointState> joint_states_;
  std::vector<double> joint_commands_;
  std::vector<baxter_msgs::JointPositions> cmd_position_msgs_;
  std::vector<baxter_msgs::JointVelocities> cmd_velocity_msgs_;
  std::size_t current_state_count_;
  std::size_t total_state_count_;
  std::size_t command_index_;

  // Indicate when experiment is finished
  bool first_update_;
  
public:

  /**
   * \brief Constructor
   * \param record_time how long to record state data and output commands
   */
  BaxterToCSV(bool position_cmd_mode);
  ~BaxterToCSV();

  void startRecording(const std::string& file_name);

  void stopRecording();

  void update(const ros::TimerEvent& e);

  bool writeToFile(const std::string& file_name);

  /**
   * \brief One method of sending commands to baxter
   */
  double continuousCommand(double x);
  
  /**
   * \brief Buffers joint state info from Baxter ROS topic
   * \param
   */
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  /**
   * \brief Buffers joint command info from velocity controllers
   * \param
   */
  void cmdPositionCallback(const baxter_msgs::JointPositionsConstPtr& msg);
  void cmdVelocityCallback(const baxter_msgs::JointVelocitiesConstPtr& msg);

  /**
   * \brief Checks if the state message from Baxter is out of date
   * \return true if expired
   */
  bool stateExpired();

};

} // namespace

#endif
