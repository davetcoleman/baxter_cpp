/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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
   Desc:   Customized the default gazebo_ros_control_plugin.cpp
*/

// Overload the default plugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

// Baxter
#include <baxter_msgs/JointCommandMode.h>


namespace baxter_gazebo_plugin
{

class BaxterGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin
{
private:
  ros::Subscriber command_mode_sub_;

public:
  void loadThread()
  {
    ROS_INFO_STREAM_NAMED("baxter_gazebo_ros_control_plugin","loading baxter specific stuff");

    command_mode_sub_ = nh_.subscribe<baxter_msgs::JointCommandMode>("/sdk/robot/command_mode_TODO",
                         1, &BaxterGazeboRosControlPlugin::modeCommandCallback, this);
  }

  void modeCommandCallback(const baxter_msgs::JointCommandModeConstPtr& msg)
  {
    ROS_DEBUG_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Switching command mode");

    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    switch(msg->mode)
    {
    case baxter_msgs::JointCommandMode::POSITION:
      start_controllers.push_back("joint_position_controller");
      stop_controllers.push_back("joint_effort_controller");
      stop_controllers.push_back("joint_velocity_controller");
      break;
    case baxter_msgs::JointCommandMode::VELOCITY:
      start_controllers.push_back("joint_velocity_controller");
      stop_controllers.push_back("joint_position_controller");
      stop_controllers.push_back("joint_effort_controller");
      break;
    case baxter_msgs::JointCommandMode::TORQUE:
      start_controllers.push_back("joint_effort_controller");
      stop_controllers.push_back("joint_position_controller");
      stop_controllers.push_back("joint_velocity_controller");
      break;
    default:
      ROS_ERROR_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Unknown command mode " << msg->mode);
      return;
    }

  /** \brief Switch multiple controllers simultaneously.
   *
   * \param start_controllers A vector of controller names to be started
   * \param stop_controllers A vector of controller names to be stopped
   * \param strictness How important it is that the requested controllers are
   * started and stopped.  The levels are defined in the
   * controller_manager_msgs/SwitchControllers service as either \c BEST_EFFORT
   * or \c STRICT.  \c BEST_EFFORT means that \ref switchController can still
   * succeed if a non-existant controller is requested to be stopped or started.
   */
    if( !controller_manager_->switchController(start_controllers,stop_controllers, 
        controller_manager_msgs::SwitchControllers::STRICT) )
    {
      ROS_ERROR_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Failed to switch controllers");
    }
    
  }


};

} // namespace
