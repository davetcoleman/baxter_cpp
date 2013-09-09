/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
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

/*
  Author: Dave Coleman
  Desc:   Randomly moves the arms around
*/

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

// Visualization
#include <block_grasp_generator/robot_viz_tools.h> // simple tool for showing graspsp

#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <baxter_msgs/HeadPanCommand.h>

// Custom environment
#include "../custom_environment.h"

namespace baxter_pick_place
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_GROUP_NAME = "both_arms";
static const std::string SUPPORT_SURFACE_NAME = "workbench";
static const std::string BASE_LINK = "base"; //"/base";
static const std::string EE_GROUP = "right_hand";
static const std::string EE_JOINT = "right_endpoint";
static const std::string EE_PARENT_LINK = "right_wrist";

class RandomPlanning
{
public:

  // class for publishing stuff to rviz
  block_grasp_generator::RobotVizToolsPtr rviz_tools_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> group_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  ros::Publisher gripper_position_topic_;
  ros::Publisher gripper_release_topic_;
  ros::Publisher head_nod_topic_;
  ros::Publisher head_turn_topic_;

  RandomPlanning()
  {
    ros::NodeHandle nh;

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to rviz
    rviz_tools_.reset(new block_grasp_generator::RobotVizTools( RVIZ_MARKER_TOPIC, EE_GROUP,
        PLANNING_GROUP_NAME, BASE_LINK, FLOOR_TO_BASE_HEIGHT));

    // ---------------------------------------------------------------------------------------------
    // Create MoveGroup
    group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));
    group_->setPlanningTime(30.0);

    // --------------------------------------------------------------------------------------------------------
    // Add objects to scene
    createEnvironment(rviz_tools_);

    // --------------------------------------------------------------------------------------------------------
    // Create publishers for stuff
    ROS_DEBUG_STREAM_NAMED("random_planning","Starting close publisher");
    gripper_position_topic_ = nh.advertise<std_msgs::Float32>("/robot/limb/right/accessory/gripper/command_grip",10);

    ROS_DEBUG_STREAM_NAMED("random_planning","Starting open publisher");
    gripper_release_topic_ = nh.advertise<std_msgs::Empty>("/robot/limb/right/accessory/gripper/command_release",10);

    ROS_DEBUG_STREAM_NAMED("random_planning","Starting head nod publisher");
    head_nod_topic_ = nh.advertise<std_msgs::Bool>("/robot/head/command_head_nod",10);

    ROS_DEBUG_STREAM_NAMED("random_planning","Starting turn head publisher");
    head_turn_topic_ = nh.advertise<baxter_msgs::HeadPanCommand>("/sdk/robot/head/command_head_pan",10);

    // Wait for everything to be ready
    ros::Duration(1.0).sleep();

    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return;

    // Do it.
    startRoutine();

    // Shutdown
    baxter_util_.disableBaxter();
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

  void startRoutine()
  {

    std_msgs::Empty empty_msg;
    std_msgs::Float32 gripper_command; // for closing gripper
    gripper_command.data = 0;
    std_msgs::Bool true_command;
    true_command.data = true;
    baxter_msgs::HeadPanCommand head_command;
    head_command.speed = 60;

    // ---------------------------------------------------------------------------------------------
    // Start the demo
    while(ros::ok())
    {
      // First look around
      head_command.target = fRand(-1,-0.1);
      head_turn_topic_.publish(head_command);

      ros::Duration(0.5).sleep();
      head_command.target = fRand(0.1,1.0);
      head_turn_topic_.publish(head_command);

      ros::Duration(0.5).sleep();
      head_command.target = 0.0;
      head_turn_topic_.publish(head_command);

      do
      {
        ROS_INFO_STREAM_NAMED("random_planning","Random target...");
        group_->setRandomTarget();
        head_nod_topic_.publish(true_command);
      } while(!group_->move() && ros::ok());

      /*
      // Open grippers
      gripper_release_topic_.publish(empty_msg);

      ros::Duration(0.25).sleep();

      // Close grippers
      gripper_position_topic_.publish(gripper_command);

      ros::Duration(0.25).sleep();
      */
    }

  }

}; // end class

} // nodehandle

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_random");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the pick place node
  baxter_pick_place::RandomPlanning();

  ros::shutdown();

  return 0;
}
