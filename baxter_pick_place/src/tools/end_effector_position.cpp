/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
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
 *   * Neither the name of CU Boulder nor the names of its
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

/**
 * \brief   Outputs to console the current location of the end effector
 * \author  Dave Coleman
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

#include <block_grasp_generator/robot_viz_tools.h> // simple tool for showing grasps

#include <geometry_msgs/PoseStamped.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

namespace baxter_pick_place
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_GROUP_NAME = "right_arm";
static const std::string SUPPORT_SURFACE_NAME = "workbench";
static const std::string SUPPORT_SURFACE_NAME2 = "little_table";
static const std::string BASE_LINK = "base"; //"/base";
static const std::string EE_GROUP = "right_hand";
static const std::string EE_JOINT = "right_gripper_l_finger_joint";
static const std::string EE_PARENT_LINK = "right_wrist";


class EndEffectorPosition
{
public:

  block_grasp_generator::RobotVizToolsPtr rviz_tools_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> group_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  EndEffectorPosition()
  {
    ros::NodeHandle nh;

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to rviz
    /*
      rviz_tools_.reset(new block_grasp_generator::RobotVizTools( RVIZ_MARKER_TOPIC, EE_GROUP,
      PLANNING_GROUP_NAME, BASE_LINK));
    */

    // --------------------------------------------------------------------------------------------------------
    // Enable servos
    baxter_util_.enableBaxter();

    // -------------------------------------------------------------------------------------
    // Create MoveGroup for right arm
    group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));

    geometry_msgs::PoseStamped ee_pose;

    while(ros::ok())
    {
      ee_pose = group_->getCurrentPose("right_gripper_l_finger");

      ROS_INFO_STREAM_NAMED("position",ee_pose);

      ros::Duration(4.0).sleep();
    }


    // -------------------------------------------------------------------------------------
    // Shutdown

    // Move to gravity neutral position
    baxter_util_.positionBaxterNeutral();

    // Disable servos
    baxter_util_.disableBaxter();
  }



};

} //namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "end_effector_position");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the pick place node
  baxter_pick_place::EndEffectorPosition();

  ros::shutdown();
  
  return 0;
}
