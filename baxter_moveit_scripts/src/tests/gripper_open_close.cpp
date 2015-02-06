/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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

/*
  Author: Dave Coleman
  Desc:   Open and close grippers using baxter_move_group_interface
*/

#include <ros/ros.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>
#include <moveit_grasps/grasp_data.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing graspsp

// Action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>

namespace baxter_moveit_scripts
{

//static const std::string PLANNING_GROUP = "right_arm";
static const std::string PLANNING_GROUP = "both_arms";

class GripperOpenClose
{
public:

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  // robot-specific data for generating grasps
  moveit_grasps::GraspData grasp_data_;

  ros::NodeHandle nh_;

  GripperOpenClose(const std::string& arm_name)
    : nh_("~")
  {
    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to rviz
    ROS_ERROR_STREAM_NAMED("temp","Warning: i hacked the base link to be hard coded string, is likely wrong");
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link"));
    visual_tools_->setFloorToBaseHeight(-0.9);

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    if (!grasp_data_.loadRobotGraspData(nh_, arm_name + "_hand", visual_tools_->getRobotModel()))
      ros::shutdown();

    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return;

    // ---------------------------------------------------------------------------------------------
    // Load the action client
    static const std::string ACTION_TOPIC = "/robot/baxter_"+arm_name+"_gripper_action/gripper_action";
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> action_client(ACTION_TOPIC, true);
    ROS_INFO("Waiting for action server to start.");
    action_client.waitForServer(); //will wait for infinite time

    // ---------------------------------------------------------------------------------------------
    // Start test
    ROS_INFO_STREAM_NAMED("gripper_open_closed","Action server started for " << ACTION_TOPIC << ", sending gripping goal.");
    bool open = true;
    while (ros::ok())
    {
      // Send a goal to the action
      control_msgs::GripperCommandGoal goal;
      if (open)
        goal.command.position = grasp_data_.pre_grasp_posture_.points[0].positions[0];
      else
        goal.command.position = grasp_data_.grasp_posture_.points[0].positions[0];      
      open = !open;
      ROS_INFO_STREAM_NAMED("gripper_open_close","Sending command:\n" << goal);
      action_client.sendGoal(goal);

      // Wait for the action to return
      bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));

      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = action_client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
      }
      else
        ROS_ERROR("Action did not finish before the time out.");

      ros::Duration(2.0).sleep();
    }

    // Shutdown
    baxter_util_.disableBaxter();
  }


}; // end class

} // namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_random");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string arm_name = "right";
  
  if( std::string(argv[1]).compare("--arm") == 0 )
  {
    ROS_INFO_STREAM_NAMED("gripper_open_close","Testing arm " << arm_name);
    arm_name = argv[2];
  }

  // Start the pick place node
  if (arm_name == "right") // super weird bug
    baxter_moveit_scripts::GripperOpenClose("right");
  else
    baxter_moveit_scripts::GripperOpenClose("left");

  ros::shutdown();

  return 0;
}

