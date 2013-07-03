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
  Desc:   Provides an action server for baxter's grippers
*/

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <control_msgs/GripperCommandAction.h>

namespace baxter_gripper_controller
{

static const std::string GRIPPER_COMMAND_ACTION_TOPIC="baxter_gripper_controller";

class GripperActionServer
{
protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // Action Server
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server_;

  // Publisher
  ros::Publisher calibrate_topic_;
  ros::Publisher position_topic_;
  ros::Publisher release_topic_;

  // Action messages
  //control_msgs::GripperCommandActionGoalConstPtr action_goal_;
  control_msgs::GripperCommandActionResult action_result_;
  //    control_msgs::GripperCommandActionFeedback action_feedback_;

  std_msgs::Empty empty_msg_;

public:

  // Constructor
  GripperActionServer(const std::string name) :
    //    nh_("/"),
    action_server_(nh_, name, false)
  {

    // Start the publishers
    ROS_DEBUG_STREAM_NAMED("gripper_action_serer","Starting calibratin publisher");
    calibrate_topic_ = nh_.advertise<std_msgs::Empty>("/robot/limb/right/accessory/gripper/command_calibrate",10);
    ROS_DEBUG_STREAM_NAMED("gripper_action_serer","Starting close publisher");
    position_topic_ = nh_.advertise<std_msgs::Float32>("/robot/limb/right/accessory/gripper/command_grip",10);
    ROS_DEBUG_STREAM_NAMED("gripper_action_serer","Starting open publisher");
    release_topic_ = nh_.advertise<std_msgs::Empty>("/robot/limb/right/accessory/gripper/command_release",10);

    // Calibrate
    ROS_INFO_STREAM_NAMED("gripper_action_serer","Calibrating gripper");
    calibrate_topic_.publish(empty_msg_);
    ros::Duration(1.0).sleep();

    // Register the goal and start
    action_server_.registerGoalCallback(boost::bind(&GripperActionServer::goalCB, this));
    action_server_.start();

    // Test Run
    if(false) // todo make this a command line argument
    {
      bool open = true;
      while(ros::ok())
      {
        if(open)
        {
          openRight();
          open = false;
        }
        else
        {
          closeRight();
          open = true;
        }
        ros::Duration(2.0).sleep();
      }
    }

    // Announce state
    ROS_INFO_STREAM_NAMED("gripper_action_server", "Baxter Gripper Action Server Ready...");
  }

  // Action server sends goals here
  void goalCB()
  {
    double position = action_server_.acceptNewGoal()->command.position;

    ROS_INFO_STREAM_NAMED("gripper_action_server","Recieved goal: " << position);

    // Open command
    if(position == 1)
    {
      openRight();
    }
    else // Close command
    {
      closeRight();
    }

    // Report success
    action_result_.result.reached_goal = true;
    //action_server_.setSucceeded(action_result_,"");
    action_server_.setSucceeded();
  }

  bool openRight()
  {
    ROS_INFO_STREAM_NAMED("gripper_action_server","Opening gripper");
    release_topic_.publish(empty_msg_);
    return true;
  }

  bool closeRight()
  {
    ROS_INFO_STREAM_NAMED("gripper_action_server","Closing gripper");

    std_msgs::Float32 command;
    command.data = 0;
    position_topic_.publish(command);

    return true;
  }

  /*
    feedback_.status = "Sending goal to move_group action server";
    if(action_server_.isActive()) // Make sure we haven't sent a fake goal
    action_server_.publishFeedback(feedback_);
  */


}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_gripper_controller");

  ROS_INFO_STREAM_NAMED("gripper_action_server", "Baxter Gripper Action Server Starting");

  baxter_gripper_controller::GripperActionServer server("baxter_gripper_controller/gripper_action");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::spin(); // keep the action server alive

  return 0;
}

