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
 * \brief   Helper functions for controlling baxter
 * \author  Dave Coleman
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Msgs
#include <std_msgs/Bool.h>

namespace baxter_control
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string PLANNING_GROUP_NAME = "both_arms";
static const std::string BASE_LINK = "base"; //"/base";
static const std::string EE_GROUP = "right_hand";
static const std::string EE_JOINT = "right_gripper_l_finger_joint";
static const std::string EE_PARENT_LINK = "right_wrist";

class BaxterUtilities
{
public:

  ros::Publisher pub_baxter_enable_;

  // Interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> group_;

  BaxterUtilities()
  {
    ros::NodeHandle nh;

    // ---------------------------------------------------------------------------------------------
    // Advertise services
    pub_baxter_enable_ = nh.advertise<std_msgs::Bool>("/robot/set_super_enable",10);

    ros::Duration(0.5).sleep();
  }

  bool setPlanningGroup()
  {
    //std::string group_name = group_->getName();

    // Create MoveGroup for both arms
    group_.reset(new move_group_interface::MoveGroup("both_arms"));
  }

  bool enableBaxter()
  {
    ROS_INFO_STREAM_NAMED("utility","Enabling Baxter");
    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    pub_baxter_enable_.publish(enable_msg);
    ros::Duration(0.5).sleep();

    return true;
  }

  bool disableBaxter()
  {
    ROS_INFO_STREAM_NAMED("utility","Disabling Baxter");
    std_msgs::Bool enable_msg;
    enable_msg.data = false;
    pub_baxter_enable_.publish(enable_msg);
    ros::Duration(0.5).sleep();

    return true;
  }

  bool positionBaxterReady()
  {
    setPlanningGroup();

    // Send to ready position
    ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm ready positions...");
    group_->setNamedTarget("both_ready"); // this is defined in Baxter's SRDF
    group_->move();
    ros::Duration(1).sleep();
  }

  bool positionBaxterNeutral()
  {
    setPlanningGroup();

    // Send to neutral position
    ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm neutral positions...");
    group_->setNamedTarget("both_neutral"); // this is defined in Baxter's SRDF
    group_->move();
    ros::Duration(1).sleep();
  }

};

} //namespace

