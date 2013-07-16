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
#include <shape_tools/solid_primitive_dims.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <baxter_msgs/HeadPanCommand.h>


static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_GROUP_NAME = "both_arms";
static const std::string SUPPORT_SURFACE_NAME = "workbench";
static const std::string BASE_LINK = "base"; //"/base";
static const std::string EE_GROUP = "right_hand";
static const std::string EE_JOINT = "right_endpoint";
static const std::string EE_PARENT_LINK = "right_wrist";
static const std::string BLOCK_NAME = "block";
static const double BLOCK_SIZE = 0.04;

// table dimensions
static const double TABLE_HEIGHT = 1.0; // .92
static const double TABLE_WIDTH = .88; //.85
static const double TABLE_DEPTH = .47;
static const double TABLE_X = 0.68; //.66
static const double TABLE_Y = 0;
static const double TABLE_Z = -0.9/2+0.01;

// class for publishing stuff to rviz
//block_grasp_generator::RobotVizToolsPtr rviz_tools_;

// grasp generator
//block_grasp_generator::BlockGraspGeneratorPtr block_grasp_generator_;

// publishers
ros::Publisher pub_collision_obj_;
ros::Publisher pub_attach_collision_obj_;

// our interface with MoveIt
boost::scoped_ptr<move_group_interface::MoveGroup> group_;

// block description
//typedef std::pair<std::string,geometry_msgs::Pose> MetaBlock;

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void publishCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = BASE_LINK;
  collision_obj.id = block_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = BLOCK_SIZE;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = BLOCK_SIZE;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = BLOCK_SIZE;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = block_pose;

  pub_collision_obj_.publish(collision_obj);

  ROS_DEBUG_STREAM_NAMED("simple_pick_place","Published collision object " << block_name);
}

void publishCollisionTable()
{
  geometry_msgs::Pose table_pose;

  // Position
  table_pose.position.x = TABLE_X;
  table_pose.position.y = TABLE_Y;
  table_pose.position.z = TABLE_Z;

  // Orientation
  double angle = 0; // M_PI / 2;
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  table_pose.orientation.x = quat.x();
  table_pose.orientation.y = quat.y();
  table_pose.orientation.z = quat.z();
  table_pose.orientation.w = quat.w();

  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = BASE_LINK;
  collision_obj.id = SUPPORT_SURFACE_NAME;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

  // Size
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = TABLE_DEPTH;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = TABLE_WIDTH;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = TABLE_HEIGHT;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = table_pose;

  pub_collision_obj_.publish(collision_obj);
}

void cleanupCO(std::string name)
{
  // Clean up old collision objects
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = BASE_LINK;
  co.id = name;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  ros::WallDuration(0.1).sleep();
  pub_collision_obj_.publish(co);
  ros::WallDuration(0.1).sleep();
  pub_collision_obj_.publish(co);
}


int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  pub_collision_obj_ = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_attach_collision_obj_ = nh.advertise<moveit_msgs::AttachedCollisionObject>
    ("/attached_collision_object", 10);

  ros::Duration(1.0).sleep();

  // ---------------------------------------------------------------------------------------------
  // Create MoveGroup
  group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));
  group_->setPlanningTime(30.0);

  // --------------------------------------------------------------------------------------------------------
  // Add objects to scene
  publishCollisionTable();

  // --------------------------------------------------------------------------------------------------------
  // Create publishers for stuff

  std_msgs::Empty empty_msg;
  std_msgs::Float32 gripper_command; // for closing gripper
  gripper_command.data = 0;
  std_msgs::Bool true_command;
  true_command.data = true;
  baxter_msgs::HeadPanCommand head_command;
  head_command.speed = 60;

  ROS_DEBUG_STREAM_NAMED("random_planning","Starting close publisher");
  ros::Publisher gripper_position_topic_ = nh.advertise<std_msgs::Float32>("/robot/limb/right/accessory/gripper/command_grip",10);

  ROS_DEBUG_STREAM_NAMED("random_planning","Starting open publisher");
  ros::Publisher gripper_release_topic_ = nh.advertise<std_msgs::Empty>("/robot/limb/right/accessory/gripper/command_release",10);

  ROS_DEBUG_STREAM_NAMED("random_planning","Starting head nod publisher");
  ros::Publisher head_nod_topic_ = nh.advertise<std_msgs::Bool>("/robot/head/command_head_nod",10);

  ROS_DEBUG_STREAM_NAMED("random_planning","Starting turn head publisher");
  ros::Publisher head_turn_topic_ = nh.advertise<baxter_msgs::HeadPanCommand>("/sdk/robot/head/command_head_pan",10);

  // Wait for everything to be ready
  ros::Duration(2.0).sleep();

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

  ros::shutdown();
  return 0;
}
