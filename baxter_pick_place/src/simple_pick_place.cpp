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
 * \brief   Simple pick place for blocks using Baxter
 * \author  Dave Coleman
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

// Grasp generation
#include <block_grasp_generator/block_grasp_generator.h>
#include <block_grasp_generator/robot_viz_tools.h> // simple tool for showing graspsp

// Baxter specific properties
#include "baxter_data.h"

// Custom environment
#include "custom_environment.h"

namespace baxter_pick_place
{

static const std::string PLANNING_GROUP_NAME = "right_arm";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string BLOCK_NAME = "block1";

struct MetaBlock
{
  std::string name;
  geometry_msgs::Pose pose;
};

class SimplePickPlace
{
public:

  // grasp generator
  block_grasp_generator::BlockGraspGeneratorPtr block_grasp_generator_;

  block_grasp_generator::RobotVizToolsPtr rviz_tools_;

  // data for generating grasps
  block_grasp_generator::RobotGraspData grasp_data_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  SimplePickPlace()
  {
    ros::NodeHandle nh;

    // Create MoveGroup for right arm
    move_group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));
    move_group_->setPlanningTime(30.0);

    // Load the Robot Viz Tools for publishing to rviz
    rviz_tools_.reset(new block_grasp_generator::RobotVizTools( RVIZ_MARKER_TOPIC, EE_GROUP,
        PLANNING_GROUP_NAME, BASE_LINK, FLOOR_TO_BASE_HEIGHT));

    // Load grasp generator
    grasp_data_ = loadRobotGraspData(); // Load robot specific data

    block_grasp_generator_.reset(new block_grasp_generator::BlockGraspGenerator(rviz_tools_));

    // Let everything load
    ros::Duration(1.0).sleep();

    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return;

    // Do it.
    startRoutine();

    // Shutdown
    baxter_util_.disableBaxter();
  }

  bool startRoutine()
  {
    // Debug - calculate and output table surface dimensions
    if( false )
    {
      double y_min, y_max, x_min, x_max;
      getTableWidthRange(y_min, y_max);
      getTableDepthRange(x_min, x_max);
      ROS_INFO_STREAM_NAMED("table","Blocks width range: " << y_min << " <= y <= " << y_max);
      ROS_INFO_STREAM_NAMED("table","Blocks depth range: " << x_min << " <= x <= " << x_max);
    }

    // Create start block positions (hard coded)
    std::vector<MetaBlock> start_blocks;
    start_blocks.push_back( createStartBlock(0.85, -0.1, "Block1") );
    start_blocks.push_back( createStartBlock(0.95, -0.1, "Block2") );
    start_blocks.push_back( createStartBlock(1.05, -0.1, "Block3") );

    geometry_msgs::Pose goal_block_pose = createGoalBlock();

    // Show grasp visualizations or not
    rviz_tools_->setMuted(false);

    // Create the walls and tables
    createEnvironment(rviz_tools_);

    std::size_t block_id = 0; // \todo temp

    // --------------------------------------------------------------------------------------------------------
    // Start pick and place
    while(ros::ok())
    {
      // -------------------------------------------------------------------------------------
      // Send Baxter to neutral position
      //if( !baxter_util_.positionBaxterNeutral() )
      //  return false;

      // -------------------------------------------------------------------------------------
      // Pick block
      
      bool foundBlock = false;
      while(!foundBlock && ros::ok())
      {
        // --------------------------------------------------------------------------------------------
        // Re-add all blocks
        for (std::size_t i = 0; i < start_blocks.size(); ++i)
        {
          // Remove attached objects
          rviz_tools_->cleanupACO(start_blocks[i].name);

          // Remove collision objects
          rviz_tools_->cleanupCO(start_blocks[i].name);

          // Add a new block that is to be moved
          rviz_tools_->publishCollisionBlock(start_blocks[i].pose, start_blocks[i].name, BLOCK_SIZE);
        }

        // Publish goal block location
        rviz_tools_->publishBlock( goal_block_pose, BLOCK_SIZE, true );

        // -------------------------------------------------------------------------------------
        // Start pick
        ROS_INFO_STREAM_NAMED("pick_place","Attempting to pick '" << start_blocks[block_id].name << "'");

        if( !pick(start_blocks[block_id].pose, start_blocks[block_id].name) )
        {
          ROS_ERROR_STREAM_NAMED("simple_pick_place","Pick failed. Press any key to retry.");
          std::cin.ignore();
        }
        else
        {
          ROS_INFO_STREAM_NAMED("simple_pick_place","Done with pick ---------------------------");
          foundBlock = true;
        }
      }

      // -------------------------------------------------------------------------------------
      ROS_INFO_STREAM_NAMED("simple_pick_place","Waiting to put...");
      ros::Duration(0.5).sleep();

      if(true)
      {

        bool putBlock = false;
        while(!putBlock && ros::ok())
        {
          if( !place(goal_block_pose, start_blocks[block_id].name) )
          {
            ROS_ERROR_STREAM_NAMED("simple_pick_place","Place failed. Press any key to retry.");
            std::cin.ignore();
          }
          else
          {
            ROS_INFO_STREAM_NAMED("simple_pick_place","Done with place");
            putBlock = true;
          }
        }
      }

      ROS_INFO_STREAM_NAMED("simple_pick_place","Pick and place cycle complete ========================================= \n");
      ROS_INFO_STREAM_NAMED("simple_pick_place","Press any key to continue:");
      std::cin.ignore();

      // Go for next block or loop
      block_id++;
      if( block_id >= start_blocks.size() )
        block_id = 0;

      // Move to gravity neutral position
      //if( !baxter_util_.positionBaxterNeutral() )
      //  return false;
    }

    // Everything worked!
    return true;
  }

  MetaBlock createStartBlock(double x, double y, const std::string name)
  {
    MetaBlock start_block;
    start_block.name = name;

    // Position
    start_block.pose.position.x = x;
    start_block.pose.position.y = y;
    start_block.pose.position.z = getTableHeight(FLOOR_TO_BASE_HEIGHT);

    // Orientation
    double angle = 0; // M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_block.pose.orientation.x = quat.x();
    start_block.pose.orientation.y = quat.y();
    start_block.pose.orientation.z = quat.z();
    start_block.pose.orientation.w = quat.w();

    return start_block;
  }

  geometry_msgs::Pose createGoalBlock()
  {
    double y_min, y_max, x_min, x_max;
    getTableWidthRange(y_min, y_max);
    getTableDepthRange(x_min, x_max);

    geometry_msgs::Pose goal_block_pose;

    // Position
    goal_block_pose.position.x = x_max - TABLE_DEPTH / 2;
    goal_block_pose.position.y = y_max - TABLE_WIDTH / 2;
    goal_block_pose.position.z = getTableHeight(FLOOR_TO_BASE_HEIGHT);

    // Orientation
    double angle = 0; //M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    goal_block_pose.orientation.x = quat.x();
    goal_block_pose.orientation.y = quat.y();
    goal_block_pose.orientation.z = quat.z();
    goal_block_pose.orientation.w = quat.w();

    rviz_tools_->publishBlock( goal_block_pose, BLOCK_SIZE, true );

    return goal_block_pose;
  }

  bool pick(const geometry_msgs::Pose& block_pose, std::string block_name)
  {
    ROS_WARN_STREAM_NAMED("","picking block "<< block_name);

    std::vector<manipulation_msgs::Grasp> grasps;

    // Pick grasp
    block_grasp_generator_->generateGrasps( block_pose, grasp_data_, grasps );

    // Prevent collision with table
    move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    //ROS_WARN_STREAM_NAMED("","testing grasp 1:\n" << grasps[0]);
    //ros::Duration(100).sleep();

    //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);
    //ROS_INFO_STREAM_NAMED("","\n\n\nGrasp 10\n" << grasps[10]);

    return move_group_->pick(block_name, grasps);
    //return pickDebug(block_name, grasps);
  }

  // Step through the pick steps one by one
  bool pickDebug(std::string block_name, const std::vector<manipulation_msgs::Grasp>& grasps)
  {


  }

  bool place(const geometry_msgs::Pose& goal_block_pose, std::string block_name)
  {
    ROS_WARN_STREAM_NAMED("pick_place","Placing "<< block_name);

    std::vector<manipulation_msgs::PlaceLocation> place_locations;
    std::vector<manipulation_msgs::Grasp> grasps;

    // Re-usable datastruct
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = BASE_LINK;
    pose_stamped.header.stamp = ros::Time::now();

    // Create 360 degrees of place location rotated around a center
    for (double angle = 0; angle < 2*M_PI; angle += M_PI/4)
    {
      //    ROS_INFO_STREAM_NAMED("temp","angle = " << angle);

      pose_stamped.pose = goal_block_pose;

      // Orientation
      Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
      pose_stamped.pose.orientation.x = quat.x();
      pose_stamped.pose.orientation.y = quat.y();
      pose_stamped.pose.orientation.z = quat.z();
      pose_stamped.pose.orientation.w = quat.w();

      // Giggle block around the area
      /*
        for (double z = -0.05; z < 0.05; z += 0.05)
        {
        for (double x = -0.05; x < 0.05; x += 0.05)
        {
        for (double y = -0.05; y < 0.05; y += 0.05)
        {
      */
      // Create new place location
      manipulation_msgs::PlaceLocation place_loc;

      place_loc.place_pose = pose_stamped;

      /*
        place_loc.place_pose.pose.position.x += x;
        place_loc.place_pose.pose.position.y += y;
        place_loc.place_pose.pose.position.z += z;
      */

      //ROS_INFO_STREAM_NAMED("temp","pose:\n" << place_loc.place_pose);
      rviz_tools_->publishBlock( place_loc.place_pose.pose, BLOCK_SIZE, true );

      // Approach
      manipulation_msgs::GripperTranslation gripper_approach;
      gripper_approach.direction.header.stamp = ros::Time::now();
      gripper_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      gripper_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      gripper_approach.direction.header.frame_id = grasp_data_.base_link_;
      gripper_approach.direction.vector.x = 0;
      gripper_approach.direction.vector.y = 0;
      gripper_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
      place_loc.approach = gripper_approach;

      // Retreat
      manipulation_msgs::GripperTranslation gripper_retreat;
      gripper_retreat.direction.header.stamp = ros::Time::now();
      gripper_retreat.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      gripper_retreat.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      gripper_retreat.direction.header.frame_id = grasp_data_.base_link_;
      gripper_retreat.direction.vector.x = 0;
      gripper_retreat.direction.vector.y = 0;
      gripper_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
      place_loc.retreat = gripper_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

      place_locations.push_back(place_loc);
      /*
        }
        }
        }
      */
    }

    /*
    // Generate grasps
    block_grasp_generator_->generateGrasps( goal_block_pose, grasp_data_, grasps );

    // Convert 'grasps' to place_locations format
    for (std::size_t i = 0; i < grasps.size(); ++i)
    {
    // Create new place location
    manipulation_msgs::PlaceLocation place_loc;

    // Pose
    pose_stamped.pose = grasps[i].grasp_pose.pose;
    place_loc.place_pose = pose_stamped;

    // DEBUG \todo remove
    ROS_ERROR_STREAM_NAMED("temp","place pose " << i << "\n" << pose_stamped);

    // Publish to Rviz
    rviz_tools_->publishArrow(pose_stamped.pose);

    int counter = 0;
    while(ros::ok() && counter < 2)
    {
    sleep(1);
    counter++;
    }

    // Approach & Retreat
    place_loc.approach = grasps[i].approach;
    //ROS_WARN_STREAM_NAMED("","is the same? \n" << place_loc.approach);
    place_loc.retreat = grasps[i].retreat;

    // Post place posture - use same as pre-grasp posture (the OPEN command)
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

    //ROS_INFO_STREAM_NAMED("place location","PlaceLocation msg:\n" << place_loc);

    place_locations.push_back(place_loc);
    }
    ROS_INFO_STREAM_NAMED("pick_place","Created " << place_locations.size() << " place locations");
    */

    // Prevent collision with table
    move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    move_group_->setPlannerId("RRTConnectkConfigDefault");

    return move_group_->place(block_name, place_locations);
  }

};

} //namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the pick place node
  baxter_pick_place::SimplePickPlace();

  ros::shutdown();

  return 0;
}
