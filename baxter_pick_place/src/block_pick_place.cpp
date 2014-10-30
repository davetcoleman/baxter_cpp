/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CU Boulder
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

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps

// Baxter specific properties
#include <moveit_simple_grasps/grasp_data.h>
#include <baxter_pick_place/custom_environment5.h>

namespace baxter_pick_place
{

struct MetaBlock
{
  std::string name;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
};

class SimplePickPlace
{
public:

  // A shared node handle
  ros::NodeHandle nh_;

  // grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  // which baxter arm are we using
  std::string arm_;
  std::string planning_group_name_;

  // settings
  bool auto_reset_;
  int auto_reset_sec_;
  int pick_place_count_; // tracks how many pick_places have run 

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  SimplePickPlace(bool verbose)
    : nh_("~"),
      verbose_(verbose),
      auto_reset_(false),
      auto_reset_sec_(4),
      arm_("right"),
      planning_group_name_(arm_+"_arm"),
      pick_place_count_(0)
  {

    // Create MoveGroup for one of the planning groups
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(30.0);

    // Load grasp generator
    if (!grasp_data_.loadRobotGraspData(nh_, arm_+"_hand"))
      ros::shutdown();

    // Temp modifications
    ROS_WARN_STREAM_NAMED("temp","temp modifications, move this maybe");
    {
      grasp_data_.grasp_depth_ = 0.0;
      //grasp_data_.approach_retreat_min_dist_ =
      //  grasp_data_.approach_retreat_desired_dist_; // = 0.6;
      std::cout << "grasp_data_.approach_retreat_desired_dist_ " << grasp_data_.approach_retreat_desired_dist_ << std::endl;
    }

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools( grasp_data_.base_link_));
    visual_tools_->setFloorToBaseHeight(-0.9);
    visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);
    visual_tools_->deleteAllMarkers();

    // Load Grasp generator
    simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

    // Let everything load
    ros::Duration(1.0).sleep();

    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return;

    // Do it.
    startRoutine();

    // Shutdown
    //baxter_util_.disableBaxter();
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
    std::vector<MetaBlock> blocks;
    double block_y = 0.4;
    double block_x = 0.64;
    // Flip the side of the table the blocks are on depending on which arm we are using
    if( arm_.compare("right") == 0 )
      block_y *= -1;
    blocks.push_back( createStartBlock(block_x,       block_y, "Block1") );
    blocks.push_back( createStartBlock(block_x+0.1,   block_y, "Block2") );
    blocks.push_back( createStartBlock(block_x+0.2,   block_y, "Block3") );

    // The goal for each block is simply translating them on the y axis
    for (std::size_t i = 0; i < blocks.size(); ++i)
    {
      blocks[i].goal_pose = blocks[i].start_pose;
      blocks[i].goal_pose.position.y += 0.1;
    }

    // Show grasp visualizations or not
    visual_tools_->setMuted(false);

    // Create the walls and tables
    createEnvironment(visual_tools_);

    // --------------------------------------------------------------------------------------------------------
    // Repeat pick and place forever
    while(ros::ok())
    {
      // -------------------------------------------------------------------------------------
      // Send Baxter to neutral position
      //if( !baxter_util_.positionBaxterNeutral() )
      //  return false;

      // --------------------------------------------------------------------------------------------
      // Re-add all blocks
      for (std::size_t i = 0; i < blocks.size(); ++i)
      {
        resetBlock(blocks[i]);
      }

      // Place on left side, then back on right side
      for (std::size_t side = 0; side < 2; ++side)
      {

        // Do for all blocks
        for (std::size_t block_id = 0; block_id < blocks.size(); ++block_id)
        {
          // Pick -------------------------------------------------------------------------------------
          while(ros::ok())
          {
            ROS_INFO_STREAM_NAMED("pick_place","Picking '" << blocks[block_id].name << "'");

            // Visualize the block we are about to pick
            if (verbose_)
            {
              visual_tools_->deleteAllMarkers();
              visual_tools_->publishBlock( blocks[block_id].start_pose, rviz_visual_tools::BLUE, BLOCK_SIZE);
            }

            if( !pick(blocks[block_id].start_pose, blocks[block_id].name) )
            {
              ROS_ERROR_STREAM_NAMED("pick_place","Pick failed.");

              // Ask user if we should try again
              if( !promptUser() )
                return false;

              // Retry
              resetBlock(blocks[block_id]);
            }
            else
            {
              ROS_INFO_STREAM_NAMED("pick_place","Done with pick ---------------------------");
              break;
            }
          }
          
          // Place -------------------------------------------------------------------------------------
          while(ros::ok())
          {
            ROS_INFO_STREAM_NAMED("pick_place","Placing '" << blocks[block_id].name << "'");

            // Publish goal block location
            if (verbose_)
            {
              visual_tools_->deleteAllMarkers();
              visual_tools_->publishBlock( blocks[block_id].goal_pose, rviz_visual_tools::BLUE, BLOCK_SIZE);
            }

            if( !place(blocks[block_id].goal_pose, blocks[block_id].name) )
            {
              ROS_ERROR_STREAM_NAMED("pick_place","Place failed.");

              // Determine if the attached collision body as already been removed, in which case
              // we can ignore the failure and just resume picking
              /*
                if( !move_group_->hasAttachedObject(blocks[block_id].name) )
                {
                ROS_WARN_STREAM_NAMED("pick_place","Collision object already detached, so auto resuming pick place.");

                // Ask user if we should try again
                if( !promptUser() )
                break; // resume picking
                }
              */

              // Ask user if we should try again
              if( !promptUser() )
                return false;
            }
            else
            {
              ROS_INFO_STREAM_NAMED("pick_place","Done with place ----------------------------");
              break;
            }
          } // place retry loop

          // Swap this block's start and end pose so that we can then move them back to position
          geometry_msgs::Pose temp = blocks[block_id].start_pose;
          blocks[block_id].start_pose = blocks[block_id].goal_pose;
          blocks[block_id].goal_pose = temp;

          pick_place_count_++;

        } // loop through 3 blocks

        ROS_INFO_STREAM_NAMED("pick_place","Finished picking and placing " << blocks.size() 
          << " blocks. Total pick_places: " << pick_place_count_);

      } // place on both sides of table

      // Ask user if we should repeat
      //if( !promptUser() )
      //  break;
      ros::Duration(1.0).sleep();

    }

    // Move to gravity neutral position
    //if( !baxter_util_.positionBaxterNeutral() )
    //  return false;

    // Everything worked!
    return true;
  }

  void resetBlock(MetaBlock block)
  {
    // Remove attached object
    visual_tools_->cleanupACO(block.name);

    // Remove collision object
    visual_tools_->cleanupCO(block.name);

    // Add the collision block
    visual_tools_->publishCollisionBlock(block.start_pose, block.name, BLOCK_SIZE);
  }

  MetaBlock createStartBlock(double x, double y, const std::string name)
  {
    MetaBlock start_block;
    start_block.name = name;

    // Position
    start_block.start_pose.position.x = x;
    start_block.start_pose.position.y = y;
    start_block.start_pose.position.z = getTableHeight(-0.9);

    // Orientation
    double angle = 0; // M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_block.start_pose.orientation.x = quat.x();
    start_block.start_pose.orientation.y = quat.y();
    start_block.start_pose.orientation.z = quat.z();
    start_block.start_pose.orientation.w = quat.w();

    return start_block;
  }

  bool pick(const geometry_msgs::Pose& block_pose, std::string block_name)
  {
    std::vector<moveit_msgs::Grasp> grasps;

    // Pick grasp
    simple_grasps_->generateBlockGrasps( block_pose, grasp_data_, grasps );
    if (verbose_)
    {
      double speed = 0.012;
      visual_tools_->publishGrasps( grasps, grasp_data_.ee_parent_link_, speed);
      visual_tools_->deleteAllMarkers();
    }

    // Prevent collision with table
    move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    // Allow blocks to be touched by end effector
    {
      // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
      std::vector<std::string> allowed_touch_objects;
      allowed_touch_objects.push_back("Block1");
      allowed_touch_objects.push_back("Block2");
      allowed_touch_objects.push_back("Block3");
      allowed_touch_objects.push_back("Block4");

      // Add this list to all grasps
      for (std::size_t i = 0; i < grasps.size(); ++i)
      {
        grasps[i].allowed_touch_objects = allowed_touch_objects;
      }
    }

    //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);

    return move_group_->pick(block_name, grasps);
  }

  bool place(const geometry_msgs::Pose& goal_block_pose, std::string block_name)
  {
    ROS_WARN_STREAM_NAMED("place","Placing '"<< block_name << "'");

    std::vector<moveit_msgs::PlaceLocation> place_locations;
    std::vector<moveit_msgs::Grasp> grasps;

    // Re-usable datastruct
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = grasp_data_.base_link_;
    pose_stamped.header.stamp = ros::Time::now();

    // Create 360 degrees of place location rotated around a center
    for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
    {
      pose_stamped.pose = goal_block_pose;

      // Orientation
      Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
      pose_stamped.pose.orientation.x = quat.x();
      pose_stamped.pose.orientation.y = quat.y();
      pose_stamped.pose.orientation.z = quat.z();
      pose_stamped.pose.orientation.w = quat.w();

      // Create new place location
      moveit_msgs::PlaceLocation place_loc;

      place_loc.place_pose = pose_stamped;

      if (verbose_)
        visual_tools_->publishBlock( place_loc.place_pose.pose, rviz_visual_tools::BLUE, BLOCK_SIZE);

      // Approach
      moveit_msgs::GripperTranslation pre_place_approach;
      pre_place_approach.direction.header.stamp = ros::Time::now();
      pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
      place_loc.pre_place_approach = pre_place_approach;

      // Retreat
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      post_place_retreat.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      post_place_retreat.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      post_place_retreat.direction.header.frame_id = grasp_data_.base_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
      place_loc.post_place_retreat = post_place_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

      place_locations.push_back(place_loc);
    }

    // Prevent collision with table
    move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    move_group_->setPlannerId("RRTConnectkConfigDefault");

    return move_group_->place(block_name, place_locations);
  }

  bool promptUser()
  {
    // Make sure ROS is still with us
    if( !ros::ok() )
      return false;

    if( auto_reset_ )
    {
      ROS_INFO_STREAM_NAMED("pick_place","Auto-retrying in " << auto_reset_sec_ << " seconds");
      ros::Duration(auto_reset_sec_).sleep();
    }
    else
    {
      ROS_INFO_STREAM_NAMED("pick_place","Retry? (y/n)");
      char input; // used for prompting yes/no
      std::cin >> input;
      if( input == 'n' )
        return false;
    }
    return true;
  }

};

} //namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  // Start the pick place node
  baxter_pick_place::SimplePickPlace server(verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
