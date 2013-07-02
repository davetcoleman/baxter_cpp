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
#include <sensor_msgs/JointState.h>

// Baxter
//#include <baxter_msgs/BaxterGripperCommandAction.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

// Grasp generation
#include <block_grasp_generator/block_grasp_generator.h>
#include <block_grasp_generator/robot_viz_tools.h> // simple tool for showing grasps

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_GROUP_NAME = "right_arm";
static const std::string SUPPORT_SURFACE_NAME = "workbench";
static const std::string BASE_LINK = "/base";
static const std::string EE_GROUP = "right_hand";
static const std::string EE_JOINT = "right_endpoint";
static const std::string EE_PARENT_LINK = "right_wrist";
static const std::string BLOCK_NAME = "block";
static const double BLOCK_SIZE = 0.04;

// table dimensions
static const double TABLE_HEIGHT = .92;
static const double TABLE_WIDTH = .85;
static const double TABLE_DEPTH = .47;
static const double TABLE_X = 0.66;
static const double TABLE_Y = 0;
static const double TABLE_Z = -0.9/2+0.01;

// class for publishing stuff to rviz
block_grasp_generator::RobotVizToolsPtr rviz_tools_;

// grasp generator
block_grasp_generator::BlockGraspGeneratorPtr block_grasp_generator_;

// publishers
ros::Publisher pub_collision_obj_;
ros::Publisher pub_attch_collision_obj_;

// data for generating grasps
block_grasp_generator::RobotGraspData grasp_data_;

// our interface with MoveIt
boost::scoped_ptr<move_group_interface::MoveGroup> group_;

// block description
typedef std::pair<std::string,geometry_msgs::Pose> MetaBlock;


void loadRobotGraspData()
{
  // -------------------------------
  // Create pre-grasp posture
  grasp_data_.pre_grasp_posture_.header.frame_id = BASE_LINK;
  grasp_data_.pre_grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_data_.pre_grasp_posture_.name.resize(1);
  grasp_data_.pre_grasp_posture_.name[0] = EE_JOINT;
  // Position of joints
  grasp_data_.pre_grasp_posture_.position.resize(1);
  grasp_data_.pre_grasp_posture_.position[0] = 1; //baxter_msgs::BaxterGripperCommandGoal::GRIPPER_OPEN;

  // -------------------------------
  // Create grasp posture
  grasp_data_.grasp_posture_.header.frame_id = BASE_LINK;
  grasp_data_.grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_data_.grasp_posture_.name.resize(1);
  grasp_data_.grasp_posture_.name[0] = EE_JOINT;
  // Position of joints
  grasp_data_.grasp_posture_.position.resize(1);
  grasp_data_.grasp_posture_.position[0] = 0; //baxter_msgs::BaxterGripperCommandGoal::GRIPPER_CLOSE;

  // -------------------------------
  // Links
  grasp_data_.base_link_ = BASE_LINK;
  grasp_data_.ee_parent_link_ = EE_PARENT_LINK;

  // -------------------------------
  // Nums
  /* Clam
     grasp_data_.approach_retreat_desired_dist_ = 0.05;
     grasp_data_.approach_retreat_min_dist_ = 0.025;
  */
     grasp_data_.approach_retreat_desired_dist_ = 0.1;
     grasp_data_.approach_retreat_min_dist_ = 0.1;
}

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void generateRandomPose(geometry_msgs::Pose& block_pose)
{
  // Position
  block_pose.position.x = fRand(0.7,TABLE_DEPTH);
  block_pose.position.y = fRand(-TABLE_WIDTH/2,TABLE_WIDTH/2);
  block_pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 + BLOCK_SIZE / 2.0;

  // Orientation
  double angle = M_PI * fRand(0.1,1);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  block_pose.orientation.x = quat.x();
  block_pose.orientation.y = quat.y();
  block_pose.orientation.z = quat.z();
  block_pose.orientation.w = quat.w();
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

bool pick(const geometry_msgs::Pose& block_pose, std::string block_name)
{
  ROS_WARN_STREAM_NAMED("","picking block "<< block_name);

  std::vector<manipulation_msgs::Grasp> grasps;

  // Pick grasp
  block_grasp_generator_->generateGrasps( block_pose, grasp_data_, grasps );

  // Prevent collision with table
  group_->setSupportSurfaceName(SUPPORT_SURFACE_NAME);

  //ROS_WARN_STREAM_NAMED("","testing grasp 1:\n" << grasps[0]);

  //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);
  //ROS_INFO_STREAM_NAMED("","\n\n\nGrasp 10\n" << grasps[10]);

  return group_->pick(block_name, grasps);
}

bool place(const MetaBlock block)
{
  ROS_WARN_STREAM_NAMED("","placing block "<< block.first);

  std::vector<manipulation_msgs::PlaceLocation> place_locations;
  std::vector<manipulation_msgs::Grasp> grasps;

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = BASE_LINK;
  pose_stamped.header.stamp = ros::Time::now();

  // Generate grasps
  block_grasp_generator_->generateGrasps( block.second, grasp_data_, grasps );

  // Convert 'grasps' to palce_locations format
  for (std::size_t i = 0; i < grasps.size(); ++i)
  {
    // Create new place location
    manipulation_msgs::PlaceLocation place_loc;

    // Pose
    pose_stamped.pose = grasps[i].grasp_pose.pose;
    place_loc.place_pose = pose_stamped;

    // Publish to Rviz
    rviz_tools_->publishArrow(pose_stamped.pose);

    // Approach & Retreat
    place_loc.approach = grasps[i].approach;
    //ROS_WARN_STREAM_NAMED("","is the same? \n" << place_loc.approach);
    place_loc.retreat = grasps[i].retreat;

    // Post place posture - use same as pre-grasp posture (the OPEN command_
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

    place_locations.push_back(place_loc);
  }
  ROS_INFO_STREAM_NAMED("pick_place","Created " << place_locations.size() << " place locations");

  // Prevent collision with table
  group_->setSupportSurfaceName(SUPPORT_SURFACE_NAME);

  group_->setPlannerId("RRTConnectkConfigDefault");

  return group_->place(block.first, place_locations);
}

void cleanupACO()
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = BASE_LINK;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  aco.link_name = EE_PARENT_LINK;
  ros::WallDuration(0.5).sleep();
  pub_attch_collision_obj_.publish(aco);
  ros::WallDuration(0.5).sleep();
  pub_attch_collision_obj_.publish(aco);

}
void cleanupCO(std::string name)
{
  // Clean up old collision objects
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = BASE_LINK;
  co.id = name;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  ros::WallDuration(0.5).sleep();
  pub_collision_obj_.publish(co);
  ros::WallDuration(0.5).sleep();
  pub_collision_obj_.publish(co);
}

void getGoalBlocks(std::vector<MetaBlock>& block_locations)
{
  // Position
  geometry_msgs::Pose block_pose;
  block_pose.position.x = 0.3;
  block_pose.position.y = 0.2;
  block_pose.position.z = BLOCK_SIZE/2 * 0.9;

  // Orientation
  double angle = M_PI / 1.5;
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  block_pose.orientation.x = quat.x();
  block_pose.orientation.y = quat.y();
  block_pose.orientation.z = quat.z();
  block_pose.orientation.w = quat.w();

  // Block1
  MetaBlock block1 = MetaBlock("Block1", block_pose);
  block_locations.push_back(block1);

  // Block2
  block_pose.position.z = block_pose.position.z + BLOCK_SIZE + BLOCK_SIZE*0.4;  // Stack ontop
  MetaBlock block2 = MetaBlock("Block2", block_pose);
  block_locations.push_back(block2);

  // Block3
  block_pose.position.z = block_pose.position.z + BLOCK_SIZE + BLOCK_SIZE*0.4;  // Stack ontop
  MetaBlock block3 = MetaBlock("Block3", block_pose);
  block_locations.push_back(block3);

  // Block4
  block_pose.position.z = block_pose.position.z + BLOCK_SIZE + BLOCK_SIZE*0.4;  // Stack ontop
  MetaBlock block4 = MetaBlock("Block4", block_pose);
  block_locations.push_back(block3);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  pub_collision_obj_ = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_attch_collision_obj_ = nh.advertise<moveit_msgs::AttachedCollisionObject>
    ("/attached_collision_object", 10);

  ros::Duration(1.0).sleep();

  // ---------------------------------------------------------------------------------------------
  // Load the Robot Viz Tools for publishing to Rviz
  rviz_tools_.reset(new block_grasp_generator::RobotVizTools( RVIZ_MARKER_TOPIC, EE_GROUP,
      PLANNING_GROUP_NAME, BASE_LINK));

  // ---------------------------------------------------------------------------------------------
  // Load grasp generator
  loadRobotGraspData(); // Load robot specific data
  block_grasp_generator_.reset(new block_grasp_generator::BlockGraspGenerator(rviz_tools_));

  // ---------------------------------------------------------------------------------------------
  // Create MoveGroup
  group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));
  group_->setPlanningTime(30.0);

  ros::Duration(1.0).sleep();

  // --------------------------------------------------------------------------------------------------------
  // Start pick and place loop

  geometry_msgs::Pose start_block_pose;
  geometry_msgs::Pose end_block_pose;

  std::vector<MetaBlock> goal_block_locations;
  getGoalBlocks(goal_block_locations);

  // Removed attached objects
  cleanupACO();
  cleanupCO("Block1");
  cleanupCO("Block2");
  cleanupCO("Block3");
  cleanupCO("Block4");

  publishCollisionTable();

  int goal_id = 0;
  while(ros::ok())
  {
    bool foundBlock = false;
    while(!foundBlock && ros::ok())
    {
      generateRandomPose(start_block_pose);

      //static int blocker_id = 0;
      //nblocker_id ++;
      //std::string temp_block_name = "block " + boost::lexical_cast<std::string>(blocker_id);

      publishCollisionBlock(start_block_pose, goal_block_locations[goal_id].first);

      ROS_INFO_STREAM_NAMED("simple_pick_place","Published collision object here " 
        << goal_block_locations[goal_id].first);

      //MetaBlock start_block = MetaBlock(goal_block_locations[goal_id].first,start_block_pose);

      if( !pick(start_block_pose, goal_block_locations[goal_id].first) )
      {
        ROS_ERROR_STREAM_NAMED("simple_pick_place","Pick failed. Retrying.");
        cleanupCO(goal_block_locations[goal_id].first);
      }
      else
      {
        ROS_INFO_STREAM_NAMED("simple_pick_place","Done with pick");
        foundBlock = true;
      }

      ros::Duration(1.0).sleep();
    }

    bool putBlock = false;
    while(!putBlock && ros::ok())
    {
      //generateRandomPose(end_block_pose);
      end_block_pose = goal_block_locations[goal_id].second;

      if( !place(goal_block_locations[goal_id]) )
      {
        ROS_ERROR_STREAM_NAMED("simple_pick_place","Place failed. Retrying.");
      }
      else
      {
        ROS_INFO_STREAM_NAMED("simple_pick_place","Done with place");
        putBlock = true;
        ++goal_id;
        if( goal_id > goal_block_locations.size() )
        {
          ROS_WARN_STREAM_NAMED("","Out of goal locations");
          ros::shutdown();
          return 0;
        }
      }
    }

    // Cycle placed block to become pick block
    start_block_pose = end_block_pose;

  }

  ros::shutdown();
  return 0;
}

// add path constraints
/*
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  group.setPathConstraints(constr);
*/
