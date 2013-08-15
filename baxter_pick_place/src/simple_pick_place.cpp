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

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

// Grasp generation
#include <block_grasp_generator/block_grasp_generator.h>
#include <block_grasp_generator/robot_viz_tools.h> // simple tool for showing grasps

// Baxter
#include <baxter_msgs/GripperState.h>

namespace baxter_pick_place
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_GROUP_NAME = "right_arm";
static const std::string SUPPORT_SURFACE_NAME = "workbench";
static const std::string SUPPORT_SURFACE_NAME2 = "little_table";
static const std::string BASE_LINK = "base"; //"/base";
static const std::string EE_GROUP = "right_hand";
//static const std::string EE_JOINT = "right_endpoint";
static const std::string EE_JOINT = "right_gripper_l_finger_joint";
static const std::string EE_PARENT_LINK = "right_wrist";
static const std::string BLOCK_NAME = "block1";
static const double BLOCK_SIZE = 0.04;

// table dimensions
static const double TABLE_HEIGHT = 1.0; // .92
static const double TABLE_WIDTH = .85;
static const double TABLE_DEPTH = .47;
static const double TABLE_X = 0.68; //.66
static const double TABLE_Y = 0;
static const double TABLE_Z = -0.9/2+0.01;

class SimplePickPlace
{
public:

  // grasp generator
  block_grasp_generator::BlockGraspGeneratorPtr block_grasp_generator_;

  block_grasp_generator::RobotVizToolsPtr rviz_tools_;

  // publishers
  ros::Publisher pub_collision_obj_;
  ros::Publisher pub_attach_collision_obj_;

  // data for generating grasps
  block_grasp_generator::RobotGraspData grasp_data_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> group_;

  SimplePickPlace()
  {
    ros::NodeHandle nh;
    pub_collision_obj_ = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    pub_attach_collision_obj_ = nh.advertise<moveit_msgs::AttachedCollisionObject>
      ("/attached_collision_object", 10);

    ros::Duration(1.0).sleep();

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    ROS_INFO_STREAM_NAMED("temp","Loading robot viz tools");
    // class for publishing stuff to rviz
    rviz_tools_.reset(new block_grasp_generator::RobotVizTools( RVIZ_MARKER_TOPIC, EE_GROUP,
        PLANNING_GROUP_NAME, BASE_LINK));

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    loadRobotGraspData(); // Load robot specific data
    block_grasp_generator_.reset(new block_grasp_generator::BlockGraspGenerator(rviz_tools_));

    // --------------------------------------------------------------------------------------------------------
    // Start pick and place loop

    geometry_msgs::Pose start_block_pose;
    geometry_msgs::Pose goal_block_pose;

    // --------------------------------------------------------------------------------------------------------
    // Create start block

    // Position
    start_block_pose.position.x = 0.0;
    start_block_pose.position.y = -0.6; // -0.55;
    start_block_pose.position.z = -0.5;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_block_pose.orientation.x = quat1.x();
    start_block_pose.orientation.y = quat1.y();
    start_block_pose.orientation.z = quat1.z();
    start_block_pose.orientation.w = quat1.w();

    // --------------------------------------------------------------------------------------------------------
    // Create goal block

    // Position
    goal_block_pose.position.x = 0.6; // table depth
    goal_block_pose.position.y = -TABLE_WIDTH/2 + 0.2; // table width
    goal_block_pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 + BLOCK_SIZE / 2.0; // table height

    // Orientation
    angle = 0; //M_PI / 1.5;
    Eigen::Quaterniond quat2(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    goal_block_pose.orientation.x = quat2.x();
    goal_block_pose.orientation.y = quat2.y();
    goal_block_pose.orientation.z = quat2.z();
    goal_block_pose.orientation.w = quat2.w();

    // temp \todo
    rviz_tools_->setMuted(true);

    // --------------------------------------------------------------------------------------------------------
    // Start pick and place

    while(ros::ok())
    {
      // --------------------------------------------------------------------------------------------------------
      // Remove attached objects
      cleanupACO(BLOCK_NAME);

      // Remove collision objects
      cleanupCO(BLOCK_NAME);
      cleanupCO(SUPPORT_SURFACE_NAME);
      cleanupCO(SUPPORT_SURFACE_NAME2);

      // -------------------------------------------------------------------------------------
      // Create MoveGroup for both arms
      if(true)
      {
        group_.reset(new move_group_interface::MoveGroup("both_arms"));
        group_->setPlanningTime(30.0);

        ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm neutral position...");
        group_->setNamedTarget("both_neutral"); // this is defined in Baxter's SRDF
        group_->move();
        ros::Duration(1).sleep();
      }

      // --------------------------------------------------------------------------------------------------------
      // Add objects to scene
      //publishCollisionTable();
      publishCollisionTableSmall();

      // Publish goal block location
      rviz_tools_->publishBlock( goal_block_pose, BLOCK_SIZE, true );

      // Add a new block that is to be moved
      publishCollisionBlock(start_block_pose, BLOCK_NAME);

      // -------------------------------------------------------------------------------------
      // Create MoveGroup for right arm
      group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));
      group_->setPlanningTime(30.0);

      // Do pick operation?
      if(true)
      {
        bool foundBlock = false;
        while(!foundBlock && ros::ok())
        {
          //publishCollisionBlock(start_block_pose, BLOCK_NAME);

          if( !pick(start_block_pose, BLOCK_NAME) )
          {
            ROS_ERROR_STREAM_NAMED("simple_pick_place","Pick failed. Retrying.");
            //cleanupCO(BLOCK_NAME);
          }
          else
          {
            ROS_INFO_STREAM_NAMED("simple_pick_place","Done with pick");
            foundBlock = true;
          }

          ros::Duration(2.0).sleep();
        }

        ROS_INFO_STREAM_NAMED("simple_pick_place","Found block!\n\n\n\n\n\n\nWaiting to put...");
        ros::Duration(1.0).sleep();

      }
      else
      {
        // Fake the pick operation by just attaching the collision object
        attachCO(BLOCK_NAME);
      }

      bool putBlock = false;
      while(!putBlock && ros::ok())
      {
        if( !place(goal_block_pose, BLOCK_NAME) )
        {
          ROS_ERROR_STREAM_NAMED("simple_pick_place","Place failed.");
          putBlock = true; // \todo remove this for demo, is wrong
        }
        else
        {
          ROS_INFO_STREAM_NAMED("simple_pick_place","Done with place");
          putBlock = true;
        }

        ros::Duration(2.0).sleep();
      }

      ROS_INFO_STREAM_NAMED("simple_pick_place","Cycle completed!\n\n\n\n\n\n\nWaiting to restart...");
      ros::Duration(1.0).sleep();

      ROS_ERROR_STREAM_NAMED("temp","restart disabled");
      break; // \todo remove for demo
    }

  }

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
    grasp_data_.pre_grasp_posture_.position[0] = baxter_msgs::GripperState::POSITION_OPEN;

    // -------------------------------
    // Create grasp posture
    grasp_data_.grasp_posture_.header.frame_id = BASE_LINK;
    grasp_data_.grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    grasp_data_.grasp_posture_.name.resize(1);
    grasp_data_.grasp_posture_.name[0] = EE_JOINT;
    // Position of joints
    grasp_data_.grasp_posture_.position.resize(1);
    grasp_data_.grasp_posture_.position[0] = baxter_msgs::GripperState::POSITION_CLOSED;

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
    grasp_data_.approach_retreat_min_dist_ = 0.001;


    grasp_data_.grasp_depth_ = 0.1; //15; // default 0.12

    grasp_data_.block_size_ = 0.04;

    // Debug
    block_grasp_generator::BlockGraspGenerator::printBlockGraspData(grasp_data_);
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

  void generateRandomPose(geometry_msgs::Pose& block_pose)
  {
    // Position
    /*
      block_pose.position.x = fRand(0.7,TABLE_DEPTH);
      block_pose.position.y = fRand(-TABLE_WIDTH/2,TABLE_WIDTH/2);
      block_pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 + BLOCK_SIZE / 2.0;
    */

    // Position
    block_pose.position.x = fRand(0.7,TABLE_DEPTH);
    block_pose.position.y = fRand(-TABLE_WIDTH/2,-0.1);
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

    //ROS_INFO_STREAM_NAMED("pick_place","CollisionObject: \n " << collision_obj);

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

  void publishCollisionTableSmall()
  {
    geometry_msgs::Pose table_pose;

    // Position
    table_pose.position.x = 0.0;
    table_pose.position.y = -0.55;
    table_pose.position.z = -0.68;

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
    collision_obj.id = SUPPORT_SURFACE_NAME2;
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    collision_obj.primitives.resize(1);
    collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

    // Size
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = .3;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = .3;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = .3;

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
    group_->setSupportSurfaceName(SUPPORT_SURFACE_NAME2);

    //ROS_WARN_STREAM_NAMED("","testing grasp 1:\n" << grasps[0]);
    //ros::Duration(100).sleep();

    //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);
    //ROS_INFO_STREAM_NAMED("","\n\n\nGrasp 10\n" << grasps[10]);

    return group_->pick(block_name, grasps);
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
      for (double z = -0.05; z < 0.05; z += 0.05)
      {
        for (double x = -0.05; x < 0.05; x += 0.05)
        {
          for (double y = -0.05; y < 0.05; y += 0.05)
          {
            // Create new place location
            manipulation_msgs::PlaceLocation place_loc;

            place_loc.place_pose = pose_stamped;

            place_loc.place_pose.pose.position.x += x;
            place_loc.place_pose.pose.position.y += y;
            place_loc.place_pose.pose.position.z += z;

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

          }
        }
      }
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
    group_->setSupportSurfaceName(SUPPORT_SURFACE_NAME);

    group_->setPlannerId("RRTConnectkConfigDefault");

    return group_->place(block_name, place_locations);
  }

  void cleanupACO(const std::string& name)
  {
    // Clean up old attached collision object
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.header.stamp = ros::Time::now();
    aco.object.header.frame_id = BASE_LINK;

    //aco.object.id = name;
    aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

    aco.link_name = EE_PARENT_LINK;

    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);
    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);

  }
  void attachCO(const std::string& name)
  {
    // Clean up old attached collision object
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.header.stamp = ros::Time::now();
    aco.object.header.frame_id = BASE_LINK;

    aco.object.id = name;
    aco.object.operation = moveit_msgs::CollisionObject::ADD;

    // Link to attach the object to
    aco.link_name = EE_PARENT_LINK;

    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);
    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);

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
