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

#include <baxter_control/baxter_utilities.h>

namespace baxter_control
{

BaxterUtilities::BaxterUtilities()
  : disabled_callback_called_(false),
    state_counter_(1)
{
  ROS_INFO_STREAM_NAMED("baxter_utilities","Loading Baxter utilities");
  ros::NodeHandle nh;

  // Preload Messages
  disable_msg_.data = false;
  enable_msg_.data = true;

  // ---------------------------------------------------------------------------------------------
  // Advertise services
  pub_baxter_enable_ = nh.advertise<std_msgs::Bool>("/robot/set_super_enable",10);
  pub_baxter_reset_ = nh.advertise<std_msgs::Empty>("/robot/set_super_reset",10);
  ROS_DEBUG_STREAM_NAMED("baxter_utilities","Publishing on topics /robot/set_super_enable and robot/set_super_reset");

  // ---------------------------------------------------------------------------------------------
  // Start the state subscriber
  sub_baxter_state_ = nh.subscribe<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC,
                      1, &BaxterUtilities::stateCallback, this);

  // ---------------------------------------------------------------------------------------------
  // Shoulder enable disable buttons
  sub_shoulder_left_ = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_shoulder_button/state",
                       1, &BaxterUtilities::leftShoulderCallback, this);
  sub_shoulder_right_ = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/right_shoulder_button/state",
                        1, &BaxterUtilities::rightShoulderCallback, this);
  ROS_DEBUG_STREAM_NAMED("baxter_utilities","Subscribing to baxter_core_msgs");
}

void BaxterUtilities::setDisabledCallback(DisabledCallback callback)
{
  disabled_callback_ = callback;
}

bool BaxterUtilities::communicationActive()
{
  int count = 0;
  while( ros::ok() && baxter_state_timestamp_.toSec() == 0 )
  {
    if( count > 40 ) // 40 is an arbitrary number for when to assume no state is being published
    {
      ROS_WARN_STREAM_NAMED("utilities","No state message has been recieved on topic "
        << BAXTER_STATE_TOPIC);
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  // Check that the message timestamp is no older than 1 second
  if(ros::Time::now() > baxter_state_timestamp_ + ros::Duration(1.0))
  {
    ROS_ERROR_STREAM_NAMED("utilities","Baxter state expired.");
    return false;
  }

  return true;
}

bool BaxterUtilities::isEnabled(bool verbose)
{
  // Check communication
  if( !communicationActive() )
  {
    // Error message aready outputed
    return false;
  }

  // Check for estop
  if( baxter_state_->stopped == true )
  {
    // Skip the switch statments if we are not wanting verbose output
    if(!verbose)
      return false;

    std::string estop_button;
    switch( baxter_state_->estop_button )
    {
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED:
        estop_button = "Robot is not stopped and button is not pressed";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_PRESSED:
        estop_button = "Pressed";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNKNOWN:
        estop_button = "STATE_UNKNOWN when estop was asserted by a non-user source";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_RELEASED:
        estop_button = "Was pressed, is now known to be released, but robot is still stopped.";
        break;
      default:
        estop_button = "Unkown button state code";
    }

    std::string estop_source;
    switch( baxter_state_->estop_source )
    {
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE:
        estop_source = "e-stop is not asserted";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_USER:
        estop_source = "e-stop source is user input (the red button)";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN:
        estop_source = "e-stop source is unknown";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_FAULT:
        estop_source = "MotorController asserted e-stop in response to a joint fault";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_BRAIN:
        estop_source = "MotorController asserted e-stop in response to a lapse of the brain heartbeat";
        break;
      default:
        estop_source = "Unkown button source code";

    }

    ROS_ERROR_STREAM_NAMED("utilities","ESTOP Button State: '" << estop_button << "'. Source: '" << estop_source << "'");
    return false;
  }

  // Check for error
  if( baxter_state_->error == true )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","Baxter has an error :(  State: \n" << *baxter_state_ );
    return false;
  }

  // Check enabled
  if( baxter_state_->enabled == false )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","Baxter is not enabled.  State: \n" << *baxter_state_ );

    return false;
  }

  return true;
}

void BaxterUtilities::stateCallback(const baxter_core_msgs::AssemblyStateConstPtr& msg)
{
  baxter_state_ = msg;
  baxter_state_timestamp_ = ros::Time::now();

  // Check for errors every CHECK_FREQ refreshes to save computation
  static const int CHECK_FREQ = 25; //50;
  if( state_counter_ % CHECK_FREQ == 0 )
  {
    if( !isEnabled() ) // baxter is disabled
    {
      if (disabled_callback_called_ == false)
      {
        // Call the parent classes' callback if they've provided one
        if (disabled_callback_)
          disabled_callback_();

        disabled_callback_called_ = true;
      }
    }
    else // baxter is not disabled
    {
      disabled_callback_called_ = false;
    }

    // Reset the counter so it doesn't overflow
    state_counter_ = 0;
  }

  state_counter_++;
}

void BaxterUtilities::leftShoulderCallback(const baxter_core_msgs::DigitalIOStateConstPtr& msg)
{
  if (msg->state == 0)
  {
    enableBaxter();
  }
}

void BaxterUtilities::rightShoulderCallback(const baxter_core_msgs::DigitalIOStateConstPtr& msg)
{
  if (msg->state == 0)
  {
    disableBaxter();
  }
}

bool BaxterUtilities::enableBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Enabling Baxter");

  // Check if we need to do anything
  if( isEnabled(false) )
    return true;

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  // Reset Baxter
  if( !resetBaxter() )
    return false;

  // Attempt to enable baxter
  pub_baxter_enable_.publish(enable_msg_);
  ros::Duration(0.5).sleep();

  // Check if enabled
  int count = 0;
  while( ros::ok() && !isEnabled(true) )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Giving up on waiting");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  return true;
}

bool BaxterUtilities::disableBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Disabling Baxter");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  pub_baxter_enable_.publish(disable_msg_);
  ros::Duration(0.5).sleep();

  // Check it enabled
  int count = 0;
  while( ros::ok() && baxter_state_->enabled == true )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Failed to disable Baxter");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  return true;
}

bool BaxterUtilities::resetBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Resetting Baxter");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  // Attempt to reset and enable robot
  pub_baxter_reset_.publish(empty_msg_);
  ros::Duration(0.5).sleep();

  return true;
}

bool BaxterUtilities::positionBaxterReady()
{
  // Send to ready position
  ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm ready positions...");

  return sendToPose("both_ready");
}

bool BaxterUtilities::positionBaxterNeutral()
{
  // Send to neutral position
  ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm neutral positions...");

  return sendToPose(NEUTRAL_POSE_NAME);
}

bool BaxterUtilities::sendToPose(const std::string &pose_name)
{
  ROS_INFO_STREAM_NAMED("baxter_utilities","Sending to pose '" << pose_name << "'");

  // Check if move group has been loaded yet
  // We only load it here so that applications that don't need this aspect of baxter_utilities
  // don't have to load it every time.
  if( !move_group_both_ )
  {
    move_group_both_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_BOTH_NAME));
  }

  move_group_both_->setNamedTarget(pose_name);
  move_group_both_->setPlanningTime(15);
  bool result = move_group_both_->move();

  if( !result )
    ROS_ERROR_STREAM_NAMED("utilities","Failed to send Baxter to pose '" << pose_name << "'");

  return result;
}

bool BaxterUtilities::sendToPose(const geometry_msgs::Pose& pose, const std::string &group_name,
  moveit_visual_tools::VisualToolsPtr visual_tools, const moveit_msgs::PlanningScene &planning_scene_diff)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = pose;
  return sendToPose(pose_stamped, group_name, visual_tools, planning_scene_diff);
}

bool BaxterUtilities::sendToPose(const geometry_msgs::PoseStamped& pose, const std::string &group_name,
  moveit_visual_tools::VisualToolsPtr visual_tools, const moveit_msgs::PlanningScene &planning_scene_diff)
{
  geometry_msgs::PoseStamped goal_pose = pose; // make readable copy

  // -----------------------------------------------------------------------------------------------
  // Connect to move_group action server
  if (!movegroup_action_)
  {
    ROS_DEBUG_STREAM_NAMED("baxter_utilities","sendToPose - load move group action");
    movegroup_action_.reset(new actionlib::SimpleActionClient
      <moveit_msgs::MoveGroupAction>("move_group", true));
    while(!movegroup_action_->waitForServer(ros::Duration(2.0)))
      ROS_WARN_STREAM_NAMED("sendToPose","Waiting for the move_group action server");
  }

  // -----------------------------------------------------------------------------------------------
  // Create move_group goal
  moveit_msgs::MoveGroupGoal goal;
  goal.request.group_name = group_name;
  goal.request.num_planning_attempts = 1;
  goal.request.allowed_planning_time = 5.0;

  // Hack to disable collisions with octomap
  goal.planning_options.planning_scene_diff = planning_scene_diff;

  // -------------------------------------------------------------------------------------------
  // Create goal state
  goal_pose.header.frame_id = visual_tools->getBaseLink();
  double tolerance_position = 1e-3; // default: 1e-3... meters
  double tolerance_angle = 1e-2; // default 1e-2... radians

  moveit_msgs::Constraints goal_constraint0 = kinematic_constraints::constructGoalConstraints(
    visual_tools->getEEParentLink(), goal_pose, tolerance_position, tolerance_angle);

  //ROS_INFO_STREAM_NAMED("verticle_test","Goal pose " << goal_pose);

  // Create offset constraint
  goal_constraint0.position_constraints[0].target_point_offset.x = 0.0;
  goal_constraint0.position_constraints[0].target_point_offset.y = 0.0;
  goal_constraint0.position_constraints[0].target_point_offset.z = 0.0;

  // Add offset constraint
  goal.request.goal_constraints.resize(1);
  goal.request.goal_constraints[0] = goal_constraint0;

  // -------------------------------------------------------------------------------------------
  // Visualize goals in rviz
  visual_tools->publishArrow(goal_pose.pose, moveit_visual_tools::GREEN);
  visual_tools->publishEEMarkers(goal_pose.pose, moveit_visual_tools::GREEN);

  // -------------------------------------------------------------------------------------------
  // Plan
  movegroup_action_->sendGoal(goal);

  if(!movegroup_action_->waitForResult(ros::Duration(20.0)))
  {
    ROS_INFO_STREAM_NAMED("utilities","Did not finish in time.");
    return false;
  }
  if (movegroup_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM_NAMED("utilities","sendToPose() plan succeeded.");
    return true;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("utilities","movegroup_action failed: " << movegroup_action_->getState().toString() << ": " << movegroup_action_->getState().getText());
    return false;
  }

  return true;
}

geometry_msgs::Pose BaxterUtilities::getCurrentPose(moveit_visual_tools::VisualToolsPtr visual_tools)
{
  ROS_INFO_STREAM_NAMED("baxter_utilities","Getting pose of end effector for " << visual_tools->getEEParentLink());

  // Start listening to the joint states
  visual_tools->getPlanningSceneMonitor()->startStateMonitor("/robot/joint_states", "/attached_collision_object");
  ros::Duration(2.0).sleep();

  robot_state::RobotState state = visual_tools->getPlanningSceneMonitor()->getPlanningScene()->getCurrentState();
  state.updateLinkTransforms();
  Eigen::Affine3d pose = state.getGlobalLinkTransform(visual_tools->getEEParentLink());
      
  geometry_msgs::Pose pose_msg = visual_tools->convertPose(pose);

  ROS_INFO_STREAM_NAMED("baxter_utilities","pose is:");
  std::cout << "pose_msg.position.x = " << pose_msg.position.x << ";\n";
  std::cout << "pose_msg.position.y = " << pose_msg.position.y << ";\n";
  std::cout << "pose_msg.position.z = " << pose_msg.position.z << ";\n";
  std::cout << "pose_msg.orientation.x = " << pose_msg.orientation.x << ";\n";
  std::cout << "pose_msg.orientation.y = " << pose_msg.orientation.y << ";\n";
  std::cout << "pose_msg.orientation.z = " << pose_msg.orientation.z << ";\n";
  std::cout << "pose_msg.orientation.w = " << pose_msg.orientation.w << ";\n";

  // Feedback
  visual_tools->publishArrow(pose_msg, moveit_visual_tools::RED, moveit_visual_tools::LARGE);

  return pose_msg;
}

geometry_msgs::Pose BaxterUtilities::getReadyPose(const std::string &side)
{
  geometry_msgs::Pose pose_msg;

  if (side == "left")
  {
    // Hard coded pose for left arm
    pose_msg.position.x = 0.626408;
    pose_msg.position.y = 0.81637;
    pose_msg.position.z = 0.247539;
    pose_msg.orientation.x = -0.381427;
    pose_msg.orientation.y = 0.921769;
    pose_msg.orientation.z = 0.0227843;
    pose_msg.orientation.w = 0.0658544;
  }
  else
  {
    pose_msg.position.x = 0.613639;
    pose_msg.position.y = -0.816773;
    pose_msg.position.z = 0.243619;
    pose_msg.orientation.x = 0.384237;
    pose_msg.orientation.y = 0.922543;
    pose_msg.orientation.z = -0.00470893;
    pose_msg.orientation.w = 0.035402;
  }

  return pose_msg;
}


} //namespace

