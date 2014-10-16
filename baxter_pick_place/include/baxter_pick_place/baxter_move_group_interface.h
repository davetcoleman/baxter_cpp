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
 * \brief   Helper functions for moving baxter
 * \author  Dave Coleman
 */

#ifndef BAXTER_CONTROL__BAXTER_MOVE_GROUP_INTERFACE_
#define BAXTER_CONTROL__BAXTER_MOVE_GROUP_INTERFACE_

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit/kinematic_constraints/utils.h>

namespace baxter_pick_place
{

static const std::string ROBOT_DESCRIPTION="robot_description";
//static const std::string PLANNING_GROUP_BOTH_NAME = "both_arms";
//static const std::string BASE_LINK = "base";
static const std::string NEUTRAL_POSE_NAME = "both_neutral";
static const std::string JOINT_STATE_TOPIC = "/robot/joint_states";

class BaxterMoveGroupInterface
{
public:

  // Interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  // Track the planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Trajectory execution manager
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  boost::shared_ptr<plan_execution::PlanExecution> plan_execution_;

  BaxterMoveGroupInterface(const std::string planning_group,
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr())
    : planning_scene_monitor_(planning_scene_monitor)
  {
    // Check if planning scene monitor was passed in
    if( !planning_scene_monitor_ )
    {
      loadPlanningSceneMonitor();
    }

    // Modify a planning scene monitor to listen to joint states so that we have the current state of the robot
    planning_scene_monitor_->startStateMonitor(JOINT_STATE_TOPIC);

    // We manually set the options to save on URDF loading time
    // We also set the move_group to operate on a paritcular planning group
    moveit::planning_interface::MoveGroup::Options opts(planning_group, ROBOT_DESCRIPTION);
    opts.robot_model_ = planning_scene_monitor_->getRobotModel();

    // Load our move_group interface
    move_group_.reset(new move_group_interface::MoveGroup(opts));

    // Create trajectory execution manager
    if( !trajectory_execution_manager_ )
    {
      // Create a trajectory execution manager
      trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager
        (planning_scene_monitor_->getRobotModel()));
      plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
    }

    ROS_DEBUG_STREAM_NAMED("baxter_move","Baxter Move Group Interface loaded");
  }

  bool positionBaxterReady()
  {
    // Send to ready position
    ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm ready positions...");

    return sendToPose("both_ready");
  }

  bool positionBaxterNeutral()
  {
    // Send to neutral position
    ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm neutral positions...");

    return sendToPose(NEUTRAL_POSE_NAME);
  }

  /**
   * \brief open/close baxter's grippers
   * \param bool if it should be open or closed
   * \return true on success
   */
  bool openEE(bool open, const moveit_simple_grasps::GraspData& grasp_data)
  {
    planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor_);
    robot_state::RobotState current_state = ls->getCurrentState();

    robot_trajectory::RobotTrajectoryPtr ee_traj(new robot_trajectory::RobotTrajectory(
        current_state.getRobotModel(), grasp_data.ee_group_));

    if (open)
    {
      ROS_INFO_STREAM_NAMED("baxter_move","Opening end effector");
      ee_traj->setRobotTrajectoryMsg(current_state, grasp_data.pre_grasp_posture_); // open
    }
    else
    {
      ROS_INFO_STREAM_NAMED("baxter_move","Closing end effector");
      ee_traj->setRobotTrajectoryMsg(current_state, grasp_data.grasp_posture_); // closed
    }

    // Convert trajectory to a moveit_msgs::RobotTrajectory message and pass to plan
    moveit_msgs::RobotTrajectory trajectory_msg;
    ee_traj->getRobotTrajectoryMsg(trajectory_msg);

    // Execute trajectory
    return executeTrajectory(trajectory_msg);
  }


  /**
   * \brief Send baxter to a named pose defined in the SRDF
   * \param pose_name - name of pose in SRDF
   * \return true if sucessful in planning and moving there
   */
  bool sendToPose(const std::string &pose_name)
  {
    ROS_INFO_STREAM_NAMED("baxter_move_group_interface","Sending to pose '" << pose_name << "'");

    move_group_->setNamedTarget(pose_name);
    move_group_->setPlanningTime(15);
    bool result = move_group_->move();

    if( !result )
      ROS_ERROR_STREAM_NAMED("utilities","Failed to send Baxter to pose '" << pose_name << "'");

    return result;
  }

  /**
   * \brief Moves the arm to a specified pose
   * \param pose - desired goal
   * \param group_name - which arm / planning group to use the pose with
   * \param visual_tools - copy of tool for publishing visual objects
   * \param planning_scene_diff - allows direct access to change the planning scene during planning
   * \return true if sucessful in planning and moving there
   */
  bool sendToPose(const geometry_msgs::Pose& pose, moveit_visual_tools::VisualToolsPtr visual_tools, 
                  const moveit_msgs::PlanningScene &planning_scene_diff)                  
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.frame_id = planning_scene_monitor_->getPlanningScene()->getCurrentState().getRobotModel()->getModelFrame();

    sendToPose(pose_stamped, visual_tools, planning_scene_diff);
  }

  bool sendToPose(const geometry_msgs::PoseStamped& pose, 
                  moveit_visual_tools::VisualToolsPtr visual_tools, const moveit_msgs::PlanningScene &planning_scene_diff)
  {
    // Make sure frame_id was set
    if (pose.header.frame_id.empty())
    {
      ROS_ERROR_STREAM_NAMED("baxter_move","Frame ID was not set for requested pose");
      return false;
    }
    // Clear out any old targets
    move_group_->clearPoseTargets();

    // Create new target
    move_group_->setPoseTarget(pose);
    move_group_->setNumPlanningAttempts(4);
    move_group_->setPlanningTime(20);
    move_group_->setGoalPositionTolerance(1e-3); // meters
    move_group_->setGoalOrientationTolerance(1e-2); // radians

    // Hack to disable collisions with octomap
    //goal.planning_options.planning_scene_diff = planning_scene_diff; // TODO re-enable?

    // Visualize goals in rviz
    visual_tools->publishArrow(pose.pose, moveit_visual_tools::GREEN);

    // Plan
    std::cout << "sending " << std::endl;
    ROS_ERROR_STREAM_NAMED("baxter_move_group_interface","waiting for move to finsih is disabled");
    moveit::planning_interface::MoveItErrorCode code = move_group_->asyncMove();
    std::cout << "sending finished" << std::endl;
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    return convertResult(code);
  }

  /*
  // Moves the arm to a specified pose
  bool sendPoseCommand(const geometry_msgs::Pose& pose, const std::string& planning_group_name, const std::string& base_link,
                       moveit_visual_tools::VisualToolsPtr visual_tools)
  {
    // -----------------------------------------------------------------------------------------------
    // Make a stamped version of the pose
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = pose;

    // -----------------------------------------------------------------------------------------------
    // Create move_group goal
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = planning_group_name;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = 5.0;

    // -------------------------------------------------------------------------------------------
    // Create goal state
    goal_pose.header.frame_id = base_link;
    double tolerance_position = 1e-3; // default: 1e-3... meters
    double tolerance_angle = 1e-2; // default 1e-2... radians
    moveit_msgs::Constraints goal_constraint0 = kinematic_constraints::constructGoalConstraints(
      grasp_data_.ee_parent_link_, goal_pose, tolerance_position, tolerance_angle);

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
      ROS_INFO_STREAM_NAMED("verticle_test","Did not finish in time.");
      return false;
    }
    if (movegroup_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM_NAMED("verticle_test","Plan successful!");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","move_group failed: " << movegroup_action_->getState().toString() << ": " << movegroup_action_->getState().getText());
      return false;
    }

    return true;
  }
  */

  /**
   * \brief Get the current pose of the desired end effector specified in the visual_tools object
   * \param visual_tools - copy of tool for publishing visual objects
   * \return pose of end effector
   */
  geometry_msgs::Pose getCurrentPose(moveit_visual_tools::VisualToolsPtr visual_tools,
    const moveit_simple_grasps::GraspData& grasp_data)
  {
    ROS_INFO_STREAM_NAMED("baxter_move_group_interface","Getting pose of end effector for " << grasp_data.ee_parent_link_);

    robot_state::RobotState state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    state.updateLinkTransforms();
    Eigen::Affine3d pose = state.getGlobalLinkTransform(grasp_data.ee_parent_link_);

    geometry_msgs::Pose pose_msg = visual_tools->convertPose(pose);

    ROS_INFO_STREAM_NAMED("baxter_move_group_interface","pose is:");
    std::cout << "geometry_msgs::PoseStamped pose_msg;\n";
    std::cout << "pose_msg.pose.position.x = " << pose_msg.position.x << ";\n";
    std::cout << "pose_msg.pose.position.y = " << pose_msg.position.y << ";\n";
    std::cout << "pose_msg.pose.position.z = " << pose_msg.position.z << ";\n";
    std::cout << "pose_msg.pose.orientation.x = " << pose_msg.orientation.x << ";\n";
    std::cout << "pose_msg.pose.orientation.y = " << pose_msg.orientation.y << ";\n";
    std::cout << "pose_msg.pose.orientation.z = " << pose_msg.orientation.z << ";\n";
    std::cout << "pose_msg.pose.orientation.w = " << pose_msg.orientation.w << ";\n";

    // Feedback
    visual_tools->publishArrow(pose_msg, moveit_visual_tools::RED, moveit_visual_tools::LARGE);

    return pose_msg;
  }

  /**
   * \brief Move baxter in particular direction
   * \param
   * \param
   * \return true on success
   */
  bool moveStraight(Eigen::Vector3d approach_direction, double desired_approach_distance,
    const moveit_simple_grasps::GraspData& grasp_data,
    const std::string& planning_group_name, moveit_visual_tools::VisualToolsPtr visual_tools)
  {
    moveit_msgs::RobotTrajectory trajectory_msg; // the resulting path

    // Create the trajectory
    if( !computeStraightLinePath(approach_direction, desired_approach_distance, trajectory_msg, grasp_data,
        planning_group_name, visual_tools) )
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Failed to generate straight line path");
      return false;
    }

    // Display the generated path
    //if( !visual_tools_->publishTrajectoryPath(trajectory_msg, false ) )
    //  return false;

    // Execute trajectory
    return executeTrajectory(trajectory_msg);
  }

  /**
   * \brief
   * \param input - description
   * \param input - description
   * \return
   */
  bool executeTrajectory(const moveit_msgs::RobotTrajectory& trajectory_msg)
  {
    // Create plan for move_group
    moveit::planning_interface::MoveGroup::Plan plan;
    //plan.start_state_ = current_state;
    plan.planning_time_ = 0.0;
    plan.trajectory_ = trajectory_msg;

    // Execute trajectory
    moveit::planning_interface::MoveItErrorCode code = move_group_->execute(plan);
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    return convertResult(code);
  }

  /**
   * \brief
   * \param approach_direction - direction to move end effector straight
   * \param desired_approach_distance - distance the origin of a robot link needs to travel
   * \param trajectory_msg - resulting path
   * \return true on success
   */
  bool computeStraightLinePath( Eigen::Vector3d approach_direction, double desired_approach_distance,
    moveit_msgs::RobotTrajectory& trajectory_msg, const moveit_simple_grasps::GraspData& grasp_data,
    const std::string& planning_group_name, moveit_visual_tools::VisualToolsPtr visual_tools)
  {
    // Get planning scene
    planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor_);
    robot_state::RobotState approach_state = ls->getCurrentState();

    // Output state info
    //approach_state.printStateInfo();
    //approach_state.printTransforms();

    // ---------------------------------------------------------------------------------------------
    // Settings for computeCartesianPath

    // End effector parent link
    const std::string &ik_link = grasp_data.ee_parent_link_;
    const moveit::core::LinkModel *ik_link_model = approach_state.getLinkModel(ik_link);

    // Joint model group
    const moveit::core::JointModelGroup *joint_model_group = approach_state.getJointModelGroup(planning_group_name);

    // Resolution of trajectory
    double max_step = 0.001; // The maximum distance in Cartesian space between consecutive points on the resulting path

    // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
    double jump_threshold = 0.0; // disabled

    // ---------------------------------------------------------------------------------------------
    // Check for kinematic solver
    if( !joint_model_group->canSetStateFromIK( ik_link ) )
    {
      // Set kinematic solver
      const std::pair<robot_model::JointModelGroup::KinematicsSolver, robot_model::JointModelGroup::KinematicsSolverMap>&
        allocators = approach_state.getJointModelGroup(planning_group_name)->getGroupKinematics();

      if( !allocators.first)
        ROS_ERROR_STREAM_NAMED("computeStraightLinePath","No IK Solver loaded - make sure moveit_config/kinamatics.yaml is loaded in this namespace");
    }

    // -----------------------------------------------------------------------------------------------
    // Compute Cartesian Path

    std::vector<robot_state::RobotStatePtr> robot_state_trajectory; // create resulting generated trajectory (result)

    double d_approach =
      approach_state.computeCartesianPath(
        joint_model_group,
        robot_state_trajectory,
        ik_link_model,
        approach_direction,
        true,                      // direction is in global reference frame
        desired_approach_distance,
        max_step,
        jump_threshold
      );

    ROS_INFO_STREAM("Approach distance: " << d_approach );
    if( d_approach == 0 )
    {
      ROS_ERROR_STREAM_NAMED("computeStraightLinePath","Failed to computer cartesian path: distance is 0");
      return false;
    }

    std::cout << " " << d_approach << " " << std::flush;

    // -----------------------------------------------------------------------------------------------
    // Smooth the path and add velocities/accelerations
    //const std::vector<moveit_msgs::JointLimits> &joint_limits = joint_model_group->getVariableLimits();

    // Copy the vector of RobotStates to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(new robot_trajectory::RobotTrajectory(
        approach_state.getRobotModel(), planning_group_name));

    for (std::size_t k = 0 ; k < robot_state_trajectory.size() ; ++k)
      robot_trajectory->addSuffixWayPoint(robot_state_trajectory[k], 0.0);

    // Debug - display the generated path
    ROS_ERROR_STREAM_NAMED("temp","debugging trajectory before smoothing");

    robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
    visual_tools->publishTrajectoryPath(trajectory_msg, true);

    // Perform iterative parabolic smoothing
    trajectory_processing::IterativeParabolicTimeParameterization iterative_smoother;
    iterative_smoother.computeTimeStamps( *robot_trajectory );
    /*                                         robot_trajectory,
                                               trajectory_out,
                                               joint_limits,
                                               this_robot_state // start_state
                                               );
    */

    // Convert trajectory to a message
    robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);

    // debug
    ROS_ERROR_STREAM_NAMED("temp","debugging trajectory after smoothing");
    visual_tools->publishTrajectoryPath(trajectory_msg, true);

    return true;
  }

  /**
   * \brief Execute planned trajectory
   * \param trajectory_msg
   * \return true if successful
   */
  bool executeTrajectoryMsg(const moveit_msgs::RobotTrajectory& trajectory_msg)
  {
    //ROS_INFO_STREAM_NAMED("cartesian_controller","Executing trajectory");

    // Reset
    plan_execution_->getTrajectoryExecutionManager()->clear();

    if(plan_execution_->getTrajectoryExecutionManager()->push(trajectory_msg))
    {
      plan_execution_->getTrajectoryExecutionManager()->execute();

      // wait for the trajectory to complete
      moveit_controller_manager::ExecutionStatus es = plan_execution_->getTrajectoryExecutionManager()->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        ROS_DEBUG_STREAM_NAMED("cartesian_controller","Trajectory execution succeeded");
      else
      {
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          ROS_INFO_STREAM_NAMED("cartesian_controller","Trajectory execution preempted");
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            ROS_INFO_STREAM_NAMED("cartesian_controller","Trajectory execution timed out");
          else
            ROS_INFO_STREAM_NAMED("cartesian_controller","Trajectory execution control failed");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("cartesian_controller","Failed to push trajectory");
      return false;
    }

    return true;
  }


  /**
   * \brief Hard coded poses for baxter's end effectors
   * \param side - left or right
   * \return pose of ready position
   */
  geometry_msgs::Pose getReadyPose(const std::string &side)
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

  bool loadPlanningSceneMonitor()
  {
    ROS_DEBUG_STREAM_NAMED("baxter_move","Loading planning scene monitor");

    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

    ros::spinOnce();
    ros::Duration(0.5).sleep(); // todo: reduce this time?
    ros::spinOnce();

    if (!planning_scene_monitor_->getPlanningScene())
    {
      ROS_ERROR_STREAM_NAMED("baxter_move","Planning scene not configured");
      return false;
    }

    return true;
  }

  bool convertResult(moveit::planning_interface::MoveItErrorCode& code)
  {
    switch (code.val)
    {
      case moveit_msgs::MoveItErrorCodes::SUCCESS:
        ROS_INFO_STREAM_NAMED("baxter_move","Planning and execution succeeded");
        return true;

      case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        ROS_ERROR_STREAM_NAMED("baxter_move","Failed because of invalid motion plan");
        return false;

        /* Template:
           case moveit_msgs::MoveItErrorCodes::
           ROS_ERROR_STREAM_NAMED("baxter_move","Failed because of ");
           return false;
        */
    }

    ROS_ERROR_STREAM_NAMED("baxter_move","Planning and execution failed with code " << code.val);
    return false;
    /*
      int32 PLANNING_FAILED=-1

      int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
      int32 CONTROL_FAILED=-4
      int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
      int32 TIMED_OUT=-6
      int32 PREEMPTED=-7

      # planning & kinematics request errors
      int32 START_STATE_IN_COLLISION=-10
      int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11

      int32 GOAL_IN_COLLISION=-12
      int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
      int32 GOAL_CONSTRAINTS_VIOLATED=-14

      int32 INVALID_GROUP_NAME=-15
      int32 INVALID_GOAL_CONSTRAINTS=-16
      int32 INVALID_ROBOT_STATE=-17
      int32 INVALID_LINK_NAME=-18
      int32 INVALID_OBJECT_NAME=-19

      # system errors
      int32 FRAME_TRANSFORM_FAILURE=-21
      int32 COLLISION_CHECKING_UNAVAILABLE=-22
      int32 ROBOT_STATE_STALE=-23
      int32 SENSOR_INFO_STALE=-24

      # kinematics errors
      int32 NO_IK_SOLUTION=-31
    */
  }

};

typedef boost::shared_ptr<BaxterMoveGroupInterface> BaxterMoveGroupInterfacePtr;
typedef boost::shared_ptr<const BaxterMoveGroupInterface> BaxterMoveGroupInterfaceConstPtr;

} //namespace

#endif
