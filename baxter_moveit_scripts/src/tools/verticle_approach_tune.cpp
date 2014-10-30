/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:   Tune PID controllers by generating a trajectory message that moves the end effector vertically.
*/

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
//#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/plan_execution/plan_execution.h>
//#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Visualization
#include <moveit_simple_grasps/visual_tools.h>

// Baxter specific properties
#include <baxter_pick_place/grasp_data_loader.h>
#include <baxter_pick_place/custom_environment2.h>

// Tuning the PID Controller
#include <baxter_pick_place/pid_tune.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

namespace baxter_pick_place
{

// Class
class VerticleApproachTune
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Action Servers and Clients
  boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > movegroup_action_;

  // MoveIt Components
  boost::shared_ptr<tf::TransformListener> tf_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  boost::shared_ptr<plan_execution::PlanExecution> plan_execution_;

  // class for publishing stuff to rviz
  moveit_simple_grasps::VisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::RobotGraspData grasp_data_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  // Up and down verticle trajectories, cached
  moveit_msgs::RobotTrajectory trajectory_msg_;
  moveit_msgs::RobotTrajectory reverse_trajectory_msg_;

  // which baxter arm are we using
  std::string arm_;
  std::string planning_group_name_;

public:

  // Constructor
  VerticleApproachTune()
    : arm_("right"),
      planning_group_name_(arm_+"_arm")
  {

    // -----------------------------------------------------------------------------------------------
    // Connect to move_group action server
    movegroup_action_.reset(new actionlib::SimpleActionClient
      <moveit_msgs::MoveGroupAction>("move_group", true));
    while(!movegroup_action_->waitForServer(ros::Duration(2.0)))
      ROS_INFO_STREAM_NAMED("verticle_tune","Waiting for the move_group action server");

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_ = loadRobotGraspData(arm_, BLOCK_SIZE); // Load robot specific data

    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    if(!loadPlanningSceneMonitor())
    {
      ROS_ERROR_STREAM_NAMED("verticle_tune","Unable to load planning scene monitor");
      return;
    }

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_simple_grasps::VisualTools(baxter_pick_place::BASE_LINK));
    visual_tools_->setLifetime(120.0);
    visual_tools_->setMuted(false);
    visual_tools_->setGraspPoseToEEFPose(grasp_data_.grasp_pose_to_eef_pose_);
    visual_tools_->setEEGroupName(grasp_data_.ee_group_);
    visual_tools_->setPlanningGroupName(planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return;

    // ---------------------------------------------------------------------------------------------
    // Do it
    geometry_msgs::Pose start_pose = createStartPose();
    tuneJointPIDs(start_pose);



    ROS_INFO_STREAM_NAMED("verticle_tune","Success! Waiting 5 sec before shutting down.");
    ros::Duration(5).sleep();

    // Shutdown
    baxter_util_.disableBaxter();
  }

  ~VerticleApproachTune()
  {
    ROS_INFO_STREAM_NAMED("temp","shutting down");
    std::cout << "deconstructor";
    // Shutdown
    baxter_util_.disableBaxter();
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  bool tuneJointPIDs(const geometry_msgs::Pose& start_pose)
  {
    // ---------------------------------------------------------------------------------------------
    // Start Position
    ROS_INFO_STREAM_NAMED("verticle_tune","Sending arm to start position ----------------------------------");

    if(!sendPoseCommand(start_pose))
    {
      ROS_ERROR_STREAM_NAMED("verticle_tune","Failed to go to start position");
      return false;
    }

    // ---------------------------------------------------------------------------------------------
    // Compute the up and down trajectory once

    double desired_approach_distance = 0.4; // The distance the origin of a robot link needs to travel
    Eigen::Vector3d approach_direction; // Approach direction (negative z axis)
    approach_direction << 0, 0, -1;

    if( !computeStraightLinePath(approach_direction, desired_approach_distance, trajectory_msg_,
        reverse_trajectory_msg_) )
    {
      ROS_ERROR_STREAM_NAMED("verticle_tune","Failed to generate straight line path");
      return false;
    }

    // ---------------------------------------------------------------------------------------------
    // Start tuning the PID controller
    PIDTune pid_tuner;

    bool runTrajectory = true;

    PerturbJointFn perturb_fn1 = boost::bind(&VerticleApproachTune::executeTrajectoryUp, this);
    PerturbJointFn perturb_fn2 = boost::bind(&VerticleApproachTune::executeTrajectoryDown, this);

    if( !pid_tuner.tuneJoint("right_s1", perturb_fn1, perturb_fn2) )
    {
      ROS_ERROR_STREAM_NAMED("verticle_tune","An error occured while tuning the PID controller");
      return false;
    }

    return true;
  }

  bool executeTrajectoryUp()
  {
    ROS_INFO_STREAM_NAMED("verticle_tune","Raising up -----------------------------------------");

    executeTrajectoryMsg(reverse_trajectory_msg_);

    ros::Duration(1).sleep();

    return true;
  }

  bool executeTrajectoryDown()
  {
    ROS_INFO_STREAM_NAMED("verticle_tune","Lowering Down -----------------------------------------");

    executeTrajectoryMsg(trajectory_msg_);

    ros::Duration(1).sleep();

    // ---------------------------------------------------------------------------------------------
    // Demo will automatically reset arm
    ROS_INFO_STREAM_NAMED("verticle_tune","Finished ------------------------------------------------");

    return true;
  }

  // Moves the arm to a specified pose
  bool sendPoseCommand(const geometry_msgs::Pose& pose)
  {
    // -----------------------------------------------------------------------------------------------
    // Make a stamped version of the pose
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = pose;

    // -----------------------------------------------------------------------------------------------
    // Create move_group goal
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = planning_group_name_;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = 5.0;

    // -------------------------------------------------------------------------------------------
    // Create goal state
    goal_pose.header.frame_id = baxter_pick_place::BASE_LINK;
    double tolerance_pose = 1e-4; // default: 1e-3... meters
    double tolerance_angle = 1e-2; // default 1e-2... radians
    ROS_ERROR_STREAM_NAMED("temp","Not implemented TODO next link");
    moveit_msgs::Constraints goal_constraint0 = 
      kinematic_constraints::constructGoalConstraints(
                                                      "", // visual_tools_->getEEParentLink(), 
                                                      goal_pose, tolerance_pose, tolerance_angle);

    //ROS_INFO_STREAM_NAMED("verticle_tune","Goal pose " << goal_pose);

    // Create offset constraint
    goal_constraint0.position_constraints[0].target_point_offset.x = 0.0;
    goal_constraint0.position_constraints[0].target_point_offset.y = 0.0;
    goal_constraint0.position_constraints[0].target_point_offset.z = 0.0;

    // Add offset constraint
    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0] = goal_constraint0;

    // -------------------------------------------------------------------------------------------
    // Visualize goals in rviz
    visual_tools_->publishArrow(goal_pose.pose, moveit_simple_grasps::GREEN);

    visual_tools_->publishEEMarkers(goal_pose.pose, moveit_simple_grasps::GREEN);

    // -------------------------------------------------------------------------------------------
    // Plan
    movegroup_action_->sendGoal(goal);

    if(!movegroup_action_->waitForResult(ros::Duration(20.0)))
    {
      ROS_INFO_STREAM_NAMED("verticle_tune","Did not finish in time.");
      return false;
    }
    if (movegroup_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM_NAMED("verticle_tune","Plan successful!");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("verticle_tune","move_group failed: " << movegroup_action_->getState().toString() << ": " << movegroup_action_->getState().getText());
      return false;
    }

    return true;
  }

  /**
   *  \brief Function for testing multiple directions
   * \param approach_direction - direction to move end effector straight
   * \param desired_approach_distance - distance the origin of a robot link needs to travel
   * \param trajectory_msg - resulting path
   * \return true on success
   */
  bool computeStraightLinePath( Eigen::Vector3d approach_direction, double desired_approach_distance,
    moveit_msgs::RobotTrajectory& trajectory_msg, moveit_msgs::RobotTrajectory& reverse_trajectory_msg )
  {
    // ---------------------------------------------------------------------------------------------
    // Get planning scene
    const planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();
    robot_state::RobotState approach_state = planning_scene->getCurrentState();

    // Output state info
    //approach_state.printStateInfo();
    //approach_state.printTransforms();

    // ---------------------------------------------------------------------------------------------
    // Settings for computeCartesianPath

    // End effector parent link
    ROS_ERROR_STREAM_NAMED("temp","Not implemented TODO next link");
    const std::string &ik_link = ""; //visual_tools_->getEEParentLink();
    const moveit::core::LinkModel *ik_link_model = approach_state.getLinkModel(ik_link);    

    // Joint model group
    const moveit::core::JointModelGroup *joint_model_group 
      = approach_state.getJointModelGroup(planning_group_name_);

    // Resolution of trajectory
    double max_step = 0.001; // The maximum distance in Cartesian space between consecutive points on the resulting path

    // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
    double jump_threshold = 0.0; // disabled

    // ---------------------------------------------------------------------------------------------
    // Check for kinematic solver
    if( !joint_model_group->canSetStateFromIK( ik_link ) )
    {
      // Set kinematic solver
      const std::pair<robot_model::JointModelGroup::KinematicsSolver, 
                      robot_model::JointModelGroup::KinematicsSolverMap>& allocators =        
        approach_state.getJointModelGroup(planning_group_name_)->getGroupKinematics();
      //const std::pair<robot_model::SolverAllocatorFn, robot_model::SolverAllocatorMapFn> &allocators =
      //approach_state.getJointModelGroup(planning_group_name_)->getSolverAllocators();
      if( !allocators.first)
        ROS_ERROR_STREAM_NAMED("verticle_test","No IK Solver loaded - make sure moveit_config/kinamatics.yaml is loaded in this namespace");
    }

    // -----------------------------------------------------------------------------------------------
    // Compute Cartesian Path
    ROS_INFO_STREAM_NAMED("verticle_test","Preparing to computer cartesian path");

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
        // TODO approach_validCallback
      );

    ROS_INFO_STREAM("Approach distance: " << d_approach );
    if( d_approach == 0 )
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Failed to computer cartesian path: distance is 0");
      return false;
    }

    // -----------------------------------------------------------------------------------------------
    // Get current RobotState  (in order to specify all joints not in robot_state_trajectory)
    //robot_state::RobotState this_robot_state = planning_scene->getCurrentState();

    // -----------------------------------------------------------------------------------------------
    // Smooth the path and add velocities/accelerations

    // Get the joint limits of planning group
    const std::vector<moveit_msgs::JointLimits> &joint_limits = joint_model_group->getVariableLimits();

    // Copy the vector of RobotStates to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(new robot_trajectory::RobotTrajectory(
        planning_scene->getRobotModel(), planning_group_name_));

    for (std::size_t k = 0 ; k < robot_state_trajectory.size() ; ++k)
      robot_trajectory->addSuffixWayPoint(robot_state_trajectory[k], 0.0);

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

    ROS_INFO_STREAM("New trajectory:\n" << trajectory_msg);

    // ----------------------------------------------------------------------
    // Experimental: reverse the trajectory
    robot_trajectory::RobotTrajectoryPtr robot_trajectory_reverse(new robot_trajectory::RobotTrajectory(
        planning_scene->getRobotModel(), planning_group_name_));

    for (std::size_t k = 0 ; k < robot_state_trajectory.size() ; ++k)
      robot_trajectory_reverse->addPrefixWayPoint(robot_state_trajectory[k], 0.0);

    // Perform iterative parabolic smoothing
    iterative_smoother.computeTimeStamps( *robot_trajectory_reverse );

    // Convert trajectory to a message
    robot_trajectory_reverse->getRobotTrajectoryMsg(reverse_trajectory_msg);

    return true;
  }

  /**
   * \brief Execute planned trajectory
   * \param trajectory_msg
   * \return true if successful
   */
  bool executeTrajectoryMsg(const moveit_msgs::RobotTrajectory& trajectory_msg)
  {
    ROS_INFO_STREAM_NAMED("verticle_tune","Executing trajectory");

    // Make sure the objects have been loaded
    if( !trajectory_execution_manager_ )
    {
      // Create a trajectory execution manager
      trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager
        (planning_scene_monitor_->getRobotModel()));
      plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
    }

    plan_execution_->getTrajectoryExecutionManager()->clear();
    if(plan_execution_->getTrajectoryExecutionManager()->push(trajectory_msg))
    {
      plan_execution_->getTrajectoryExecutionManager()->execute();

      // wait for the trajectory to complete
      moveit_controller_manager::ExecutionStatus es = plan_execution_->getTrajectoryExecutionManager()->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        ROS_INFO_STREAM_NAMED("verticle_tune","Trajectory execution succeeded");
      else
      {
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          ROS_INFO_STREAM_NAMED("verticle_tune","Trajectory execution preempted");
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            ROS_INFO_STREAM_NAMED("verticle_tune","Trajectory execution timed out");
          else
            ROS_INFO_STREAM_NAMED("verticle_tune","Trajectory execution control failed");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("verticle_tune","Failed to push trajectory");
      return false;
    }

    return true;
  }

  /**
   * \brief Create the location for the end effector to start at
   * \return the pose
   */
  geometry_msgs::Pose createStartPose()
  {
    geometry_msgs::Pose start_pose;

    // Position
    start_pose.position.x = 0.6;
    start_pose.position.z = 0.2;
    if( arm_.compare("right") == 0 ) // equal
      start_pose.position.y = -0.4;
    else
      start_pose.position.y = 0.3;

    // Orientation
    double angle = M_PI;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    start_pose.orientation.x = quat.x();
    start_pose.orientation.y = quat.y();
    start_pose.orientation.z = quat.z();
    start_pose.orientation.w = quat.w();


    return start_pose;
  }

  /**
   * \brief Load a planning scene monitor
   * \return true if successful in loading
   */
  bool loadPlanningSceneMonitor()
  {
    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    tf_.reset(new tf::TransformListener());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf_));

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (planning_scene_monitor_->getPlanningScene())
    {
      //planning_scene_monitor_->startWorldGeometryMonitor();
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      //planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
      //"dave_planning_scene");
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("verticle_tune","Planning scene not configured");
      return false;
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Wait for complete state to be recieved
    std::vector<std::string> missing_joints;
    int counter = 0;
    while( !planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok() )
    {
      ROS_INFO_STREAM_NAMED("verticle_tune","Waiting for complete state...");
      ros::Duration(0.25).sleep();
      ros::spinOnce();

      // Show unpublished joints
      if( counter > 6 )
      {
        planning_scene_monitor_->getStateMonitor()->haveCompleteState( missing_joints );
        for(int i = 0; i < missing_joints.size(); ++i)
          ROS_WARN_STREAM_NAMED("verticle_tune","Unpublished joints: " << missing_joints[i]);
      }
      counter++;
    }

    return true;
  }

}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("verticle_tune","Verticle Approach Tune");

  ros::init(argc, argv, "verticle_approach_tune");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  baxter_pick_place::VerticleApproachTune tester;

  ROS_INFO_STREAM_NAMED("verticle_tune","Shutting down.");

  return 0;
}

