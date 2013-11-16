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
   Desc:   Generates a trajectory message that moves the end effector vertically. Used for testing PIDs
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
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Grasp generation
#include <block_grasp_generator/block_grasp_generator.h>

// Baxter specific properties
#include <baxter_pick_place/baxter_data.h>
#include <baxter_pick_place/custom_environment2.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>
#include <baxter_control/baxter_to_csv.h>

namespace baxter_pick_place
{

// Class
class VerticleApproachTest
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
  block_grasp_generator::VisualizationToolsPtr visual_tools_;

  // data for generating grasps
  block_grasp_generator::RobotGraspData grasp_data_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  // baxter data recorder
  baxter_control::BaxterToCSV baxter_record_;

  // which baxter arm are we using
  std::string arm_;
  std::string planning_group_name_;

  // where to save trajectory files
  std::string file_path_;
  int file_id_; // incremental file naming

public:

  // Constructor
  VerticleApproachTest()
    : arm_("left"),
      planning_group_name_(arm_+"_arm"),
      baxter_record_(true), // true means we are recording position commands
      file_path_("/home/dave/ros/ws_baxter/src/baxter/baxter_pick_place/src/tests/verticle_approach_matlab/"),
      file_id_(0)
  {

    // -----------------------------------------------------------------------------------------------
    // Connect to move_group action server
    movegroup_action_.reset(new actionlib::SimpleActionClient
      <moveit_msgs::MoveGroupAction>("move_group", true));
    while(!movegroup_action_->waitForServer(ros::Duration(2.0)))
      ROS_INFO_STREAM_NAMED("verticle_test","Waiting for the move_group action server");

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_ = loadRobotGraspData(arm_, BLOCK_SIZE); // Load robot specific data

    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    if(!loadPlanningSceneMonitor())
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Unable to load planning scene monitor");
      return;
    }

    // ---------------------------------------------------------------------------------------------
    // Create trajectory execution manager
    if( !trajectory_execution_manager_ )
    {
      // Create a trajectory execution manager
      trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager
        (planning_scene_monitor_->getRobotModel()));
      plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
    }

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new block_grasp_generator::VisualizationTools(baxter_pick_place::BASE_LINK));
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
    // Create start pose
    geometry_msgs::Pose start_pose = createStartPose();

    // ---------------------------------------------------------------------------------------------
    // Move into start position
    ROS_INFO_STREAM_NAMED("verticle_test","Sending arm to start position ----------------------------------");

    if(!sendPoseCommand(start_pose))
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Failed to go to start position");
      return;
    }

    // ---------------------------------------------------------------------------------------------
    // Create the trajectory
    createVerticleTrajectory(start_pose);

    ROS_INFO_STREAM_NAMED("verticle_test","Success! Waiting 5 sec before shutting down.");
    ros::Duration(5).sleep();

    // Shutdown
    baxter_util_.disableBaxter();
  }

  ~VerticleApproachTest()
  {
    ROS_INFO_STREAM_NAMED("temp","shutting down");
    std::cout << "deconstructor";
    // Shutdown
    baxter_util_.disableBaxter();
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  bool createVerticleTrajectory(const geometry_msgs::Pose& start_pose)
  {
    // ---------------------------------------------------------------------------------------------
    // Compute the trajectory once
    double desired_approach_distance = 0.4; // The distance the origin of a robot link needs to travel
    Eigen::Vector3d approach_direction; // Approach direction (negative z axis)
    approach_direction << 0, 0, -1;
    //approach_direction << 0, 0, 1; // Approach direction (positive z axis)

    moveit_msgs::RobotTrajectory trajectory_msg; // the resulting path
    moveit_msgs::RobotTrajectory reverse_trajectory_msg;
    if( !computeStraightLinePath(approach_direction, desired_approach_distance, trajectory_msg,
        reverse_trajectory_msg) )
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Failed to generate straight line path");
      return false;
    }

    bool runTrajectory = true;

    while(ros::ok())
    {
      // -------------------------------------------------------------------------------------------
      ROS_INFO_STREAM_NAMED("verticle_test","Lowering down --------------------------------------");

      // Display the generated path
      if( !visual_tools_->publishTrajectoryPath(trajectory_msg, !runTrajectory ) )
        return false;

      // Record the path
      std::string file_name = file_path_ + "test" + boost::to_string(file_id_) + ".csv";
      file_id_++;
      baxter_record_.startRecording(file_name);

      // Execute the path
      if(runTrajectory)
      {
        executeTrajectoryMsg(trajectory_msg);
      }

      ros::Duration(1).sleep();

      // -------------------------------------------------------------------------------------------
      ROS_INFO_STREAM_NAMED("verticle_test","Raising up -----------------------------------------");

      // Display the generated path
      if( !visual_tools_->publishTrajectoryPath(reverse_trajectory_msg, !runTrajectory ) )
        return false;

      // Execute the path
      if(runTrajectory)
      {
        executeTrajectoryMsg(reverse_trajectory_msg);
      }

      // Save the recorded path to file
      baxter_record_.stopRecording();

      ros::Duration(1).sleep();
    }

    // ---------------------------------------------------------------------------------------------
    // Demo will automatically reset arm
    ROS_INFO_STREAM_NAMED("verticle_test","Finished ------------------------------------------------");

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
    double tolerance_position = 1e-3; // default: 1e-3... meters
    double tolerance_angle = 1e-2; // default 1e-2... radians
    moveit_msgs::Constraints goal_constraint0 = kinematic_constraints::constructGoalConstraints(
      visual_tools_->getEEParentLink(), goal_pose, tolerance_position, tolerance_angle);

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
    visual_tools_->publishArrow(goal_pose.pose, block_grasp_generator::GREEN);
    visual_tools_->publishEEMarkers(goal_pose.pose, block_grasp_generator::GREEN);

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

  /**
   * \brief Function for testing multiple directions
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
    const std::string &ik_link = visual_tools_->getEEParentLink();
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
    //const std::vector<moveit_msgs::JointLimits> &joint_limits = joint_model_group->getVariableLimits();

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
    ROS_INFO_STREAM_NAMED("verticle_test","Executing trajectory");

    plan_execution_->getTrajectoryExecutionManager()->clear();
    if(plan_execution_->getTrajectoryExecutionManager()->push(trajectory_msg))
    {
      plan_execution_->getTrajectoryExecutionManager()->execute();

      // wait for the trajectory to complete
      moveit_controller_manager::ExecutionStatus es = plan_execution_->getTrajectoryExecutionManager()->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        ROS_INFO_STREAM_NAMED("verticle_test","Trajectory execution succeeded");
      else
      {
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          ROS_INFO_STREAM_NAMED("verticle_test","Trajectory execution preempted");
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            ROS_INFO_STREAM_NAMED("verticle_test","Trajectory execution timed out");
          else
            ROS_INFO_STREAM_NAMED("verticle_test","Trajectory execution control failed");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Failed to push trajectory");
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
      ROS_FATAL_STREAM_NAMED("verticle_test","Planning scene not configured");
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
      ROS_INFO_STREAM_NAMED("verticle_test","Waiting for complete state...");
      ros::Duration(0.25).sleep();
      ros::spinOnce();

      // Show unpublished joints
      if( counter > 6 )
      {
        planning_scene_monitor_->getStateMonitor()->haveCompleteState( missing_joints );
        for(int i = 0; i < missing_joints.size(); ++i)
          ROS_WARN_STREAM_NAMED("verticle_test","Unpublished joints: " << missing_joints[i]);
      }
      counter++;
    }

    return true;
  }

}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("verticle_test","Verticle Approach Test");

  ros::init(argc, argv, "verticle_approach_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  baxter_pick_place::VerticleApproachTest tester;

  ROS_INFO_STREAM_NAMED("verticle_test","Shutting down.");

  return 0;
}

