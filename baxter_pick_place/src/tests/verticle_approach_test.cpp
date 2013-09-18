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
//#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Grasp generation
#include <block_grasp_generator/block_grasp_generator.h>
//#include <block_grasp_generator/grasp_filter.h>

// Baxter specific properties
#include <baxter_pick_place/baxter_data.h>
#include <baxter_pick_place/custom_environment.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

namespace baxter_pick_place
{

// Static const vars
static const std::string PLANNING_GROUP_NAME = "right_arm";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";

// Class
class VerticleApproachTest
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Action Servers and Clients
  boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > movegroup_action_;

  // MoveIt Components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  boost::shared_ptr<plan_execution::PlanExecution> plan_execution_;

  // class for publishing stuff to rviz
  block_grasp_generator::RobotVizToolsPtr rviz_tools_;

  // data for generating grasps
  block_grasp_generator::RobotGraspData grasp_data_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

public:

  // Constructor
  VerticleApproachTest()
  {
    // -----------------------------------------------------------------------------------------------
    // Connect to move_group action server
    movegroup_action_.reset(new actionlib::SimpleActionClient
      <moveit_msgs::MoveGroupAction>("move_group", true));
    while(!movegroup_action_->waitForServer(ros::Duration(4.0)))
    {
      ROS_INFO_STREAM_NAMED("verticle_test","Waiting for the move_group action server");
    }

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_data_ = loadRobotGraspData(BLOCK_SIZE); // Load robot specific data

    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    if(!loadPlanningSceneMonitor())
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Unable to load planning scene monitor");
      return;
    }

    // ---------------------------------------------------------------------------------------------
    // Create a trajectory execution manager
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager
      (planning_scene_monitor_->getRobotModel()));
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    rviz_tools_.reset(new block_grasp_generator::RobotVizTools(RVIZ_MARKER_TOPIC, baxter_pick_place::EE_GROUP,
        PLANNING_GROUP_NAME, baxter_pick_place::BASE_LINK));
    rviz_tools_->setLifetime(120.0);
    rviz_tools_->setMuted(false);
    rviz_tools_->setGraspPoseToEEFPose(grasp_data_.grasp_pose_to_eef_pose_);

    // ---------------------------------------------------------------------------------------------
    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return;

    // ---------------------------------------------------------------------------------------------
    // Do it
    geometry_msgs::Pose start_pose = createStartPose();
    createVerticleTrajectory(start_pose);

    ROS_INFO_STREAM_NAMED("verticle_test","Success! Waiting 10 sec before shutting down.");
    ros::Duration(10).sleep();

    // Shutdown
    baxter_util_.disableBaxter();
  }

  // Execute series of tasks
  bool createVerticleTrajectory(const geometry_msgs::Pose& start_pose)
  {
    // ---------------------------------------------------------------------------------------------
    // Start Position
    ROS_INFO_STREAM_NAMED("verticle_test","Sending arm to start position ----------------------------------");

    if(!sendPoseCommand(start_pose))
    {
      ROS_ERROR_STREAM_NAMED("verticle_test","Failed to go to start position");
      return false;
    }

    double desired_approach_distance = 0.4; // The distance the origin of a robot link needs to travel

    while(ros::ok())
    {
      // ---------------------------------------------------------------------------------------------
      // Down
      // try to compute a straight line path that arrives at the goal using the specified approach direction
      ROS_INFO_STREAM_NAMED("verticle_test","Lowering down -------------------------------------------");
      Eigen::Vector3d approach_direction; // Approach direction (negative z axis)
      approach_direction << 0, 0, -1;

      if( !computeStraightLinePath(approach_direction, desired_approach_distance) )
      {
        ROS_ERROR_STREAM_NAMED("verticle_test","Failed to follow straight line path");
        return false;
      }

      ros::Duration(1).sleep();

      // ---------------------------------------------------------------------------------------------
      // Up
      // try to compute a straight line path that arrives at the goal using the specified approach direction
      ROS_INFO_STREAM_NAMED("verticle_test","Raising up -------------------------------------------");

      approach_direction << 0, 0, 1; // Approach direction (positive z axis)

      if( !computeStraightLinePath(approach_direction, desired_approach_distance) )
      {
        ROS_ERROR_STREAM_NAMED("verticle_test","Failed to follow straight line path");
        return false;
      }

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
    goal.request.group_name = PLANNING_GROUP_NAME;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = 5.0;

    // -------------------------------------------------------------------------------------------
    // Create goal state
    goal_pose.header.frame_id = baxter_pick_place::BASE_LINK;
    double tolerance_pose = 1e-4; // default: 1e-3... meters
    double tolerance_angle = 1e-2; // default 1e-2... radians
    moveit_msgs::Constraints goal_constraint0 = kinematic_constraints::constructGoalConstraints(
      rviz_tools_->getEEParentLink(), goal_pose, tolerance_pose, tolerance_angle);

    double x_offset = 0;
    ROS_INFO_STREAM_NAMED("verticle_test","Goal pose with x_offset of: " << x_offset << "\n" << goal_pose);

    // Create offset constraint
    goal_constraint0.position_constraints[0].target_point_offset.x = x_offset;
    goal_constraint0.position_constraints[0].target_point_offset.y = 0.0;
    goal_constraint0.position_constraints[0].target_point_offset.z = 0.0;

    // Add offset constraint
    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0] = goal_constraint0;

    // -------------------------------------------------------------------------------------------
    // Visualize goals in rviz
    rviz_tools_->publishArrow(goal_pose.pose, block_grasp_generator::GREEN);
    rviz_tools_->publishEEMarkers(goal_pose.pose, block_grasp_generator::GREEN);

    // -------------------------------------------------------------------------------------------
    // Plan
    movegroup_action_->sendGoal(goal);

    if(!movegroup_action_->waitForResult(ros::Duration(5.0)))
    {
      ROS_INFO_STREAM_NAMED("verticle_test","Returned early?");
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

    ROS_INFO_STREAM_NAMED("verticle_test","Sleeping...");
    ros::Duration(4.0).sleep();

    return true;
  }

  /* Function for testing multiple directions
   * \param approach_direction - direction to move end effector straight
   * \param desired_approach_distance - distance the origin of a robot link needs to travel
   */
  bool computeStraightLinePath( Eigen::Vector3d approach_direction, double desired_approach_distance )
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
    const std::string &ik_link = rviz_tools_->getEEParentLink();

    // Resolution of trajectory
    double max_step = 0.001; // The maximum distance in Cartesian space between consecutive points on the resulting path

    // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
    double jump_threshold = 0.0; // disabled

    // ---------------------------------------------------------------------------------------------
    // Check for kinematic solver
    if( !approach_state.getJointStateGroup(PLANNING_GROUP_NAME)->getJointModelGroup()->canSetStateFromIK( ik_link ) )
    {
      // Set kinematic solver
      const std::pair<robot_model::SolverAllocatorFn, robot_model::SolverAllocatorMapFn> &allocators =
        approach_state.getJointStateGroup(PLANNING_GROUP_NAME)->getJointModelGroup()->getSolverAllocators();
      if( !allocators.first)
        ROS_ERROR_STREAM_NAMED("verticle_test","No IK Solver loaded - make sure moveit_config/kinamatics.yaml is loaded in this namespace");
    }

    // -----------------------------------------------------------------------------------------------
    // Compute Cartesian Path
    ROS_INFO_STREAM_NAMED("verticle_test","Preparing to computer cartesian path");

    /** \brief Compute the sequence of joint values that correspond to a Cartesian path.

        The Cartesian path to be followed is specified as a direction of motion (\e direction, unit vector) for the origin of a robot
        link (\e link_name). The direction is assumed to be either in a global reference frame or in the local reference frame of the
        link. In the latter case (\e global_reference_frame is true) the \e direction is rotated accordingly. The link needs to move in a
        straight line, following the specified direction, for the desired \e distance. The resulting joint values are stored in
        the vector \e states, one by one. The maximum distance in Cartesian space between consecutive points on the resulting path
        is specified by \e max_step.  If a \e validCallback is specified, this is passed to the internal call to|
        setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to the distance that
        was computed and for which corresponding states were added to the path.  At the end of the function call, the state of the
        group corresponds to the last attempted Cartesian pose.  During the computation of the trajectory, it is sometimes prefered if
        consecutive joint values do not 'jump' by a large amount in joint space, even if the Cartesian distance between the
        corresponding points is as expected. To account for this, the \e jump_threshold parameter is provided.  As the joint values
        corresponding to the Cartesian path are computed, distances in joint space between consecutive points are also computed. Once
        the sequence of joint values is computed, the average distance between consecutive points (in joint space) is also computed. It
        is then verified that none of the computed distances is above the average distance by a factor larger than \e jump_threshold. If
        a point in joint is found such that it is further away than the previous one by more than average_consecutive_distance * \e jump_threshold,
        that is considered a failure and the returned path is truncated up to just before the jump. The jump detection can be disabled
        by settin \e jump_threshold to 0.0
        double computeCartesianPath(std::vector<boost::shared_ptr<RobotState> > &traj, const std::string &link_name, const Eigen::Vector3d &direction, bool global_reference_frame,
        ............................double distance, double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback = StateValidityCallbackFn());
    */

    std::vector<robot_state::RobotStatePtr> approach_traj_result; // create resulting generated trajectory (result)

    double d_approach =
      approach_state.getJointStateGroup(PLANNING_GROUP_NAME)->computeCartesianPath(approach_traj_result,
        ik_link,                   // link name
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
    // Get current RobotState  (in order to specify all joints not in approach_traj_result)
    //robot_state::RobotState this_robot_state = planning_scene->getCurrentState();

    // -----------------------------------------------------------------------------------------------
    // Smooth the path and add velocities/accelerations

    trajectory_processing::IterativeParabolicTimeParameterization iterative_smoother;
    trajectory_msgs::JointTrajectory trajectory_out;

    // Get the joint limits of planning group
    const robot_model::JointModelGroup *joint_model_group =
      planning_scene->getRobotModel()->getJointModelGroup(PLANNING_GROUP_NAME);
    const std::vector<moveit_msgs::JointLimits> &joint_limits = joint_model_group->getVariableLimits();

    // Copy the vector of RobotStates to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr approach_traj(new robot_trajectory::RobotTrajectory(
        planning_scene->getRobotModel(), PLANNING_GROUP_NAME));
    for (std::size_t k = 0 ; k < approach_traj_result.size() ; ++k)
      approach_traj->addSuffixWayPoint(approach_traj_result[k], 0.0);

    // Perform iterative parabolic smoothing
    iterative_smoother.computeTimeStamps( *approach_traj );
    /*                                         approach_traj,
                                               trajectory_out,
                                               joint_limits,
                                               this_robot_state // start_state
                                               );
    */

    ROS_INFO_STREAM("New trajectory\n" << approach_traj);

    // -----------------------------------------------------------------------------------------------
    // Display the path in rviz

    // Create publisher
    ros::Publisher display_path_publisher_;
    display_path_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>
      ("/move_group/display_planned_path", 10, true);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    // Create the message
    moveit_msgs::DisplayTrajectory rviz_display;
    rviz_display.model_id = planning_scene->getRobotModel()->getName();
    //    rviz_display.trajectory_start = this_robot_state;
    //    rviz_display.trajectory.resize(1, approach_traj_result);

    robot_state::robotStateToRobotStateMsg(approach_traj->getFirstWayPoint(), rviz_display.trajectory_start);
    rviz_display.trajectory.resize(1);
    approach_traj->getRobotTrajectoryMsg(rviz_display.trajectory[0]);

    // Publish message
    display_path_publisher_.publish(rviz_display);
    ROS_INFO_STREAM_NAMED("verticle_test","Sent display trajectory message");

    ROS_INFO_STREAM_NAMED("verticle_test","Sleeping 5...\n\n");
    ros::Duration(5.0).sleep();

    // -----------------------------------------------------------------------------------------------
    // Execute the planned trajectory
    ROS_INFO_STREAM_NAMED("verticle_test","Executing trajectory");

    // Convert trajectory to a message
    moveit_msgs::RobotTrajectory traj_msg;
    approach_traj->getRobotTrajectoryMsg(traj_msg);

    plan_execution_->getTrajectoryExecutionManager()->clear();
    if(plan_execution_->getTrajectoryExecutionManager()->push(traj_msg))
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
    start_pose.position.y = -0.4;
    start_pose.position.z = 0.2; // torso

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
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      //planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
      //  "dave_planning_scene");
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("verticle_test","Planning scene not configured");
      return false;
    }

    ros::spinOnce();
    ros::Duration(0.5).sleep();
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

