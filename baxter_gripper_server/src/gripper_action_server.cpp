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
  Desc:   Provides an action server for baxter's grippers
*/

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <baxter_msgs/GripperState.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommandAction.h>

namespace baxter_gripper_action
{

static const std::string GRIPPER_COMMAND_ACTION_TOPIC="baxter_gripper_action";
static const std::string BASE_LINK = "base"; //"/base";

// Copied from URDF \todo read straight from URDF? 
static const double GRIPPER_FINGER_JOINT_UPPER = 0.0095; //open
static const double GRIPPER_FINGER_JOINT_LOWER = -0.0125; //close

class GripperActionServer
{
protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // Action Server
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server_;

  // Publisher
  ros::Publisher calibrate_topic_;
  ros::Publisher position_topic_;
  ros::Publisher release_topic_;
  ros::Publisher reset_topic_;
  ros::Publisher joint_state_topic_;
  ros::Subscriber gripper_state_sub_;

  // Action messages
  control_msgs::GripperCommandResult action_result_;

  // Cache an empty message
  std_msgs::Empty empty_msg_;
  std_msgs::Bool bool_msg_;
  std_msgs::Float32 zero_msg_;

  // Remember the last gripper state and time
  baxter_msgs::GripperStateConstPtr gripper_state_;
  ros::Time gripper_state_timestamp_;

  bool in_simulation_; // Using Gazebo or not
  double gripper_finger_joint_stroke_; // cache the diff between upper and lower limits
  std::string arm_name_; // Remember which arm this class is for

  // Rate to publish joint states
  ros::Timer timer_;

public:

  // Constructor
  GripperActionServer(const std::string action_name, const std::string arm_name, const bool in_simulation)
    : action_server_(nh_, action_name, false),
      arm_name_(arm_name),
      in_simulation_(in_simulation)
  {
    ROS_INFO_STREAM_NAMED(arm_name_, "Baxter " + arm_name + " Gripper Action Server starting");

    // Start the publishers
    calibrate_topic_ = nh_.advertise<std_msgs::Empty>("/robot/limb/" + arm_name_
                       + "/accessory/gripper/command_calibrate",10);

    position_topic_ = nh_.advertise<std_msgs::Float32>("/robot/limb/" + arm_name_
                      + "/accessory/gripper/command_grip",10);

    release_topic_ = nh_.advertise<std_msgs::Empty>("/robot/limb/" + arm_name_
                     + "/accessory/gripper/command_release",10);

    reset_topic_ = nh_.advertise<std_msgs::Bool>("/robot/limb/" + arm_name_
                     + "/accessory/gripper/command_reset",10);

    // Start the subscriber
    gripper_state_sub_ = nh_.subscribe<baxter_msgs::GripperState>("/sdk/robot/limb/" + arm_name_
                         + "/accessory/gripper/state",
                         1, &GripperActionServer::stateCallback, this);

    // Decide if we are in simulation based on the existence of the gripper state message
    if( !in_simulation )
    {
      int count = 0;
      while( ros::ok() && gripper_state_timestamp_.toSec() == 0 )
      {
        if( count > 20 ) // 20 is an arbitrary number for when to assume we are in simulation mode
        {
          ROS_INFO_STREAM_NAMED(arm_name_,"Assuming Baxter is in simulaion mode because unable to get gripper state");
          in_simulation_ = true;
          break;
        }

        ++count;
        ros::Duration(0.05).sleep();
      }
    }

    // If in simulation, fill in dummy state values
    if( in_simulation )
    {
      baxter_msgs::GripperStatePtr simulation_state_;
      simulation_state_.reset(new baxter_msgs::GripperState());
      simulation_state_->enabled = 1;
      simulation_state_->calibrated = 1;
      simulation_state_->ready = 1;
      simulation_state_->moving = 0;
      simulation_state_->gripping = 0;
      simulation_state_->missed = 0;
      simulation_state_->error = 0;
      simulation_state_->command = 0; // \todo
      simulation_state_->position = 0; // \todo
      simulation_state_->force = 7; // base line value unloaded
      gripper_state_ = simulation_state_;
    }

    // Gazebo publishes a joint state for the gripper, but Baxter does not do so in the right format
    if( !in_simulation_ )
    {
      joint_state_topic_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_states",10);
    }

    // Register the goal and start
    action_server_.registerGoalCallback(boost::bind(&GripperActionServer::goalCB, this));
    action_server_.start();

    // Cache zero command
    zero_msg_.data = 0;

    // Calculate joint stroke
    gripper_finger_joint_stroke_ = GRIPPER_FINGER_JOINT_UPPER - GRIPPER_FINGER_JOINT_LOWER;

    // Reset error just in case
    resetError();

    // Calibrate if needed
    calibrate();

    // Set publish frequency
    ros::NodeHandle nh_tilde("~");
    double publish_freq;
    nh_tilde.param("publish_frequency", publish_freq, 50.0);
    ros::Duration publish_interval = ros::Duration(1.0/std::max(publish_freq,1.0));

    // trigger to publish fixed joints
    timer_ = nh_tilde.createTimer(publish_interval, &GripperActionServer::update, this);

    // Announce state
    ROS_INFO_STREAM_NAMED(arm_name_, "Baxter " + arm_name + " Gripper Action Server ready.");
  }

  void calibrate()
  {
    if( in_simulation_ )
      return;

    // Calibrate if needed
    if( !gripper_state_->calibrated )
    {
      ROS_INFO_STREAM_NAMED(arm_name_,"Calibrating gripper");
      calibrate_topic_.publish(empty_msg_);
      ros::Duration(2.0).sleep();
    }
  }

  void stateCallback(const baxter_msgs::GripperStateConstPtr& msg)
  {
    gripper_state_ = msg;
    gripper_state_timestamp_ = ros::Time::now();
  }

  void runTest()
  {
    // Error check gripper
    hasError();

    bool open = true;
    while(ros::ok())
    {
      if(open)
      {
        openGripper();
        open = false;
      }
      else
      {
        closeGripper();
        open = true;
      }
      ros::Duration(2.0).sleep();
    }
  }

  void update(const ros::TimerEvent& e)
  {
    // Gazebo publishes a joint state for the gripper, but Baxter does not do so in the right format
    if( in_simulation_ )
      return;

    // Create state message
    sensor_msgs::JointState state;
    state.header.stamp = gripper_state_timestamp_;
    state.header.frame_id = BASE_LINK;
    state.name.push_back(arm_name_ + "_gripper_l_finger_joint");
    state.velocity.push_back(0);
    state.effort.push_back(gripper_state_->force);

    // Convert 0-100 state to joint position
    state.position.push_back(GRIPPER_FINGER_JOINT_LOWER + gripper_finger_joint_stroke_ * 
      (gripper_state_->position / 100));

    joint_state_topic_.publish(state);
  }

  /**
   * \brief Check if gripper is in good state
   * \param checkReady - flag if the ready state should be checked. It should not be checked during
   *                     operation
   * \return true if there is no error
   */
  bool hasError(bool checkReady = true)
  {
    // Populate these now in case an error is detected below
    action_result_.position = gripper_state_->position;
    action_result_.effort = gripper_state_->force;
    action_result_.stalled = false; // \todo implement
    action_result_.reached_goal = false;

    // Run Checks
    if( !in_simulation_ &&
      ros::Time::now() > gripper_state_timestamp_ + ros::Duration(1.0)) // check that the message timestamp is no older than 1 second
    {
      ROS_ERROR_STREAM_NAMED(arm_name_,"Gripper " << arm_name_ << " state expired. State: \n" << *gripper_state_ );
      action_server_.setAborted(action_result_,std::string("Gripper state expired"));
      return true;
    }
    if( !gripper_state_->enabled )
    {
      ROS_ERROR_STREAM_NAMED(arm_name_,"Gripper " << arm_name_ << " not enabled. State: \n" << *gripper_state_ );
      action_server_.setAborted(action_result_,"Gripper not enabled");
      return true;
    }
    if( !gripper_state_->calibrated )
    {
      ROS_ERROR_STREAM_NAMED(arm_name_,"Gripper " << arm_name_ << " not calibrated. State: \n" << *gripper_state_ );
      action_server_.setAborted(action_result_,"Gripper not calibrated");
      return true;
    }
    if( checkReady && !gripper_state_->ready )
    {
      ROS_ERROR_STREAM_NAMED(arm_name_,"Gripper " << arm_name_ << " not ready. State: \n" << *gripper_state_ );
      action_server_.setAborted(action_result_,"Gripper not ready");

      // Attempt to fix error
      ROS_WARN_STREAM_NAMED(arm_name_,"Attempting to auto fix");
      resetError();

      return true;
    }
    if( gripper_state_->error )
    {
      ROS_ERROR_STREAM_NAMED(arm_name_,"Gripper " << arm_name_ << " has error. State: \n" << *gripper_state_ );
      action_server_.setAborted(action_result_,"Gripper has error");

      // Attempt to fix error
      ROS_WARN_STREAM_NAMED(arm_name_,"Attempting to auto fix");
      resetError();

      return true;
    }

    return false;
  }

  void resetError()
  {
    ROS_INFO_STREAM_NAMED(arm_name_,"Resetting gripper");

    bool_msg_.data = true;
    reset_topic_.publish(bool_msg_);
    ros::Duration(0.5).sleep();

    calibrate();
  }

  // Action server sends goals here
  void goalCB()
  {
    double position = action_server_.acceptNewGoal()->command.position;

    ROS_INFO_STREAM_NAMED(arm_name_,"Recieved goal for command position: " << position);

    // Open command
    if(position > 50)
    {
      // Error check gripper
      if( hasError(false) ) // don't check ready bit when opening
        return;

      openGripper();
    }
    else // Close command
    {
      // Error check gripper
      if( hasError(true) ) // check ready bit when closing
        return;

      closeGripper();
    }

    // Report success
    action_result_.position = gripper_state_->position;
    action_result_.effort = gripper_state_->force;
    action_result_.stalled = false; // \todo implement
    action_result_.reached_goal = true;
    action_server_.setSucceeded(action_result_,"Success");
  }

  bool openGripper()
  {
    ROS_INFO_STREAM_NAMED(arm_name_,"Opening gripper");

    // Send command several times to be safe
    for (std::size_t i = 0; i < 4; ++i)
    {
      release_topic_.publish(empty_msg_);
      ros::Duration(0.1).sleep();
      ros::spinOnce(); //todo remove
    }

    // Error check gripper
    if( hasError(false) )
      return false;

    return true;
  }

  bool closeGripper()
  {
    ROS_INFO_STREAM_NAMED(arm_name_,"Closing gripper");

    // Send command several times to be safe
    for (std::size_t i = 0; i < 4; ++i)
    {
      position_topic_.publish(zero_msg_);
      ros::Duration(0.1).sleep();
      ros::spinOnce(); //todo remove
    }

    // Error check gripper
    if( hasError(false) )
      return false;

    return true;
  }

}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_gripper_action");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool in_simulation = false;
  bool run_test = false;

  // Parse command line arguments
  for (std::size_t i = 0; i < argc; ++i)
  {
    if( std::string(argv[i]).compare("--simulation") == 0 )
    {
      ROS_INFO_STREAM_NAMED("main","Gripper action server in simulation mode");
      in_simulation = true;
    }
    else if( std::string(argv[i]).compare("--test") == 0 )
    {
      ROS_INFO_STREAM_NAMED("main","Gripper action server running test");
      run_test = true;
    }
  }

  baxter_gripper_action::GripperActionServer right_server("baxter_right_gripper_action/gripper_action",
    "right", in_simulation);

  baxter_gripper_action::GripperActionServer left_server("baxter_left_gripper_action/gripper_action",
    "left", in_simulation);

  // Run optional test
  if(run_test)
  {
    right_server.runTest();
  }

  ros::spin();

  return 0;
}

