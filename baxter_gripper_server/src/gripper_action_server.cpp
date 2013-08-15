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
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <baxter_msgs/GripperState.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommandAction.h>

namespace baxter_gripper_action
{

static const std::string GRIPPER_COMMAND_ACTION_TOPIC="baxter_gripper_action";
static const std::string BASE_LINK = "base"; //"/base";

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
  ros::Publisher joint_state_topic_;
  ros::Subscriber gripper_state_sub_;

  // Action messages
  control_msgs::GripperCommandResult action_result_;

  // Cache an empty message
  std_msgs::Empty empty_msg_;
  std_msgs::Float32 zero_msg_;

  // Remeber the last gripper state
  baxter_msgs::GripperStateConstPtr gripper_state_;
  ros::Time gripper_state_timestamp_;

  // Using Gazebo or not
  bool in_simulation_;

  // Remember which arm this class is for
  std::string arm_name_;

public:

  // Constructor
  GripperActionServer(const std::string name, const std::string arm_name, const bool in_simulation)
    : action_server_(nh_, name, false),
      arm_name_(arm_name),
      in_simulation_(in_simulation)
  {
    ROS_INFO_STREAM_NAMED(arm_name_, "Baxter Gripper Action Server starting");

    // Start the publishers
    ROS_DEBUG_STREAM_NAMED(arm_name_,"Starting gripper calibration publisher");
    calibrate_topic_ = nh_.advertise<std_msgs::Empty>("/robot/limb/" + arm_name_
                       + "/accessory/gripper/command_calibrate",10);

    ROS_DEBUG_STREAM_NAMED(arm_name_,"Starting gripper close publisher");
    position_topic_ = nh_.advertise<std_msgs::Float32>("/robot/limb/" + arm_name_
                      + "/accessory/gripper/command_grip",10);

    ROS_DEBUG_STREAM_NAMED(arm_name_,"Starting gripper open publisher");
    release_topic_ = nh_.advertise<std_msgs::Empty>("/robot/limb/" + arm_name_
                     + "/accessory/gripper/command_release",10);

    ROS_DEBUG_STREAM_NAMED(arm_name_,"Starting gripper joint state subscriber");
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
      ROS_DEBUG_STREAM_NAMED(arm_name_,"Starting gripper joint state publisher");
      joint_state_topic_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_states",10);
    }

    // Register the goal and start
    action_server_.registerGoalCallback(boost::bind(&GripperActionServer::goalCB, this));
    action_server_.start();

    // Cache zero command
    zero_msg_.data = 0;

    // Calibrate if needed
    calibrate();

    // Announce state
    ROS_INFO_STREAM_NAMED(arm_name_, "Baxter Gripper Action Server ready.");
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
      ROS_INFO_STREAM_NAMED(arm_name_,"Done calibrating gripper");
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

  void update()
  {
    // Gazebo publishes a joint state for the gripper, but Baxter does not do so in the right format
    if( in_simulation_ )
      return;

    sensor_msgs::JointState state;
    state.header.stamp = gripper_state_timestamp_;
    state.header.frame_id = BASE_LINK;
    state.name.push_back(arm_name_ + "_gripper_l_finger_joint");
    state.position.push_back(gripper_state_->position);
    state.velocity.push_back(0);
    state.effort.push_back(gripper_state_->force);

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
      return true;
    }
    if( gripper_state_->error )
    {
      ROS_ERROR_STREAM_NAMED(arm_name_,"Gripper " << arm_name_ << " has error. State: \n" << *gripper_state_ );
      action_server_.setAborted(action_result_,"Gripper has error");
      return true;
    }

    return false;
  }

  // Action server sends goals here
  void goalCB()
  {
    double position = action_server_.acceptNewGoal()->command.position;

    ROS_INFO_STREAM_NAMED(arm_name_,"Recieved goal for command position: " << position);

    // Error check gripper
    if( hasError() )
      return;


    // Open command
    if(position > 50)
    {
      openGripper();
    }
    else // Close command
    {
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

  /*
    feedback_.status = "Sending goal to move_group action server";
    if(action_server_.isActive()) // Make sure we haven't sent a fake goal
    action_server_.publishFeedback(feedback_);
  */


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

  baxter_gripper_action::GripperActionServer server("baxter_gripper_action/gripper_action",
    "right", in_simulation);

  // Run optional test
  if(run_test)
  {
    server.runTest();
  }

  // Publish joint states if needed
  while(ros::ok())
  {
    server.update();
    ros::Duration(0.1).sleep();
  }

  return 0;
}

