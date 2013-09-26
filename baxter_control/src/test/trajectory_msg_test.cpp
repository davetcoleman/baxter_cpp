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
   Desc:   Sends out simple trajectory messages as a test
*/

// ROS
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

namespace ros_control
{

class TrajectoryMsgTest
{
private:

  // Number of joints we are using
  unsigned int n_dof_;

  // A shared node handle
  ros::NodeHandle nh_;

  // Output
  trajectory_msgs::JointTrajectory msg_;

  // Buffer of joint states
  sensor_msgs::JointStateConstPtr state_msg_;
  ros::Time state_msg_timestamp_;

  // Subscriber
  ros::Subscriber sub_joint_state_;
  ros::Publisher traj_pub_;

  std::string arm_name_;
  
public:

  /**
   * \brief Constructor
   */
  TrajectoryMsgTest()
    : arm_name_("right")
  {
    // Listen to joint positions
    sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/robot/limb/" + arm_name_ +
                       "/joint_states", 1, &TrajectoryMsgTest::stateCallback, this);

    // Publish trajectories
    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
      "/robot/right_joint_position_trajectory_controller/command",10);

    // Wait for first state message to be recieved
    while(ros::ok() && state_msg_timestamp_.toSec() == 0)
    {
      ROS_INFO_STREAM_NAMED(arm_name_,"Waiting for first state message to be recieved");
      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }

    sendTrajectory();
  }

  void sendTrajectory()
  {
    msg_.header.stamp = ros::Time::now();

    msg_.joint_names.push_back(arm_name_+"_e0");
    msg_.joint_names.push_back(arm_name_+"_e1");
    msg_.joint_names.push_back(arm_name_+"_s0");
    msg_.joint_names.push_back(arm_name_+"_s1");
    msg_.joint_names.push_back(arm_name_+"_w0");
    msg_.joint_names.push_back(arm_name_+"_w1");
    msg_.joint_names.push_back(arm_name_+"_w2");
    n_dof_ = msg_.joint_names.size();

    // Settings
    int num_points = 2;
    double start_a = 0; double end_a = -1;

    // Math
    double diff_a = start_a - end_a;
    double increment_a = diff_a / num_points;
    ROS_DEBUG_STREAM_NAMED("temp","diff a = " << diff_a);
    ROS_DEBUG_STREAM_NAMED("temp","increment a = " << increment_a);

    // Add initial point - our current state
    msg_.points.push_back(getCurrentPosition());

    // Generate
    for (std::size_t i = 0; i < num_points; ++i)
    {
      msg_.points.push_back(createPoint(msg_.points.back(), increment_a));
    }

    ROS_DEBUG_STREAM_NAMED("temp","sending traj message:\n" << msg_);

    traj_pub_.publish(msg_);
    ros::spinOnce();
    sleep(5);
  }

  trajectory_msgs::JointTrajectoryPoint createPoint(trajectory_msgs::JointTrajectoryPoint prev, 
    double increment_a)
  {
    trajectory_msgs::JointTrajectoryPoint point = prev;

    point.positions[0] += increment_a;
    point.time_from_start = point.time_from_start + ros::Duration(2.0);

    return point;
  }

  trajectory_msgs::JointTrajectoryPoint getCurrentPosition()
  {
    trajectory_msgs::JointTrajectoryPoint point;
    
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      point.positions.push_back(state_msg_->position[i]);      
    }

    point.time_from_start = point.time_from_start + ros::Duration(2.0);

    ROS_DEBUG_STREAM_NAMED("temp","current position:\n" << point);

    return point;
  }

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    // Copy the latest message into a buffer
    state_msg_ = msg;
    state_msg_timestamp_ = ros::Time::now();
  }

  /**
   * \brief Destructor
   */
  ~TrajectoryMsgTest()
  {

  }

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("trajectory_test","Trajectory Message Test");

  ros::init(argc, argv, "trajectory_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros_control::TrajectoryMsgTest tester;

  ROS_INFO_STREAM_NAMED("trajectory_test","Shutting down.");

  return 0;
}

