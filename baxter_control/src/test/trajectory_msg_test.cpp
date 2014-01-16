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

  // Joint to move
  std::size_t joint_id_;

  std::string arm_name_;

  static const double WAYPOINT_INTERVALS = 0.4;

public:

  /**
   * \brief Constructor
   */
  TrajectoryMsgTest()
    : arm_name_("left"),
      joint_id_(1)
  {
    srand (time(NULL));

    // Listen to joint positions
    sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, 
                       &TrajectoryMsgTest::stateCallback, this);

    // Publish trajectories
    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
      "/robot/"+arm_name_+"_joint_position_trajectory_controller/command",10);

    // Wait for first state message to be recieved
    while(ros::ok() && state_msg_timestamp_.toSec() == 0)
    {
      ROS_INFO_STREAM_NAMED(arm_name_,"Waiting for first state message to be recieved");
      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }

    sendTrajectory();
  }

  /**
   * \brief Destructor
   */
  ~TrajectoryMsgTest()
  {
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

    // Add initial point - our current state
    trajectory_msgs::JointTrajectoryPoint current_state = getCurrentPosition();
    msg_.points.push_back(current_state);

    //ROS_DEBUG_STREAM_NAMED("temp","current position2:\n" << current_state);

    // Settings for a joint "a"
    double max_a = 3.05417993878 - 1;
    double min_a = -3.05417993878 + 1;
    int num_points = 10;
    double start_a = current_state.positions[joint_id_];
    double end_a; 
    double diff_a;
    do
    {
      end_a = fRand(min_a, max_a);
      diff_a = end_a - start_a;
    } while ( abs(diff_a) < 0.5 );
    double increment_a = diff_a / num_points;

    // Generate
    for (std::size_t i = 0; i < num_points; ++i)
    {
      msg_.points.push_back(createPoint(msg_.points.back(), increment_a));
    }

    ROS_DEBUG_STREAM_NAMED("temp","sending traj message:\n" << msg_);
    ROS_INFO_STREAM_NAMED("trajectory_test","Moving joint " << msg_.joint_names[joint_id_]
      << " from " << start_a << " to position " << end_a);
    ROS_INFO_STREAM_NAMED("temp","diff a = " << diff_a << " increment a = " << increment_a);

    traj_pub_.publish(msg_);
    ros::spinOnce();
    sleep(5);
  }

  void sendTrajectoryOfZeros()
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

    // Add initial point - our current state
    trajectory_msgs::JointTrajectoryPoint current_state = getCurrentPosition();
    msg_.points.push_back(current_state);

    // Generate
    bool hasNonZero = true; // flag for when to stop moving joints
    while( hasNonZero )
    {
      hasNonZero = false;

      // Copy last point to new point
      trajectory_msgs::JointTrajectoryPoint point = msg_.points.back();

      static const double STEP_SIZE = 0.1;

      // Move every position in point closer to zero
      for (std::size_t j = 0; j < point.positions.size(); ++j)
      {
        if (point.positions[j] > 0)
        {
          point.positions[j] -= STEP_SIZE;
          if (point.positions[j] < 0)
            point.positions[j] = 0;
        }
        else
        {
          point.positions[j] += STEP_SIZE;          
          if (point.positions[j] > 0)
            point.positions[j] = 0;
        }

        // Check if we need to stop generating points
        if (point.positions[j] != 0)
          hasNonZero = true;
      }

      point.time_from_start += ros::Duration(WAYPOINT_INTERVALS);
      msg_.points.push_back(point);

      ROS_DEBUG_STREAM_NAMED("temp","end of adding new point loop");
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

    point.positions[joint_id_] += increment_a;
    point.time_from_start = point.time_from_start + ros::Duration(WAYPOINT_INTERVALS);

    return point;
  }

  trajectory_msgs::JointTrajectoryPoint getCurrentPosition()
  {
    trajectory_msgs::JointTrajectoryPoint point;

    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      ROS_ERROR_STREAM_NAMED("temp","getCurrentPosition() function has not been updated!");
      exit(99);
      point.positions.push_back(state_msg_->position[i]); // TODO: this line is wrong, need to better filter the joints from the message
      point.velocities.push_back(0.0);
    }

    point.time_from_start = ros::Duration(WAYPOINT_INTERVALS);

    return point;
  }

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    // Copy the latest message into a buffer
    state_msg_ = msg;
    state_msg_timestamp_ = ros::Time::now();
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
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

