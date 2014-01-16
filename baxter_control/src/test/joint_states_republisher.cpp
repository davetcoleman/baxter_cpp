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
   Desc:   Filters joint state messages for the ones coming directly from Baxter, not the finger ones
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace ros_control
{

class JointStatesRepublisher
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Buffer of joint states
  sensor_msgs::JointStateConstPtr state_msg_;

  // Subscriber
  ros::Subscriber sub_joint_state_;
  ros::Publisher pub_joint_state_;

public:

  /**
   * \brief Constructor
   */
  JointStatesRepublisher()
  {
    // Listen to joint positions
    sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, 
                       &JointStatesRepublisher::stateCallback, this);

    // Publish trajectories
    pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_states_filtered",10);      

    ROS_INFO_STREAM_NAMED("js_republisher","Starting to filter and republish joint_state messages");
  }

  /**
   * \brief Destructor
   */
  ~JointStatesRepublisher()
  {
  }

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    // Copy the latest message into a buffer
    state_msg_ = msg;

    static const int JOINT_STATE_COUNT_FILTER = 4; // republish joint states with more than this number of joints
        
    // Republish the joint state if not the finger one
    if (state_msg_->name.size() > JOINT_STATE_COUNT_FILTER)
    {
      pub_joint_state_.publish(state_msg_);
    }
  }

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("js_republisher","Joint State Filter Republisher");

  ros::init(argc, argv, "js_republisher");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros_control::JointStatesRepublisher tester;
  ros::spin();

  ROS_INFO_STREAM_NAMED("js_republisher","Shutting down.");

  return 0;
}

