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
  Desc:   Listens to joint_states from Baxter and sends them to a Gazebo controller
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

namespace baxter_gazebo
{

class BaxterMimicPhysical
{
private:
  ros::NodeHandle nh_;

  std::map<std::string,ros::Publisher> joint_command_publishers_;
  ros::Subscriber joint_position_sub_;

  bool publishers_init_;

public:

  BaxterMimicPhysical() :
    publishers_init_(false)
  {
    joint_position_sub_ = nh_.subscribe("/robot/joint_states", 10, &BaxterMimicPhysical::jointPositionCallback, this);

    ROS_WARN_STREAM_NAMED("temp","subscribed");
  }

  ~BaxterMimicPhysical()
  {
  }

  void jointPositionCallback(const sensor_msgs::JointState& msg)
  {
    ROS_INFO_STREAM("Recieved callback for " << msg.name.size() << " joints");

    if(!publishers_init_)
    {
      ROS_INFO_STREAM_NAMED("mimic_physical","Initializing publishers");

      std::string topic_name;

      // Loop through each recieved joint position
      for (std::size_t i = 0; i < msg.name.size(); ++i)
      {
        topic_name = "/baxter/" + msg.name[i] + "_position_controller/command";

        ROS_INFO_STREAM_NAMED("mimic_physical","Creating publisher " << topic_name);
        joint_command_publishers_[msg.name[i]] = nh_.advertise<std_msgs::Float64>(topic_name, 1000);
      }

      // Remember that we've already initialized the publishers for next time
      publishers_init_ = true;
    }

    // Publish to each publisher

    std_msgs::Float64 command;
    for (std::size_t i = 0; i < msg.name.size(); ++i)
    {
      command.data = msg.position[i];
      ROS_INFO_STREAM_NAMED("mimic_physical","Publishing value " << command.data << " to " 
        << msg.name[i]);
      joint_command_publishers_[msg.name[i]].publish(command);
    }
  }

}; // class

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_mimic_physicsal");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO_STREAM_NAMED("mimic_physical","Starting mimic node for baxter");
  baxter_gazebo::BaxterMimicPhysical();

  ros::spin(); // keep the action server alive

  return 0;
}

