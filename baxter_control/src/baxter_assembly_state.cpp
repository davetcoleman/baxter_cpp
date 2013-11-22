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

/* Author: Dave Coleman
   Desc:   Publishes fake assembly state messages
*/

#include <ros/ros.h>

// Baxter
#include <baxter_core_msgs/AssemblyState.h>

namespace baxter_control
{

static const std::string BAXTER_STATE_TOPIC = "/robot/state";

class BaxterAssemblyState
{
private:
  ros::Publisher assembly_state_pub_;

  // Rate to publish assembly state
  ros::Timer timer_;
  
  // Cache the message
  baxter_core_msgs::AssemblyState assembly_state_;

  ros::NodeHandle nh_;

public:

  BaxterAssemblyState()
  {
    // Start a publisher that publishes fake AssemblyState.msg data about Baxter
    assembly_state_pub_ = nh_.advertise<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC,10);

    // Create assembly state message 
    assembly_state_.enabled = 1;             // true if enabled
    assembly_state_.stopped = 0;            // true if stopped -- e-stop asserted
    assembly_state_.error = 0;              // true if a component of the assembly has an error
    assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;      // button status
    assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;     // If stopped is true, the source of the e-stop.  

    // Set publish frequency
    ros::NodeHandle nh_tilde("~");
    double publish_freq;
    nh_tilde.param("publish_frequency", publish_freq, 50.0);
    ros::Duration publish_interval = ros::Duration(1.0/std::max(publish_freq,1.0));

    // trigger to publish fixed joints
    timer_ = nh_tilde.createTimer(publish_interval, &BaxterAssemblyState::update, this);

  }

  void update(const ros::TimerEvent& e)
  {
    assembly_state_pub_.publish(assembly_state_);
  }

};

} // namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("baxter_assembly_state","Starting publisher...");

  ros::init(argc, argv, "baxter_assembly_state");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  baxter_control::BaxterAssemblyState baxter;

  ros::spin();

  return 0;
}
