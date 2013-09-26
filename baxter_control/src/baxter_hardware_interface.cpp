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
   Desc:   ros_control hardware interface layer for Baxter
*/

#include <baxter_control/baxter_hardware_interface.h>

namespace baxter_control
{

BaxterHardwareInterface::BaxterHardwareInterface()
  : right_arm_hw_("right")
{
  // Initialize right arm
  right_arm_hw_.init(js_interface_, ej_interface_, vj_interface_, pj_interface_);

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&pj_interface_);

  // Enable baxter
  if( !baxter_util_.enableBaxter() )
  {
    ROS_ERROR_STREAM_NAMED("hardware_interface","Unable to enable Baxter");
    exit(0);
  }

  ROS_INFO_NAMED("hardware_interface", "Loaded baxter_hardware_interface.");
}

BaxterHardwareInterface::~BaxterHardwareInterface()
{
  baxter_util_.disableBaxter();
}

void BaxterHardwareInterface::read()
{
  right_arm_hw_.read();
}

void BaxterHardwareInterface::write()
{
  right_arm_hw_.write();
}

} // namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface","Starting hardware interface...");

  ros::init(argc, argv, "baxter_hardware_interface");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  baxter_control::BaxterHardwareInterface baxter;

  // Create the controller manager
  controller_manager::ControllerManager controller_manager(&baxter, nh);
  ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");

  double total_sec = 0;
  std::size_t total_samples = 0;

  // Update controllers
  ros::Rate rate(100); // 50 hz
  while (ros::ok())
  {
    baxter.read();
    controller_manager.update(ros::Time::now(), rate.cycleTime() ); // get the actual run time of a cycle from start to ros::Duration(5.0).sleep();
    baxter.write();

    total_sec += rate.cycleTime().toSec();
    total_samples++;
    if( total_samples % 100 == 0 )
      ROS_INFO_STREAM_NAMED("hardware_interface", "Avg Rate:" << 1/(total_sec/total_samples)/100 << " hz");

    rate.sleep();
  }

  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

  return 0;
}


