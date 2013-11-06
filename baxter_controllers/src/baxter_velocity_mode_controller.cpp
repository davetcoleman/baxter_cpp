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
   Desc:   a custom interface for switching mode (this would simply be a function call in your robot class like: void switch(int mode)
*/

#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <baxter_control/baxter_hardware_interface.h>

namespace baxter_controllers {

class BaxterVelocityModeController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{

private:


public:
  BaxterVelocityModeController()
  {}

  ~BaxterVelocityModeController()
  {
  }

  bool init(
    hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &nh)
  {
    // This is the only purpose of this controller
    baxter_control::BaxterHardwareInterface *bhi;
    bhi = (baxter_control::BaxterHardwareInterface*) robot;
    bhi->armModeSwitch(baxter_control::VELOCITY);
    ROS_ERROR_STREAM_NAMED("temp","done");

    return true;
  }

  /*
  void starting(const ros::Time& time)
  {

  }

  void stopping(const ros::Time& time)
  {

  }
  */

  void update(const ros::Time& time, const ros::Duration& period)
  {

  }

};

} // namespace

PLUGINLIB_EXPORT_CLASS(
  baxter_controllers::BaxterVelocityModeController,
  controller_interface::ControllerBase)
