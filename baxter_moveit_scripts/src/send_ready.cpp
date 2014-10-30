/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CU Boulder
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
 *   * Neither the name of CU Boulder nor the names of its
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

/**
 * \brief   Send both arms to ready position
 * \author  Dave Coleman
 */

// ROS
#include <ros/ros.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>
#include <baxter_pick_place/baxter_move_group_interface.h>

namespace baxter_experimental
{

//static const std::string PLANNING_GROUP_BOTH = "both_arms";
static const std::string PLANNING_GROUP_BOTH = "left_arm";

class SendReady
{
private:

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;
  baxter_pick_place::BaxterMoveGroupInterfacePtr baxter_move_;

  // settings
  bool verbose_; // for debugging

public:
  SendReady(bool verbose, std::string planning_group)
    : verbose_(verbose)
  {
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    // Enable
    baxter_util_.enableBaxter();

    while(ros::ok())
    {
      // Move baxter to neutral
      if (planning_group == "both_arms")
      {
        // Load baxter move group interface helper    
        baxter_move_.reset(new baxter_pick_place::BaxterMoveGroupInterface("both_arms"));

        // Send
        baxter_move_->sendToPose("both_neutral");
      }
      else if (planning_group == "left_arm")
      {
        // Load baxter move group interface helper    
        baxter_move_.reset(new baxter_pick_place::BaxterMoveGroupInterface("left_arm"));

        // Send
        baxter_move_->sendToPose("left_neutral");
      }
      else if (planning_group == "right_arm")
      {
        // Load baxter move group interface helper    
        baxter_move_.reset(new baxter_pick_place::BaxterMoveGroupInterface("right_arm"));

        // Send
        baxter_move_->sendToPose("right_neutral");
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("send_ready","Unknown planning group");
        return;
      }
    }
    
    // std::cout << "Press y then enter to send to ready pose " << std::endl;

    // std::string input;
    // std::cin >> input;


    ROS_INFO_STREAM_NAMED("temp","DONE");
  }

};

} //namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "send_ready");

  ROS_INFO_STREAM("Sending arm(s) to ready position");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  std::string planning_group = "both_arms";
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "-h") == 0)
      {
        std::cout << "Usage: rosrun baxter_pick_place send_ready --planning_group [both_arms,left_arm,right_arm] --verbose" << std::endl;
        return 0;
      }

      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode");
        verbose = true;
      }

      if (strcmp(argv[i], "--planning_group") == 0)
      {
        ++i;
        planning_group = argv[i];
        ROS_INFO_STREAM_NAMED("main","Using planning_group " << planning_group);
      }
    }
  }

  baxter_experimental::SendReady server(verbose, planning_group);

  ros::Duration(5).sleep();
  ros::spinOnce();

  ROS_INFO_STREAM("Shutting down.");

  ros::shutdown();

  return 0;
}
