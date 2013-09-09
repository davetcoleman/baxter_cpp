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

// Gripper
#include <baxter_gripper_server/electric_parallel_gripper.h>


namespace baxter_gripper_server
{

class GripperActionServer
{
protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher joint_state_topic_;

  // Rate to publish joint states
  ros::Timer timer_;

  bool in_simulation_; // Using Gazebo or not

  // Gripper objects
  baxter_gripper_server::ElectricParallelGripper right_gripper;
  baxter_gripper_server::ElectricParallelGripper left_gripper;

public:

  // Constructor
  GripperActionServer(bool in_simulation, bool run_test)
    : in_simulation_(in_simulation),
      right_gripper("baxter_right_gripper_action/gripper_action","right", in_simulation),
      left_gripper("baxter_left_gripper_action/gripper_action","left", in_simulation)
  {
    // Get from either gripper if we are in simulation
    in_simulation_ = right_gripper.isInSimulation();

    // Wait for both calibrations to finish
    ros::Duration(2.0).sleep();

    // Run optional test
    if(run_test)
      runTest();

    // Gazebo publishes a joint state for the gripper, but Baxter does not do so in the right format
    if( in_simulation_ )
      return;

    // Publish joint_states
    joint_state_topic_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_states",10);

    // Set publish frequency
    ros::NodeHandle nh_tilde("~");
    double publish_freq = 50; // hz
    nh_tilde.param("publish_frequency", publish_freq, 50.0);

    ros::Rate rate(publish_freq); // hz
    while (ros::ok())
    {
      update();
      rate.sleep();
    }
  }

  void runTest()
  {
    // Error check gripper
    left_gripper.hasError();
    right_gripper.hasError();

    bool open = false;
    while(ros::ok())
    {
      if(open)
      {
        left_gripper.openGripper();
        right_gripper.openGripper();
        open = false;
      }
      else
      {
        left_gripper.closeGripper();
        right_gripper.closeGripper();
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

    // Create state message
    sensor_msgs::JointState state;
    right_gripper.populateState(state);
    left_gripper.populateState(state);

    joint_state_topic_.publish(state);
  }

}; // end of class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_gripper_server");

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

  baxter_gripper_server::GripperActionServer(in_simulation, run_test);

  ros::spin();

  return 0;
}

