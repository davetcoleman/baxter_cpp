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
   Desc:   Sends commands to individual baxter joints and records results to csv file
*/

#include <baxter_experimental/baxter_to_csv.h>

namespace baxter_experimental
{

BaxterToCSV::BaxterToCSV(double record_time, const std::string &file_name)
  : state_msg_(new sensor_msgs::JointState()),
    arm_name_("left"),
    joint_name_("w1"),
    current_state_count_(0),
    file_name_(file_name),
    finished_(false)
{


  // Setup output message
  output_command_msg_.angles.resize(1);
  output_command_msg_.names.resize(1);
  output_command_msg_.angles[0] = fRand(-1,1);
  output_command_msg_.names[0] = joint_name_;

  // Decide how much data to record. Preallocate to save time in realtime loop
  joint_states_.resize(record_time*100);
  joint_commands_.resize(record_time*100);

  // Start publishers
  pub_position_command_ = nh_.advertise<baxter_msgs::JointPositions>("/robot/limb/"+arm_name_+
                          "/command_joint_angles",10);

  // Start subscribers
  sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/robot/limb/" + arm_name_ +
                     "/joint_states", 1, &BaxterToCSV::stateCallback, this);

  // Load URDF to get joint limits
  urdf_.initFile("/home/dave/ros/ws_baxter/src/baxter_common/baxter_description/urdf/baxter.urdf");
  joint_ = urdf_.getJoint(arm_name_ + "_" + joint_name_);

  // Wait for first state message to be recieved
  while(ros::ok() && state_msg_timestamp_.toSec() == 0)
  {
    ROS_INFO_STREAM_NAMED(arm_name_,"Waiting for first state message to be recieved");
    ros::spinOnce();
    ros::Duration(0.25).sleep();
  }

  // Enable baxter
  bool enabled = false;
  while(!enabled && ros::ok())
  {
    if( !baxter_util_.enableBaxter() )
    {
      ROS_WARN_STREAM_NAMED("baxter_to_csv","Unable to enable Baxter, retrying...");
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
    else
    {
      enabled = true;
    }
  }

  // Start the experiment -----------------------------------------

  ros::Duration update_freq = ros::Duration(1.0/RECORD_RATE_HZ);
  non_realtime_loop_ = nh_.createTimer(update_freq, &BaxterToCSV::update, this);

  ROS_INFO_NAMED("baxter_to_csv", "Started to run experiment...");

  // Wait for finished flag
  while(ros::ok() && !finished_)
  {
    ros::Duration(0.5).sleep();
  }
}

BaxterToCSV::~BaxterToCSV()
{
  baxter_util_.disableBaxter();
}

void BaxterToCSV::update(const ros::TimerEvent& e)
{
  static bool first = true;
  if (first)
    first = false;
  else
    ROS_INFO_STREAM_THROTTLE_NAMED(2, "update","Updating with period: " 
      << ((e.current_real - e.last_real)*100) << " hz" );

  // \todo should we send the command first then record it and the state, or record the state and the
  // previous command before sending the next command?

  // Record to our matrix the latest state
  if (current_state_count_ < joint_states_.size())
  {
    joint_states_[current_state_count_] = *state_msg_;
    joint_commands_[current_state_count_] = output_command_msg_.angles[0]; // record current command
    current_state_count_++;
  }
  else
  {
    ROS_INFO_STREAM_NAMED("update","Finished recording");
    non_realtime_loop_.stop();
    writeToFile(file_name_);
    finished_ = true;
    return;
  }

  // Output new command
  if( current_state_count_ % int(RECORD_RATE_HZ / COMMAND_CHANGE_HZ) == 0)
  {
    // Choose rand position between min and max
    output_command_msg_.angles[0] = fRand(joint_->limits->lower + SAFETY_LIMIT,
                                    joint_->limits->upper - SAFETY_LIMIT);
  }
  
  // Output command
  pub_position_command_.publish(output_command_msg_);
}

void BaxterToCSV::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  // Copy the latest message into a buffer
  state_msg_ = msg;
  state_msg_timestamp_ = ros::Time::now();

}

bool BaxterToCSV::stateExpired()
{
  // Check that we have a non-expired state message
  // \todo lower the expiration duration
  if( ros::Time::now() > state_msg_timestamp_ + ros::Duration(STATE_EXPIRED_TIMEOUT)) // check that the message timestamp is no older than 1 second
  {

    ROS_WARN_STREAM_THROTTLE_NAMED(1,arm_name_,"State expired. Last recieved state " << (ros::Time::now() - state_msg_timestamp_).toSec() << " seconds ago." );
    return true;
  }
  return false;
}

bool BaxterToCSV::writeToFile(const std::string& file_name)
{
  if (!joint_states_.size())
  {
    ROS_ERROR_STREAM_NAMED("baxter_to_csv","No joint states populated");
    return false;
  }

  std::ofstream output_file;
  output_file.open (file_name.c_str());

  // Output header
  output_file << "timestamp,";
  for (std::size_t j = 0; j < joint_states_[0].position.size(); ++j)
  {
    output_file << joint_states_[0].name[j] << "_pos,"
                << joint_states_[0].name[j] << "_vel,"
                << joint_states_[0].name[j] << "_eff,";
  }
  output_file << joint_name_ << "_command\n";

  // Subtract starting time
  double start_time = joint_states_[0].header.stamp.toSec();

  for (std::size_t i = 0; i < joint_states_.size(); ++i)
  {
    // Timestamp
    output_file << joint_states_[i].header.stamp.toSec() - start_time << ",";

    // Output entire state of robot to single line
    for (std::size_t j = 0; j < joint_states_[i].position.size(); ++j)
    {
      output_file << joint_states_[i].position[j] << ","
                  << joint_states_[i].velocity[j] << ","
                  << joint_states_[i].effort[j] << ",";
    }
    // Commanded position
    output_file << joint_commands_[i] << std::endl;
  }
  output_file.close();
  ROS_INFO_STREAM_NAMED("baxter_to_csv","Wrote to file " << file_name);
  return true;
}


} // namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("baxter_to_csv","Starting baxter to csv script...");

  ros::init(argc, argv, "baxter_to_csv");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  std::string file_name = "/home/dave/ros/baxter_recordings/characterization/output.csv";
  // Parse command line arguments
  for (std::size_t i = 0; i < argc; ++i)
  {
    if( std::string(argv[i]).compare("--output") == 0 )
    {
      ++i;
      file_name = argv[i];
      ROS_INFO_STREAM_NAMED("main","Outputting data to " << file_name);
    }
  }

  // Seed random
  srand(ros::Time::now().toSec());

  // Load experiment
  baxter_experimental::BaxterToCSV baxter(baxter_experimental::RECORD_TIME, file_name);

  ROS_INFO_STREAM_NAMED("baxter_to_csv","Shutting down.");

  return 0;
}



