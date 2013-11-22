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
   Desc:   Baxter's head turns based on sonar sensors
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

// Baxter
#include <baxter_core_msgs/HeadPanCommand.h>

// C++
#include <numeric>      // std::accumulate

namespace baxter_scripts
{

class BaxterHeadFollow
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher pub_head_turn_;

  // Subscriber
  ros::Subscriber sub_sonars_;

  // Constants
  const int SONAR_COUNT_; // Total number of sonars on baxter
  const double FREQ_INCREMENT_SCALE_;  // Amount to increment the occupancy frequency
  const double FREQ_DECREMENT_;  // AMount to decreate the occupancy frenquency
  const double SONAR_TRIGGER_THRESHOLD_; // ignore sonars with values less than this

  // Track frequency of occupancy
  std::vector<double> occupancy_freq_;
  //std::vector<double> occupancy_areas_;

  // Update loop
  ros::Timer non_realtime_loop_;

  baxter_core_msgs::HeadPanCommand head_command_;
  
  bool verbose_; // show debug info

public:

  /**
   * \brief Constructor
   */
  BaxterHeadFollow()
    : SONAR_COUNT_(12),
      FREQ_INCREMENT_SCALE_(0.01),
      FREQ_DECREMENT_(-0.7),
      SONAR_TRIGGER_THRESHOLD_(0.4),
      verbose_(false)
  {
    // Start publishers
    pub_head_turn_ = nh_.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",10);
    head_command_.speed = 10;

    // Start subscribers
    sub_sonars_ = nh_.subscribe<sensor_msgs::PointCloud>("/robot/sonar/head_sonar/state", 1, &BaxterHeadFollow::sonarCallback, this);

    // Create probability structure and initialize
    occupancy_freq_.resize(SONAR_COUNT_);
    //occupancy_areas_.resize(SONAR_COUNT_);
    for (std::size_t i = 0; i < occupancy_freq_.size(); ++i)
    {
      occupancy_freq_[i] = 0;
    }

    // Create head control loop
    double hz = 1; // times per second
    ros::Duration update_freq = ros::Duration(1.0/hz);
    non_realtime_loop_ = nh_.createTimer(update_freq, &BaxterHeadFollow::update, this);
  }

  /**
   * \brief Destructor
   */
  ~BaxterHeadFollow()
  {

  }

  /**
   * \brief Callback from subscriber
   * \param msg - the recieved ROS message
   */
  void sonarCallback(const sensor_msgs::PointCloudConstPtr& msg)
  {
    //ROS_INFO_STREAM_NAMED("temp","recieved " << msg->channels[0].values.size());

    // Increment the channels that are detected this round
    for (std::size_t i = 0; i < msg->channels[0].values.size(); ++i)
    {
      int sonar_id = msg->channels[0].values[i];

      // Ignore the back sonars
      if (sonar_id == 5 || sonar_id == 6 || sonar_id == 7)
        continue;

      // Increment the probability based on distance
      occupancy_freq_[sonar_id] =
        std::min(1.0, occupancy_freq_[sonar_id] +
          1.0 /
          msg->channels[1].values[i]
          * FREQ_INCREMENT_SCALE_);

      /*ROS_INFO_STREAM_NAMED("temp","increased " << sonar_id << " with " <<
        msg->channels[1].values[i]
        <<" by " <<
        1.0 /
        msg->channels[1].values[i]
        * FREQ_INCREMENT_SCALE_);
        */
    }
  }

  /**
   * \brief Called at increments to move head
   */
  void update(const ros::TimerEvent& e)
  {
    // Debug array
    if (verbose_)
    {
      for (std::size_t i = 0; i < occupancy_freq_.size(); ++i)
      {
        std::cout << std::fixed << std::setprecision(1) << occupancy_freq_[i] << " | ";
      }
    }

    // Choose where to move head
    // add sonars into groups of two
    double largest_value = 0;
    int largest_index = -1;
    for (std::size_t i = 0; i < occupancy_freq_.size(); ++i)
    {
      // Check for max grouping
      if (occupancy_freq_[i] > largest_value)
      {
        largest_value = occupancy_freq_[i];
        largest_index = i;
      }
    }

    // Don't move if no sonars are very strongly triggered
    if (largest_value < SONAR_TRIGGER_THRESHOLD_)
    {
      largest_index = -1;
    }

    // Use the largest index to move head
    double command;
    if (largest_index == -1 ) // they area all blanks
    {
      command = 0; // straight ahead
    }
    else
    {
      if ( largest_index == 4 )
      {
        // Move all the way to the left
        command = -1.0;
      }
      else if ( largest_index < 4 )
      {
        // Move to left
        // Map 1-3 to 0 to -1
        command = std::max(-1.0, -1 * double(largest_index) / 3);
      }
      else if ( largest_index == 8 )
      {
        // Move all the way to the right
        command = 1.0;
      }
      else // (greater than 8)
      {
        // Move to right
        // Map 11-9 to 0 to 1
        command = std::min(1.0, 1.33 - double(largest_index - 8) / 3);
      }
      head_command_.target = command;
      pub_head_turn_.publish(head_command_);
    }
    if (verbose_)
      std::cout << "== " << command << " from " << largest_value << " at " << largest_index << std::endl;

    // Decrement all channels to allow baxter to "forget"
    for (std::size_t i = 0; i < occupancy_freq_.size(); ++i)
    {
      occupancy_freq_[i] = std::max(0.0, occupancy_freq_[i] + FREQ_DECREMENT_);
    }
  }


}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("baxter_head_follow","Baxter Head Follow");

  ros::init(argc, argv, "baxter_head_follow");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  baxter_scripts::BaxterHeadFollow tester;

  ros::spin();

  ROS_INFO_STREAM_NAMED("baxter_head_follow","Shutting down.");

  return 0;
}

