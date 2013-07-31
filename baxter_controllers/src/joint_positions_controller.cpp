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

/**
 *  \author Dave Coleman
 *  \desc   Multiple joint position controller for Baxter SDK
 */

#include "joint_positions_controller.h"
#include <pluginlib/class_list_macros.h>

namespace baxter_controllers {

JointPositionsController::JointPositionsController()
  : new_command_(true),
    update_counter_(0)
{}

JointPositionsController::~JointPositionsController()
{
  position_command_sub_.shutdown();
}

bool JointPositionsController::init(
  hardware_interface::EffortJointInterface *robot, ros::NodeHandle &nh)
{
  // Store nodehandle
  nh_ = nh;

  // Get joint names
  XmlRpc::XmlRpcValue xml_array;
  if( !nh_.getParam("joint_names", xml_array) )
  {
    ROS_ERROR("No 'joint_names' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }
  // Make sure it's an array type
  if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The 'joint_names' parameter is not an array (namespace '%s')",nh_.getNamespace().c_str());
    return false;
  }

  // Get number of joints
  n_joints_ = xml_array.size();
  ROS_INFO_STREAM("Initializing JointPositionsController with "<<n_joints_<<" joints.");

  joint_names_.resize(n_joints_);
  position_controllers_.resize(n_joints_);

  for(int i=0; i<n_joints_; i++)
  {
    // Get joint name
    if(xml_array[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("The 'joint_names' parameter contains a non-string element (namespace '%s')",nh_.getNamespace().c_str());
      return false;
    }
    joint_names_[i] = static_cast<std::string>(xml_array[i]);

    // Get the joint-namespace nodehandle
    {
      ros::NodeHandle joint_nh(nh_, "joints/"+joint_names_[i]);
      ROS_INFO("Loading joint info for '%s', Namespace: %s", joint_names_[i].c_str(), joint_nh.getNamespace().c_str());

      position_controllers_[i].reset(new effort_controllers::JointPositionController());
      position_controllers_[i]->init(robot, joint_nh);

      // DEBUG
      //position_controllers_[i]->printDebug();

    } // end of joint-namespaces

    // Add joint name to map (allows unordered list to quickly be mapped to the ordered index)
    joint_to_index_map_.insert(std::pair<std::string,std::size_t>(joint_names_[i],i));
  }

  // Create command subscriber custom to baxter
  position_command_sub_ = nh_.subscribe<baxter_msgs::JointPositions>(
    "command", 1, &JointPositionsController::commandCB, this);

  return true;
}



void JointPositionsController::starting(const ros::Time& time)
{
  baxter_msgs::JointPositions initial_command;

  // Fill in the initial command
  for(int i=0; i<n_joints_; i++)
  {
    initial_command.names.push_back( position_controllers_[i]->getJointName());
    initial_command.angles.push_back(position_controllers_[i]->getPosition());
  }
  position_command_buffer_.initRT(initial_command);
  new_command_ = true;
}

void JointPositionsController::stopping(const ros::Time& time)
{

}

void JointPositionsController::update(const ros::Time& time, const ros::Duration& period)
{
  // Debug info
  verbose_ = false;
  update_counter_ ++;
  if( update_counter_ % 100 == 0 )
    verbose_ = true;

  updateCommands();

  // Apply joint commands
  for(size_t i=0; i<n_joints_; i++)
  {
    // Update the individual joint controllers
    position_controllers_[i]->update(time, period);
  }
}

void JointPositionsController::updateCommands()
{
  // Check if we have a new command to publish
  if( !new_command_ )
    return;

  // Go ahead and assume we have proccessed the current message
  new_command_ = false;

  // Get latest command
  const baxter_msgs::JointPositions &command = *(position_command_buffer_.readFromRT());

  // Error check message data
  if( command.angles.count() != command.names.count() )
  {
    ROS_ERROR_STREAM_NAMED("update","List of names does not match list of angles size, "
      command.angles.count() << " != " << command.names.count() );
    return;
  }

  std::map<std::string,std::size_t>::iterator name_it;

  // Map incoming joint names and angles to the correct internal ordering
  for(size_t i=0; i<command.names.size(); i++)
  {
    // Check if the joint name is in our map
    name_it = joint_to_index_map.find(command.names[i]);

    if( name_it != joint_to_index_map.end() )
    {
      // Joint is in the map, so we'll update the joint position
      position_controllers_[i]->setCommand( command.angles[i] );
    }
  }
}

void JointPositionsController::commandCB(const baxter_msgs::JointPositionsConstPtr& msg)
{
  ROS_DEBUG("Received new command");

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  position_command_buffer_.writeFromNonRT(*msg);

  new_reference_traj_ = true;
}


} // namespace

PLUGINLIB_EXPORT_CLASS(
  baxter_controllers::JointPositionsController,
  controller_interface::ControllerBase)
