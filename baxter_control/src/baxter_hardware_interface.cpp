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

// Boost
#include <boost/bind.hpp>

#include <baxter_control/baxter_hardware_interface.h>

namespace baxter_control
{

// Contructor
BaxterHardwareInterface::BaxterHardwareInterface()
  : has_joint_interface_to_joint_state_(false),
    state_msg_(new sensor_msgs::JointState())
{
}

bool BaxterHardwareInterface::init()
{
  robot_namespace_ = "robot";
  robot_description_ = "robot_description"; // default

  // Get parameters/settings for controllers from ROS param server
  model_nh_ = ros::NodeHandle(robot_namespace_);
  nh_ = ros::NodeHandle("/");
  ROS_INFO_NAMED("hardware_interface", "Starting baxter_control in namespace: %s", robot_namespace_.c_str());

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  if (!parseTransmissionsFromURDF())
  {
    ROS_ERROR_STREAM_NAMED("hardware_interface", "Error parsing URDF.\n");
    return false;
  }

  // Resize vectors to our DOF
  n_dof_ = transmissions_.size();
  joint_names_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  joint_interface_to_joint_state_.resize(n_dof_);

  // Initialize values
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions_[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("hardware_interface","Transmission " << transmissions_[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions_[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("hardware_interface","Transmission " << transmissions_[j].name_
        << " has more than one joint. Currently this "
        << " interface only supports one.");
      continue;
    }

    // Check that this transmission has one actuator
    if(transmissions_[j].actuators_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("hardware_interface","Transmission " << transmissions_[j].name_
        << " has no associated actuators.");
      continue;
    }
    else if(transmissions_[j].actuators_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("hardware_interface","Transmission " << transmissions_[j].name_
        << " has more than one actuator. Currently the default robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    // Add data from transmission
    joint_names_[j] = transmissions_[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_position_command_[j] = 0.0;
    joint_effort_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    const std::string& hardware_interface = "PositionJointInterface"; //transmissions_[j].actuators_[0].hardware_interface_;

    // Debug
    ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading joint '" << joint_names_[j]
      << "' of type '" << hardware_interface << "'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    // Decide what kind of command interface this actuator/joint has
    if(hardware_interface == "EffortJointInterface")
    {
      // Create effort joint interface
      ej_interface_.registerHandle(hardware_interface::JointHandle(
          js_interface_.getHandle(joint_names_[j]),&joint_effort_command_[j]));
    }
    else if(hardware_interface == "VelocityJointInterface")
    {
      // Create velocity joint interface
      vj_interface_.registerHandle(hardware_interface::JointHandle(
          js_interface_.getHandle(joint_names_[j]),&joint_velocity_command_[j]));
    }
    else if(hardware_interface == "PositionJointInterface")
    {
      // Create position joint interface
      pj_interface_.registerHandle(hardware_interface::JointHandle(
          js_interface_.getHandle(joint_names_[j]),&joint_position_command_[j]));
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("hardware_interface","No matching hardware interface found for '"
        << hardware_interface );
      return false;
    }
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&pj_interface_);

  // Start publishers and subscribers
  pub_position_command_ = nh_.advertise<baxter_msgs::JointPositions>("/robot/limb/right/command_joint_angles",10);

  sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/robot/joint_states/",
                     1, &BaxterHardwareInterface::stateCallback, this);

  ros::spinOnce();
  ros::Duration(5.0).sleep();

  ROS_INFO_NAMED("hardware_interface", "Loaded baxter_hardware_interface.");
  return true;
}

BaxterHardwareInterface::~BaxterHardwareInterface()
{
}

void BaxterHardwareInterface::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  // Copy the latest message into a buffer
  state_msg_ = msg;
  state_msg_timestamp_ = ros::Time::now();

  //ROS_DEBUG_STREAM_NAMED("temp","recieved message \n" << *msg);

  if( has_joint_interface_to_joint_state_ == false)
  {
    has_joint_interface_to_joint_state_ = true;
    loadJointStateNameMap();
  }
}

void BaxterHardwareInterface::loadJointStateNameMap()
{
  int controlled_joints_count = 0;
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    // Find the name in the state message that corresponds to the transmission index
    std::vector<std::string>::const_iterator name_it;
    name_it = std::find(state_msg_->name.begin(), state_msg_->name.end(), joint_names_[i]);

    // Error check
    if( name_it == state_msg_->name.end() )
    {
      ROS_DEBUG_STREAM_NAMED("hardware_interface","No mapping for '" <<
        joint_names_[i] << "' in Baxter state message");
    }
    else
    {
      // We will use this in our published command message - keep track of size of vector
      controlled_joints_count++;
    }

    // Copy index of found joint name
    joint_interface_to_joint_state_[i] = name_it - state_msg_->name.begin() - 1;
  }

  //std::copy(joint_interface_to_joint_state_.begin(), joint_interface_to_joint_state_.end(), 
  //  std::ostream_iterator<double>(std::cout, "\n"));      

  // Resize the output publisher message to match input message
  output_msg_.angles.resize(controlled_joints_count);
  output_msg_.names.resize(controlled_joints_count);

  ROS_DEBUG_STREAM_NAMED("ros_control","Matched " << controlled_joints_count << 
    " joints from state message and URDF transmissions");

  // \temp
  sleep(2);
}

bool BaxterHardwareInterface::stateExpired()
{
  // Check that we have a non-expired state message
  // \todo lower the expiration duration
  if( ros::Time::now() > state_msg_timestamp_ + ros::Duration(10.0)) // check that the message timestamp is no older than 1 second
  {
    ROS_WARN_STREAM_NAMED("ros_control","State expired. \n" << ros::Time::now() << "\n" <<
      state_msg_timestamp_ + ros::Duration(10.0) << "\n" << " State: \n" << *state_msg_ );
    return true;
  }
  return false;
}

void BaxterHardwareInterface::read()
{  
  if( stateExpired() )
    return;

  // Copy state message to our datastructures
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    //ROS_INFO_STREAM_NAMED("read","on index " << i << " which maps to " << joint_interface_to_joint_state_[i]);
    //ROS_INFO_STREAM_NAMED("read","and the max message is " << state_msg_->position.size() );
    //ROS_INFO_STREAM_NAMED("read","joint_position size is " << joint_position_.size());
    //ROS_INFO_STREAM_NAMED("read","state_msg position size is " << state_msg_->position.size());
    //ROS_INFO_STREAM_NAMED("read","position is " << state_msg_->position[joint_interface_to_joint_state_[i]]);

    // Check if this joint has no mapping
    if( joint_interface_to_joint_state_[i] >= state_msg_->name.size() - 1)
    {
      ROS_WARN_STREAM_ONCE_NAMED("read","Skipping joint " << joint_names_[i]);
      continue;
    }

    joint_position_[i] = state_msg_->position[joint_interface_to_joint_state_[i]];
    joint_velocity_[i] = state_msg_->velocity[joint_interface_to_joint_state_[i]];
    joint_effort_[i] = state_msg_->effort[joint_interface_to_joint_state_[i]];
  }
}

void BaxterHardwareInterface::write()
{
  if( stateExpired() )
    return;

  //ROS_ERROR_STREAM_NAMED("temp","WRITING ----------------------------------");

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    //ROS_INFO_STREAM_NAMED("write","id = "<<i);
    //ROS_INFO_STREAM_NAMED("write","name = " << joint_names_[i]);
    //ROS_INFO_STREAM_NAMED("write","mapping = " << joint_interface_to_joint_state_[i]);

    // Check if this joint has no mapping
    if( joint_interface_to_joint_state_[i] >= state_msg_->name.size() - 1 )
    {
      ROS_WARN_STREAM_ONCE_NAMED("write","Skipping joint " << joint_names_[i]);
      continue;
    }

    output_msg_.angles[joint_interface_to_joint_state_[i]] = joint_position_command_[i];
    // Add to output message
    output_msg_.names[joint_interface_to_joint_state_[i]] = joint_names_[i];
  }

  pub_position_command_.publish(output_msg_);
}

// Get the URDF XML from the parameter server
std::string BaxterHardwareInterface::getURDF(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_STREAM_NAMED("hardware_interface", "waiting for URDF on parameter server " << param_name 
        << " in namespace " << nh_.getNamespace());

      nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_STREAM_NAMED("hardware_interface", "waiting for URDF on parameter server " << param_name 
        << " in namespace " << nh_.getNamespace());

      nh_.getParam(param_name, urdf_string);
    }
    ros::Duration(1.0).sleep();
  }
  ROS_DEBUG_STREAM_NAMED("hardware_interface", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool BaxterHardwareInterface::parseTransmissionsFromURDF()
{
  std::string urdf_string = getURDF(robot_description_);

  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);

  if( transmissions_.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED("hardware_interface","No transmissions found in URDF. Unable to setup any hardware interfaces.");
    return false;
  }

  return true;
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
  if( !baxter.init() )
    return 0;

  // Create the controller manager
  controller_manager::ControllerManager controller_manager(&baxter, nh);
  ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");

  // Update controllers
  ros::Rate rate(100); // 50 hz
  while (ros::ok())
  {
    baxter.read();
    controller_manager.update(ros::Time::now(), rate.cycleTime() ); // get the actual run time of a cycle from start to sleep
    baxter.write();
    rate.sleep();    
  }

  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

  return 0;
}


