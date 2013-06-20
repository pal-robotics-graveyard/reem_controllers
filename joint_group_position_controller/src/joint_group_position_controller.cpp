/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2011, PAL Robotics, S.L.
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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
 */


// C++ standard headers
#include <algorithm>

// ROS headers
#include "joint_group_position_controller/joint_group_position_controller.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(joint_group_position_controller, JointGroupPositionController, controller::JointGroupPositionController, pr2_controller_interface::Controller)

namespace controller
{

using std::size_t;

JointGroupPositionController::JointGroupPositionController()
  : robot_(0)
{}

JointGroupPositionController::~JointGroupPositionController()
{
  sub_command_.shutdown();
}

bool JointGroupPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  using namespace XmlRpc;
  node_  = n;
  robot_ = robot;

  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                node_.getNamespace().c_str());
      return false;
    }

    pr2_mechanism_model::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",
                ((std::string)name_value).c_str(), node_.getNamespace().c_str());
      return false;
    }
    joints_.push_back(j);
  }

  // Ensures that all the joints are calibrated
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
    {
      ROS_ERROR("Joint %s was not calibrated (namespace: %s)",
                joints_[i]->joint_->name.c_str(), node_.getNamespace().c_str());
      return false;
    }
  }

  // Sets up pid controllers for all of the joints
  std::string gains_ns;
  if (!node_.getParam("gains", gains_ns)) {gains_ns = node_.getNamespace() + "/gains";}
  pids_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i]->joint_->name))) {return false;}
  }

  // Preallocate resources
  errors_  = JointErrorList(joints_.size());
  ref_pos_ = JointPosList(joints_.size());
  lookup_  = JointStateToCmd(joints_.size());

  // Create input topic subscriptor
  sub_command_ = node_.subscribe("command", 1, &JointGroupPositionController::commandCB, this);

  return true;
}

void JointGroupPositionController::starting()
{
  // Reset PIDs
  for (size_t i = 0; i < pids_.size(); ++i) {pids_[i].reset();}

  // Cache time and joint positions
  last_time_ = robot_->getTime();
  for (size_t i = 0; i < joints_.size(); ++i) {ref_pos_[i] = joints_[i]->position_;}
}

void JointGroupPositionController::update()
{
  // Compute delta time and update time cache
  ros::Time     time = robot_->getTime();
  ros::Duration dt   = time - last_time_;
  last_time_         = time;

  for(size_t i = 0; i < joints_.size(); ++i)
  {
    // Tracking error
    if(joints_[i]->joint_->type == urdf::Joint::REVOLUTE)
    {
      angles::shortest_angular_distance_with_limits(ref_pos_[i], joints_[i]->position_, joints_[i]->joint_->limits->lower, joints_[i]->joint_->limits->upper, errors_[i]);
    }
    else if(joints_[i]->joint_->type == urdf::Joint::CONTINUOUS)
    {
      errors_[i] = angles::shortest_angular_distance(ref_pos_[i], joints_[i]->position_);
    }
    else // Prismatic
    {
      errors_[i] = ref_pos_[i] - joints_[i]->position_;
    }

    // Commanded effort
    joints_[i]->commanded_effort_ = pids_[i].computeCommand(errors_[i], dt);
  }
}

void JointGroupPositionController::commandCB(const sensor_msgs::JointState::ConstPtr &command)
{
  if (command->name.size() != command->position.size())
  {
    ROS_ERROR("Size mismatch in members of input command (name, position).");
    return;
  }

  // Map the controlled joints to the joints in the message
  lookup_ = JointStateToCmd(lookup_.size(), -1);
  for(size_t i = 0; i < joints_.size(); ++i)
  {
    for (size_t j = 0; j < command->name.size(); ++j)
    {
      if (command->name[j] == joints_[i]->joint_->name)
      {
        lookup_[i] = j;
        break;
      }
    }

    if (lookup_[i] == -1)
    {
      ROS_ERROR_STREAM("Unable to locate joint " << joints_[i]->joint_->name << " in the commanded trajectory.");
      return;
    }
  }

  // Populate reference position vector
  for(size_t i = 0; i < joints_.size(); ++i)
  {
    ref_pos_[i] = command->position[lookup_[i]];
  }
}

} // controller
