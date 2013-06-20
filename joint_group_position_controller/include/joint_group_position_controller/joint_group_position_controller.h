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


#ifndef JOINT_GROUP_POSITION_CONTROLLER_H
#define JOINT_GROUP_POSITION_CONTROLLER_H

// C++ standard headers
#include <cstddef>
#include <string>
#include <vector>

// ROS headers
#include <ros/node_handle.h>
#include <control_toolbox/pid.h>
#include <pr2_controller_interface/controller.h>
#include <sensor_msgs/JointState.h>

namespace controller
{

/// \brief Controls the position of a group of joints using a pid loop.
/// \author Adolfo Rodriguez Tsouroukdissian.
/// This class is heavily inspired on the structure of the
/// <a href="http://www.ros.org/wiki/robot_mechanism_controllers/JointSplineTrajectoryController">JointSplineTrajectoryController</a>
/// written by Stuart Glaser, and the
/// <a href="http://www.ros.org/wiki/robot_mechanism_controllers/JointPositionController">JointPositionController</a>.

class JointGroupPositionController : public pr2_controller_interface::Controller
{
public:
  typedef std::vector<control_toolbox::Pid> PidList;
  typedef std::vector<double>               JointErrorList;
  typedef std::vector<double>               JointPosList;
  typedef std::vector<int>                  JointStateToCmd;

  JointGroupPositionController();
  ~JointGroupPositionController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void starting();
  void update(const ros::Time&, const ros::Duration&) {update();} // ros_control compatibility fix
  void update();

private:
  pr2_mechanism_model::RobotState *robot_;
  ros::Time last_time_;
  std::vector<pr2_mechanism_model::JointState*> joints_;
  PidList pids_;
  JointPosList    ref_pos_;
  JointErrorList  errors_;
  JointStateToCmd lookup_;

  ros::NodeHandle node_;

  void commandCB(const sensor_msgs::JointState::ConstPtr &command);
  ros::Subscriber sub_command_;
};


} // controller

#endif // JOINT_GROUP_POSITION_CONTROLLER_H