/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

/** \author Gennaro Raiola
 *   inspired on the sot_pr2 class written by Thomas Moulard, available here https://github.com/laas/sot_pr2.
 */

# ifndef SOT_CONTROLLER_H
# define SOT_CONTROLLER_H

# include <forward_command_controller/forward_command_controller.h>
# include <dynamic_graph_bridge/ros_interpreter.hh>
# include "sot_controller/sot_device.h"
# include <boost/shared_ptr.hpp>
# include <realtime_tools/realtime_publisher.h>
# include <sensor_msgs/JointState.h>

/**
 * \brief Position controller for the robot. It is wrapping sot_device.
 *
 * This class works as plugin for ros_control.
 *
 */

namespace sot_controller
{

class SotController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{

public:
    SotController();
    ~SotController();

    /// \brief Run python commands in the embedded python interpreter.
    void runPython(std::ostream& file, const std::string& command, dynamicgraph::Interpreter& interpreter);
    /// \brief Configure the python enviroment and create entities for the dynamic graph.
    void startupPythonEnv(ros::NodeHandle& controller_nh);
    /// \brief Load the free flyer pose from the parameter server.
    stdVector_t loadFreeFlyer(ros::NodeHandle& controller_nh) const;
    /// \name Inherited control methods.
    /// \{
    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    /// \}
    /// \brief Get the device pointer.
    SotDevice* getDevicePtr();

private:
    /// \brief Define the log file name for the python startup.
    static const std::string LOG_PYTHON;
    /// \brief Vector of joint handles.
    joints_t joints_;
    /// \brief Vector of joint names.
    jointNames_t jointNames_;
    /// \brief Free flyer pose.
    stdVector_t ffpose_;
    /// \brief Initial position of the robot, the size of this vector is equal to the number of joints plus six for the ff pose.
    stdVector_t init_position_;
    /// \brief Initial velocity of the robot, the size of this vector is equal to the number of joints plus six for the ff velocity.
    stdVector_t init_velocity_;
    /// \brief Real time publisher.
    boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > publisher_;
    /// \brief Embedded python interpreter accessible via a ROS service.
    boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;
    /// \brief Pointer to Entity StackOfTasks.
    SotDevice* device_;
    /// \brief Position vector of the joints, the size of this vector is equal to the number of joints.
    stdVector_t position_;
    /// \brief Velocity vector of the joints, the size of this vector is equal to the number of joints.
    stdVector_t velocity_;
};
}

# endif
