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

// Enable the collision check
// Note: this is not real time safe due FCL
//# define COLLISION_CHECK

# ifdef COLLISION_CHECK
# include <ros_control_pipeline/safety.hpp>
# endif

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

    void runPython(std::ostream& file, const std::string& command, dynamicgraph::Interpreter& interpreter);
    void startupPythonEnv(ros::NodeHandle& controller_nh);
    stdVector_t loadFreeFlyer(ros::NodeHandle& controller_nh) const;

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

    SotDevice* getDevicePtr();

private:

    static const std::string LOG_PYTHON;
    joints_t joints_;
    jointNames_t jointNames_;
    stdVector_t ffpose_;
    stdVector_t init_conf_;

    /// \brief Real time publisher.
    boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > publisher_;

    /// \brief Embedded python interpreter accessible via a ROS service.
    boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

    /// \brief Pointer to Entity StackOfTasks.
    SotDevice* device_;
    stdVector_t position_;
    stdVector_t velocity_;

# ifdef COLLISION_CHECK
    /// \brief Safety checker.
    boost::shared_ptr<pipeline::BipedSafety>  bs_;
# endif

/*
    /// \brief Internal vectors, used to speed up the computations
    stdVector_t internalStdVector_;
    ml::Vector internalMaalVector_;

    /// \brief Convert ml::Vector into std::vector or viceversa, use internal vectors
    /// to be real time safe
    void convertVector(const ml::Vector& v)
    {
        for (unsigned i = 0; i < v.size(); ++i)
            internalStdVector_[i] = v(i);
    }

    void convertVector(const stdVector_t& v)
    {
        for (unsigned i = 0; i < v.size(); ++i)
            internalMaalVector_(i) = v[i];
    }
*/

};
}

# endif
