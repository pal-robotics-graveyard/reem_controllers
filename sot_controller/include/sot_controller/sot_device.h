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

# ifndef SOT_DEVICE_H
# define SOT_DEVICE_H

# include <sot/core/device.hh>
# include <forward_command_controller/forward_command_controller.h>

# include <boost/thread.hpp>

# define CPP11
# ifdef CPP11
# include <atomic>
# endif

// Enable the collision check in the device
# define COLLISION_CHECK_DEVICE

# ifdef COLLISION_CHECK_DEVICE
# include <ros_control_pipeline/safety.hpp>
# endif

// Enable the closed loop
// Note: this is not real time safe
// # define CLOSED_LOOP

// Enable the threading with condition variables
// Note: this is not real time safe
// # define COND_VAR_VER

/**
 * \brief Interface controller for the stack of tasks. It is wrapped by sot_controller.
 */

typedef std::vector<hardware_interface::JointHandle> joints_t;
typedef std::vector<std::string> jointNames_t;
typedef std::vector<double> stdVector_t;
typedef boost::mutex mutex_t;

namespace sot_controller
{

class SotDevice : public dynamicgraph::sot::Device
{
    DYNAMIC_GRAPH_ENTITY_DECL();
public:
    /// \brief Default constructor
    ///
    /// \param entityName entity name in the dynamic-graph pool
    SotDevice(const std::string& entityName);
    ~SotDevice();

    /// \name Inherited control methods.
    /// \{

    /// \brief Non real-time device start.
    bool init(unsigned int jointsSize);

    /// \brief Called when the plug-in is started.
    void starting(const stdVector_t& initial_configuration);

    /// \}

    /// \brief Trigger the device computation.
    void runDevice(const ros::Duration& period);

    /// \brief Compute the new state.
    void computeNewState();

    /// \brief Get the execution state of the device.
    bool getDeviceStatus();

    /// \brief Set the execution state of the device.
    void setDeviceStatus(bool status);

    /// \brief Check if the thread should be killed.
    bool getKillThreadStatus();

    /// \brief Set the kill signal.
    void setKillThreadStatus(bool kill);

    /// \brief Wait for the trigger.
    /// \brief Returns whether thread was killed
    bool waitTillTriggered();

    /// \brief Start the thread.
    void startThread();

    /// \brief Stop the thread.
    void stopThread();

    /// \brief Set the robot state {position,velocity}, function used by the device.
    void setSharedState(ml::Vector const &inputPosition, ml::Vector const &inputVelocity);

    /// \brief Get the robot state {position,velocity}, function used by the controller.
    bool getSharedState(stdVector_t &outputPosition, stdVector_t &outputVelocity);

    /// \brief Get the robot position.
    void getSharedPosition(ml::Vector &outputPosition);
    void getSharedPosition(stdVector_t &outputPosition);

    /// \brief Set the robot position.
    void setSharedPosition(ml::Vector const &inputPosition);
    void setSharedPosition(stdVector_t const &inputPosition);

    /// \brief Get the robot velocity.
    void getSharedVelocity(ml::Vector &outputVelocity);
    void getSharedVelocity(stdVector_t &outputVelocity);

    /// \brief Set the robot velocity.
    void setSharedVelocity(ml::Vector const &inputVelocity);
    void setSharedVelocity(stdVector_t const &inputVelocity);

    /// \brief Signal velocity.
    dynamicgraph::Signal<ml::Vector,int> velocitySOUT;

    /// \brief Sample time, given by the controller manager.
    dynamicgraph::Signal<double,int> dtSOUT;

# ifdef COLLISION_CHECK_DEVICE
    /// \brief Safety checker.
    boost::shared_ptr<pipeline::BipedSafety>  bs_;
# endif

private:

    /// \name Attributes used for threading and synchronization with the controller.
    /// \{
    ml::Vector shared_position_;
    ml::Vector shared_velocity_;
    mutex_t mtx_run_;
    mutex_t mtx_position_;
    mutex_t mtx_velocity_;
    mutex_t mtx_state_; // state = {position, velocity}
# ifdef COND_VAR_VER
    boost::condition_variable cond_;
# endif

    /// \brief Execution state of the Device, true is running, false is stopped.
# ifdef CPP11
    std::atomic<bool> status_;
# else
    mutex_t mtx_status_;
    bool status_;
# endif

    /// \}

    /// \brief Object thread.
    boost::thread thread_;

    /// \brief Stop the thread.
# ifdef CPP11
    std::atomic<bool> killThread_;
# else
    mutex_t mtx_kill_thread_;
    bool killThread_;
# endif

    /// \brief Time period.
    ros::Duration period_;

    /// \brief Control value used to compute the new state.
    ml::Vector control_;

    /// \brief Default dimension for the free flyer pose.
    static const unsigned int offset_ = 6;

    /// \brief Called at each control loop.
    void update();

};

}

# endif
