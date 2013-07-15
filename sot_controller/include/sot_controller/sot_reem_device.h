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

# ifndef SOT_REEM_DEVICE_H
# define SOT_REEM_DEVICE_H

# include <sot/core/device.hh>
# include <forward_command_controller/forward_command_controller.h>

/**
 * \brief Interface controller for the stack of tasks. It is wrapped by sot_reem_controller.
 */

typedef std::vector<hardware_interface::JointHandle> joints_t;

namespace sot_reem_controller
{

class SotReemDevice : public dynamicgraph::sot::Device
{
    DYNAMIC_GRAPH_ENTITY_DECL();
public:
    /// \brief Default constructor
    ///
    /// \param entityName entity name in the dynamic-graph pool
    SotReemDevice(const std::string& entityName);
    ~SotReemDevice();

    /// \name Inherited control methods.
    /// \{

    /// \brief Non real-time device start.
    bool init();

    /// \brief Called when plug-in is started.
    void starting(const ros::Time& time,joints_t& joints_);

    /// \brief Called at each control loop.
    void update(const ros::Time& time, const ros::Duration& period, joints_t& joints_);

    /// \}

private:
    /// \brief Default offset.
    static const unsigned int offset = 6;
};

}

# endif