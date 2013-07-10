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

# ifndef SOT_REEM_CONTROLLER_H
# define SOT_REEM_CONTROLLER_H

# include <dynamic_graph_bridge/ros_interpreter.hh>
# include "sot_controller/sot_reem_device.h"

/**
 * \brief Position controller for the REEM robot. It is wrapping sot_reem_device.
 *
 * This class works as interface for ros_control.
 *
 */

namespace sot_reem_controller
{
  class SotReemController : public controller_interface::Controller<hardware_interface::PositionJointInterface>,
          public dynamicgraph::sot::AbstractSotExternalInterface
  {

  private:

    /// Device entity for the StackOfTasks
    SotReemDevice device_;

  public:

    static const std::string LOG_PYTHON;

    /// Embedded python interpreter accessible via Corba
    boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

    SotReemController();
    ~SotReemController();

    /// Derived from controller_interface
    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

    /// Derived from AbstractSotExternalInterface
    void setupSetSensors(std::map<std::string,dynamicgraph::sot::SensorValues> &sensorsIn) {}
    void nominalSetSensors(std::map<std::string,dynamicgraph::sot::SensorValues> &sensorsIn) {}
    void cleanupSetSensors(std::map<std::string,dynamicgraph::sot::SensorValues> &sensorsIn) {}
    void getControl(std::map<std::string,dynamicgraph::sot::ControlValues> &) {}

    joints_t joints_;


  protected:

    void runPython(std::ostream& file, const std::string& command, dynamicgraph::Interpreter& interpreter);

    virtual void startupPython();

  };
}

# endif
