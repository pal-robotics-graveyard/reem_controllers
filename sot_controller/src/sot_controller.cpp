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

#include <sot_controller/sot_controller.h>
#include <pluginlib/class_list_macros.h>

#include <strings.h>
#include <Python.h>
#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/RunCommand.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

namespace sot_controller {

const std::string SotController::LOG_PYTHON="/tmp/sot_controller.out";

SotController::SotController():
    interpreter_ (),
    device_ (new SotDevice("robot_device")) {}

SotController::~SotController() {
    device_->stopThread();

    for (unsigned int i = 0; i < joints_.size(); ++i){
        ROS_INFO("Current joint_%d position: %f64\n",i+1,joints_[i].getPosition());
    }
}

void SotController::runPython(std::ostream& file, const std::string& command, dynamicgraph::Interpreter& interpreter)
{
    file << ">>> " << command << std::endl;
    std::string lerr(""),lout(""),lres("");
    interpreter.runCommand(command,lres,lout,lerr);
    if (lres != "None")
    {
        if (lres=="<NULL>")
        {
            file << lout << std::endl;
            file << "------" << std::endl;
            file << lerr << std::endl;

            std::string err("Exception catched during sot controller initialization, please check the log file: " + LOG_PYTHON);
            throw std::runtime_error(err);
        }
        else
            file << lres << std::endl;
    }
}

void SotController::startupPythonEnv(ros::NodeHandle& controller_nh){

    interpreter_.reset(new dynamicgraph::Interpreter(controller_nh));

    std::ofstream aof(LOG_PYTHON.c_str());

    runPython (aof, "import sys, os", *interpreter_);
    runPython (aof, "pythonpath = os.environ['PYTHONPATH']", *interpreter_);
    runPython (aof, "path = []", *interpreter_);
    runPython (aof,
               "for p in pythonpath.split(':'):\n"
               "  if p not in sys.path:\n"
               "    path.append(p)", *interpreter_);
    runPython (aof, "path.extend(sys.path)", *interpreter_);
    runPython (aof, "sys.path = path", *interpreter_);
    runPython (aof, "sys.argv = 'sot_controller'", *interpreter_);
    runPython(aof,"from sot_ros_api import *", *interpreter_);

    aof.close();

    interpreter_->startRosService();

}

SotDevice* SotController::getDevicePtr(){
    return device_;
}

bool SotController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{

    publisher_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(controller_nh,"joint_states",10));

    // Create the python environment with instances of robot and sot solver
    try
    {
        startupPythonEnv(controller_nh);
    }
    catch(const std::runtime_error& e)
    {
        ROS_ERROR_STREAM(e.what());
        return false;
    }

    // Get joint names from the parameter server, these correspond to the actuated joints parsed by jrl_dynamics_urdf
    using namespace XmlRpc;
    XmlRpcValue joint_names;
    std::string paramName = "jrl_joints_list";
    if (!controller_nh.getParam(paramName, joint_names))
    {
        ROS_ERROR_STREAM("No joints given (expected namespace: /" + paramName + ").");
        return false;
    }
    if (joint_names.getType() != XmlRpcValue::TypeArray)
    {
        ROS_ERROR_STREAM("Malformed joint specification (namespace: /" + paramName + ").");
        return false;
    }

    joints_t joints_tmp; // Temporary container of joint handles
    jointNames_t jointNames_tmp; // Temporary container of joint names

    joints_tmp.resize(joint_names.size());
    jointNames_tmp.resize(joint_names.size());

    std::fill(joints_tmp.begin(),joints_tmp.end(),hardware_interface::JointHandle());
    for (int i = 0; i < joint_names.size(); ++i)
    {
        XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpcValue::TypeString)
        {
            ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << controller_nh.getNamespace() << ").");
            return false;
        }
        const std::string joint_name = static_cast<std::string>(name_value);

        // Get a joint handle
        try
        {
            joints_tmp[i] = robot->getHandle(joint_name);

            ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in the '" <<
                             getHardwareInterfaceType() << "' hardware interface.");
        }
        catch (...)
        {
            ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in the '" <<
                             getHardwareInterfaceType() << "' hardware interface.");
            return false;
        }

        // Get the joint name
        jointNames_tmp[i] = joint_name;

        // Real time publisher initialization
        publisher_->msg_.name.push_back(joint_name);
        publisher_->msg_.position.push_back(0.0);
        publisher_->msg_.velocity.push_back(0.0);

    }

    // Member list of joint handles is updated only once all resources have been claimed
    joints_ = joints_tmp;
    jointNames_ = jointNames_tmp;

    // Load the initial free flyer configuration from the parameter server
    ffpose_ = loadFreeFlyer(controller_nh);

    unsigned int jointsSize = joints_.size();

    // Resize the initial configuration vector
    init_conf_.resize(ffpose_.size() + jointsSize);

    // Resize position_ and velocity_
    // From this point we are assuming they have a constant size
    velocity_.resize(jointsSize);
    position_.resize(jointsSize);

    try{
        // Call non-real time initialization of sot_device
        device_->init(jointsSize);
    }
    catch(const std::range_error& e)
    {
        ROS_ERROR_STREAM(e.what());
    }

# ifdef COLLISION_CHECK
    // Create the self collision checker
    // Note: dt is fixed to 0.0 because we are not going to use velocity limits
    bs_.reset(new pipeline::BipedSafety(&root_nh,jointNames_,0.0));
# endif

# ifdef COLLISION_CHECK_DEVICE
    // Create the self collision checker in the device
    // Note: dt is fixed to 0.0 because we are not going to use velocity limits
    device_->bs_.reset(new pipeline::BipedSafety(&root_nh,jointNames_,0.0));
# endif

    // Create the thread
    device_->startThread();

    return true;
}

void SotController::starting(const ros::Time& time) {
    // Merge the joints_ and ffpose_ vectors to create the initial configuration.
    // The first 6 dofs of the state variable are associated to the Freeflyer frame
    // Freeflyer reference frame should be the same as global
    // frame so that operational point positions correspond to
    // position in freeflyer frame.
    // The freeflyer is automatically updated inside the solver.
    for (unsigned int i = 0; i<ffpose_.size(); i++)
        init_conf_[i] = ffpose_[i];
    for (unsigned int i = ffpose_.size(); i<init_conf_.size(); i++)
        init_conf_[i] = joints_[i-ffpose_.size()].getPosition();

    // Call starting, this is setting up the current robot configuration into the device
    device_->starting(init_conf_);

}

void SotController::update(const ros::Time& time, const ros::Duration& period) {

# ifdef CLOSED_LOOP
    // Read the motor positions
    for (unsigned int i = 0; i<joints_.size(); i++)
        position_[i] = joints_[i].getPosition();

    // Close the loop with the device
    device_->setSharedPosition(position_);
# endif

    // Trigger one sot computation
    device_->runDevice(period);

# ifdef CLOSED_LOOP
    // Let's be patient, wait the end of the computation
    // Note: This is too fast!!!!!!!!!
    // This is not working on the real time machine because it's using all
    // the available cpu.
    while(device_->getDeviceStatus()){}
# endif

    // Take the new position and velocity from the sot
    //device_->getSharedPosition(position_);
    //device_->getSharedVelocity(velocity_);

    device_->getSharedState(position_,velocity_);

# ifdef COLLISION_CHECK
    // Check for collisions
    if(bs_->is_safe(position_,time))
        // Set the motor commands
        for (unsigned int i = 0; i<joints_.size(); i++)
            joints_[i].setCommand(position_[i]);
# else
    // Set the motor commands
    for (unsigned int i = 0; i<joints_.size(); i++)
        joints_[i].setCommand(position_[i]);
# endif

    // Publish the position and the velocity
    if(publisher_->trylock()){
        publisher_->msg_.header.stamp = time;
        for (unsigned int i = 0; i<joints_.size(); i++){
            publisher_->msg_.position[i] = position_[i];
            publisher_->msg_.velocity[i] = velocity_[i];
        }
        publisher_->unlockAndPublish();
    }
}

stdVector_t SotController::loadFreeFlyer(ros::NodeHandle& controller_nh) const
{
    XmlRpc::XmlRpcValue ffpose;
    std::string param_name = "ffpose";
    if (controller_nh.hasParam(param_name)){
        controller_nh.getParam(param_name, ffpose);
        ROS_ASSERT(ffpose.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(ffpose.size() == 6);
        for (int i = 0; i < ffpose.size(); ++i)
        {
            ROS_ASSERT(ffpose[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        }
    }
    else
    {
        ROS_INFO("sot_control: No free flyer pose defined on the parameter server, default pose loaded (0.0,0.0,0.0,0.0,0.0,0.0)");
        ffpose.setSize(6);
        for (int i = 0; i < ffpose.size(); ++i)
            ffpose[i] = 0.0;
    }

    // Convert to standard vector
    stdVector_t res;
    res.resize(ffpose.size());
    for (unsigned int i = 0; i < res.size(); ++i)
        res[i] = static_cast<double>(ffpose[i]);

    return res;
}

}// namespace

PLUGINLIB_DECLARE_CLASS(sot_controller,
                        SotController,
                        sot_controller::SotController,
                        controller_interface::ControllerBase)
