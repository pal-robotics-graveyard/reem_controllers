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

# include "sot_controller/sot_device.h"

# include <dynamic-graph/factory.h>
# include <dynamic-graph/command-setter.h>
# include <dynamic-graph/debug.h>
# include "soth/debug.hpp"
# include <sot/core/exception-factory.hh>
# include <dynamic-graph/all-commands.h>

// Activate timing functions
//# define TIMING
# ifdef TIMING
# include <time.h>
# include <ctime>
struct timeval start;
struct timeval end;
void start_timing(){
    gettimeofday(&start, NULL);
}
void end_timing(){
    gettimeofday(&end, NULL);
    long elapsed = (end.tv_usec-start.tv_usec);
    std::cout<<(1.0/1000000) * elapsed<<std::endl;
}
# endif


using sot_controller::SotDevice;
using dynamicgraph::sot::ExceptionFactory;

const std::string SotDevice::CLASS_NAME = "SotDevice";

SotDevice::SotDevice(const std::string& entityName):
    dynamicgraph::sot::Device(entityName),
    velocitySOUT("SotDevice("+entityName+")::output(vector)::velocity"),
    dtSOUT("SotDevice("+entityName+")::output(double)::dt"),
    status_(false),
    period_(0.01)
{
    // Register signals into the entity.
    signalRegistration (velocitySOUT);
    signalRegistration (dtSOUT);
}

SotDevice::~SotDevice(){}

bool SotDevice::init(unsigned int jointsSize)
{
    // Read state from dynamic graph and use it to check the sizes
    int t = stateSOUT.getTime();
    maal::boost::Vector state = stateSOUT.access(t);
    const unsigned int deviceStateSize = state.size();
    const unsigned int controllerStateSize = jointsSize + offset_;

    sotDEBUG(25) << "stateSOUT.access ("<< t << ") = " << state << std::endl;
    sotDEBUG(25) << "deviceStateSize = " << deviceStateSize << std::endl;
    sotDEBUG(25) << "controllerStateSize = " << controllerStateSize << std::endl;

    if (deviceStateSize == controllerStateSize){
        // Initialize the shared variables to zero
        // We are not in real time yet
        shared_position_.resize(controllerStateSize);
        shared_velocity_.resize(controllerStateSize);
        return true;
    }
    else {
        std::stringstream err;
        err << "The robot state in dynamic graph has size ["<< deviceStateSize
            <<"], it is different from the state size in the controller ["
            << controllerStateSize <<"].";
        throw std::range_error(err.str());
        return false;
    }
}

void SotDevice::starting(const stdVector_t& initConf){

    // Initialize the shared variable with motor positions
    // Now we are in real time
    setSharedPosition(initConf);
}

void SotDevice::startThread(){
    setKillThreadStatus(false);
    thread_ = boost::thread(&SotDevice::update, this);
}

void SotDevice::stopThread(){
    //setKillThreadStatus(true);
# ifdef COND_VAR_VER
    cond_.notify_all();
# endif
    thread_.interrupt();
}

void SotDevice::setSharedState(ml::Vector const &inputPosition,ml::Vector const &inputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_state_);
    if(inputPosition.size() == shared_position_.size() && inputVelocity.size() == shared_velocity_.size()){ // inputPosition and inputVelocity contain the ffpose
        shared_position_ = inputPosition;
        shared_velocity_ = inputVelocity;
    }
    /*
    else
        for (unsigned i = 0; i<inputPosition.size(); i++){
            shared_position_(i+offset_) = inputPosition(i);
            shared_velocity_(i+offset_) = inputVelocity(i);
        }
        */
}

bool SotDevice::getSharedState(stdVector_t &outputPosition, stdVector_t &outputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_state_,boost::defer_lock);
    if(guard.try_lock()){
        /*
        if(outputPosition.size() == shared_position_.size() && outputVelocity.size() == outputPosition.size()) // outputPosition and outputVelocity contain the ffpose
            for (unsigned i = 0; i<outputPosition.size(); i++){
                outputPosition[i] = shared_position_(i);
                outputVelocity[i] = shared_velocity_(i);
            }
        else*/
        for (unsigned i = 0; i<outputPosition.size(); i++){
            outputPosition[i] = shared_position_(i+offset_);
            outputVelocity[i] = shared_velocity_(i+offset_);
        }
        guard.unlock();
        return true;
    }
    else
        return false;
}

void SotDevice::getSharedPosition(stdVector_t &outputPosition){
    boost::unique_lock<mutex_t> guard(mtx_position_);
    if(outputPosition.size() == shared_position_.size()) // outputPosition contains the ffpose
        for (unsigned i = 0; i<outputPosition.size(); i++)
            outputPosition[i] = shared_position_(i);
    else
        for (unsigned i = 0; i<outputPosition.size(); i++)
            outputPosition[i] = shared_position_(i+offset_);
}

void SotDevice::getSharedPosition(ml::Vector &outputPosition){
    boost::unique_lock<mutex_t> guard(mtx_position_);
    if(outputPosition.size() == shared_position_.size()) // outputPosition contains the ffpose
        outputPosition = shared_position_;
    else
        for (unsigned i = 0; i<outputPosition.size(); i++)
            outputPosition(i) = shared_position_(i+offset_);
}

void SotDevice::setSharedPosition(stdVector_t const &inputPosition){
    boost::unique_lock<mutex_t> guard(mtx_position_);
    if(inputPosition.size() == shared_position_.size()) // inputPosition contains the ffpose
        for (unsigned i = 0; i<inputPosition.size(); i++)
            shared_position_(i) = inputPosition[i];
    else
        for (unsigned i = 0; i<inputPosition.size(); i++)
            shared_position_(i+offset_) = inputPosition[i];
}

void SotDevice::setSharedPosition(ml::Vector const &inputPosition){
    boost::unique_lock<mutex_t> guard(mtx_position_);
    if(inputPosition.size() == shared_position_.size()) // inputPosition contains the ffpose
        shared_position_ = inputPosition;
    else
        for (unsigned i = 0; i<inputPosition.size(); i++)
            shared_position_(i+offset_) = inputPosition(i);
}

void SotDevice::getSharedVelocity(stdVector_t &outputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_velocity_);
    if(outputVelocity.size() == shared_velocity_.size()) // outputVelocity contains the ffpose
        for (unsigned i = 0; i<outputVelocity.size(); i++)
            outputVelocity[i] = shared_velocity_(i);
    else
        for (unsigned i = 0; i<outputVelocity.size(); i++)
            outputVelocity[i] = shared_velocity_(i+offset_);
}

void SotDevice::getSharedVelocity(ml::Vector &outputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_velocity_);
    if(outputVelocity.size() == shared_velocity_.size()) // outputVelocity contains the ffpose
        outputVelocity = shared_velocity_;
    else
        for (unsigned i = 0; i<outputVelocity.size(); i++)
            outputVelocity(i) = shared_velocity_(i+offset_);
}

void SotDevice::setSharedVelocity(stdVector_t const &inputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_velocity_);
    if(inputVelocity.size() == shared_velocity_.size()) // inputVelocity contains the ffpose
        for (unsigned i = 0; i<inputVelocity.size(); i++)
            shared_velocity_(i) = inputVelocity[i];
    else
        for (unsigned i = 0; i<inputVelocity.size(); i++)
            shared_velocity_(i+offset_) = inputVelocity[i];
}

void SotDevice::setSharedVelocity(ml::Vector const &inputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_velocity_);
    if(inputVelocity.size() == shared_velocity_.size()) // inputVelocity contains the ffpose
        shared_velocity_ = inputVelocity;
    else
        for (unsigned i = 0; i<inputVelocity.size(); i++)
            shared_velocity_(i+offset_) = inputVelocity(i);
}

bool  SotDevice::waitTillTriggered()
{
# ifdef COND_VAR_VER
    boost::unique_lock<mutex_t> guard(mtx_run_);
    while(!getDeviceStatus())
    {
        if (getKillThreadStatus())
            return true;
        cond_.wait(guard);
    }
# else
    boost::unique_lock<mutex_t> guard(mtx_run_, boost::defer_lock);
    while(!getDeviceStatus())
    {
        if (getKillThreadStatus())
            return true;
    }
    guard.lock();
# endif
    return false;
}

void SotDevice::computeNewState() {
    if (waitTillTriggered())
        return;
    try{

# ifdef CLOSED_LOOP
        // Get the current state
        getSharedPosition(state_);
# endif

        // Integrate the control to obtain the new state
        increment(period_.toSec());

# ifdef COLLISION_CHECK_DEVICE
        // Convert state maal::boost::Vector in std::vector without freeflyer
        stdVector_t jointPositions_tmp;
        jointPositions_tmp.resize(state_.size()-offset_);
        for (unsigned i = 0; i < jointPositions_tmp.size(); ++i)
            jointPositions_tmp[i] = (state_(i+offset_));
        if(bs_->is_safe(jointPositions_tmp)){
            // Position safe, set the new state
            //setSharedPosition(state_);
            //setSharedVelocity(control_);
            setSharedState(state_,control_);
        }
# else
        // Set the new state
        //setSharedPosition(state_);
        //setSharedVelocity(control_);
        setSharedState(state_,control_);
# endif

        // Export the old control value and dt as dynamic graph signals
        control_ = controlSIN.accessCopy();
        velocitySOUT.setConstant(control_);
        velocitySOUT.setTime(controlSIN.getTime());
        dtSOUT.setConstant(period_.toSec());
        dtSOUT.setTime(controlSIN.getTime());

    }
    catch (...)
    {}
}

void SotDevice::runDevice(const ros::Duration& period) {
# ifdef COND_VAR_VER
    {
        boost::unique_lock<mutex_t> guard(mtx_run_);
        period_ = period;         //Set the internal period
        setDeviceStatus(true);
    }
    cond_.notify_one();
# else
    /*
    boost::unique_lock<mutex_t> guard(mtx_run_);
    period_ = period;         //Set the internal period
    setDeviceStatus(true);
    */
    boost::unique_lock<mutex_t> guard(mtx_run_, boost::defer_lock);
    if(guard.try_lock()){
        period_ = period;         //Set the internal period
        setDeviceStatus(true);
        guard.unlock();
    }

# endif
}

bool SotDevice::getDeviceStatus() {
# ifndef CPP11
    boost::lock_guard<mutex_t> guard(mtx_status_);
    return status_;
# else
    //return status_.load(std::memory_order_relaxed);
    return status_;
# endif
}

void SotDevice::setDeviceStatus(bool status) {
# ifndef CPP11
    boost::lock_guard<mutex_t> guard(mtx_status_);
    status_ = status;
# else
    //status_.store(status,std::memory_order_relaxed);
    status_ = status;
# endif
}

bool SotDevice::getKillThreadStatus() {
# ifndef CPP11
    boost::lock_guard<mutex_t> guard(mtx_kill_thread_);
    return killThread_;
# else
    //return status_.load(std::memory_order_relaxed);
    return killThread_;
# endif
}

void SotDevice::setKillThreadStatus(bool kill) {
# ifndef CPP11
    boost::lock_guard<mutex_t> guard(mtx_kill_thread_);
    killThread_ = kill;
# else
    //status_.store(status,std::memory_order_relaxed);
    killThread_ = kill;
# endif
}

void SotDevice::update(){
    while(!getKillThreadStatus()){
        computeNewState();
        setDeviceStatus(false);
    }
}

