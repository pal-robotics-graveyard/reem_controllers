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

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/debug.h>
#include "soth/debug.hpp"
#include <sot/core/exception-factory.hh>
#include <dynamic-graph/all-commands.h>

#include <dynamic-graph/all-commands.h>

// Activate some functions to use for timing
//#define TIMING
#ifdef TIMING
#include <time.h>
#include <ctime>
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
#endif

// Activate the threading with condition variables
//#define COND_VAR_VER

using sot_controller::SotDevice;
using dynamicgraph::sot::ExceptionFactory;

const std::string SotDevice::CLASS_NAME = "SotDevice";

SotDevice::SotDevice(const std::string& entityName):
    dynamicgraph::sot::Device(entityName),
    status_(false),
    period_(0.001),
    controlSOUT("SotDevice("+entityName+")::output(vector)::controlOut"),
    dtSOUT("SotDevice("+entityName+")::output(double)::dt")
{
    // Register signals into the entity.
    signalRegistration (controlSOUT);
    signalRegistration (dtSOUT);
}

SotDevice::~SotDevice(){}

bool SotDevice::init()
{
    return true;
}

void SotDevice::starting(joints_t& joints_){

    // Read state from motor command
    int t = stateSOUT.getTime () + 1;
    maal::boost::Vector state = stateSOUT.access (t);

    sotDEBUG (25) << "stateSOUT.access (" << t << ") = " << state << std::endl;
    sotDEBUG (25) << "state.size () = " << state.size () << std::endl;
    sotDEBUG (25) << "joints_.size () = " << joints_.size () << std::endl;

    if (state.size() == joints_.size() + offset_)
        for (unsigned int i = 0 ; i < joints_.size() ; i++){
            state(i+offset_) = joints_[i].getPosition();
        }
    else{
        std::stringstream err;
        err << "state.size() ("<< state.size() <<") is different from joints_.size() + offset ("<< (joints_.size()) <<").";
        throw std::range_error(err.str());
    }

    stateSOUT.setConstant (state);

    // Set the initial conditions
    state_ = state;
    shared_state_ = state;

}

void SotDevice::startThread(){
    thread_ = boost::thread(&SotDevice::update, this);
}

void SotDevice::stopThread(){
    thread_.join(); //TODO: it is correct? Or Should I interrupt the thread?
}

ml::Vector SotDevice::getState(){
    boost::unique_lock<mutex_t> guard(mtx_state_);
    ml::Vector outputState;
    outputState.resize(shared_state_.size()-6);
    for (unsigned int i = 0; i<outputState.size(); i++){
        outputState(i) = shared_state_(i+offset_);
    }
    return outputState;
}

void SotDevice::setState(ml::Vector state){
    boost::unique_lock<mutex_t> guard(mtx_state_);
    shared_state_ = state;
}

void SotDevice::pauseDevice() {
#ifdef COND_VAR_VER
    boost::unique_lock<mutex_t> guard(mtx_run_);
    while(!getDeviceStatus())
    {
        cond_.wait(guard);
    }
#else
    boost::unique_lock<mutex_t> guard(mtx_run_, boost::defer_lock);
    while(!getDeviceStatus()){ }
    guard.lock();
#endif
    // Integrate control
    try{

        increment(period_.toSec());

        // Export some control and dt to the dynamic graph
        control_ = controlSIN.accessCopy();
        controlSOUT.setConstant(control_);
        controlSOUT.setTime(controlSIN.getTime());

        dtSOUT.setConstant(period_.toSec());
        dtSOUT.setTime(controlSIN.getTime());

        setState(state_);

        // If you are going to close the loop, this is the spot where to place the code.

    }
    catch (...)
    {}
}

void SotDevice::runDevice(const ros::Duration& period) {
#ifdef COND_VAR_VER
    {
        boost::unique_lock<mutex_t> guard(mtx_run_);
        period_ = period;         //Set the internal period
        setDeviceStatus(true);
    }
    cond_.notify_one();
#else
    boost::unique_lock<mutex_t> guard(mtx_run_);
    period_ = period;         //Set the internal period
    setDeviceStatus(true);
#endif
}

bool SotDevice::getDeviceStatus() {
    boost::lock_guard<mutex_t> guard(mtx_status_);
    return status_;
}

void SotDevice::setDeviceStatus(bool status) {
    boost::lock_guard<mutex_t> guard(mtx_status_);
    status_ = status;
}

void SotDevice::update(){
    while(true){
        pauseDevice();
        setDeviceStatus(false);
    }
}

