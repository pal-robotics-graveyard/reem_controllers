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

# include "sot_controller/sot_reem_device.h"

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/debug.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-factory.hh>
#include <dynamic-graph/all-commands.h>

#include <dynamic-graph/all-commands.h>
#include "sot/core/api.hh"

using sot_reem_controller::SotReemDevice;
using dynamicgraph::sot::ExceptionFactory;

const std::string SotReemDevice::CLASS_NAME = "SotReemDevice";

SotReemDevice::SotReemDevice(const std::string& entityName):
    dynamicgraph::sot::Device(entityName),
    run_sot_(false)

{}

SotReemDevice::~SotReemDevice(){}


bool SotReemDevice::init()
{
    return true;
}

void SotReemDevice::starting(const ros::Time& time,joints_t& joints_){


    // Read state from motor command
    int t = stateSOUT.getTime () + 1;
    maal::boost::Vector state = stateSOUT.access (t);

    sotDEBUG (25) << "stateSOUT.access (" << t << ") = " << state << std::endl;
    sotDEBUG (25) << "state.size () = " << state.size () << std::endl;
    sotDEBUG (25) << "joints_.size () = " << joints_.size () << std::endl;

    if (state.size() == joints_.size() + offset_)
        for (unsigned int i = 0 ; i < joints_.size() ; i++){
            state(i+offset_) = joints_[i].getPosition();
            //std::cout<<"joint: "<<joints_[i].getName()<<" init state: "<<state(i+offset)<<std::endl;
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

void SotReemDevice::startThread(const ros::Time& time, const ros::Duration& period){

    m_Thread_ = boost::thread(&SotReemDevice::update, this, time, period);

}

void SotReemDevice::stopThread(){

    m_Thread_.join();

}

ml::Vector SotReemDevice::getState(){
    boost::unique_lock<mutex_t> guard(mtx_state_);
    std::cout<<" getState "<<boost::this_thread::get_id()<<std::endl;
    ml::Vector outputState;
    outputState.resize(shared_state_.size()-6);
    for (unsigned int i = 0; i<outputState.size(); i++){
        outputState(i) = shared_state_(i+offset_);
    }

    return outputState;

}

void SotReemDevice::setState(ml::Vector state){
    boost::unique_lock<mutex_t> guard(mtx_state_);
    std::cout<<" setState "<<boost::this_thread::get_id()<<std::endl;
    shared_state_ = state;
}

void SotReemDevice::WaitSot() {
    boost::unique_lock<mutex_t> guard(mtx_run_);
    std::cout<<" WaitSot START "<<boost::this_thread::get_id()<<" run_sot_ "<<GetStatus()<<std::endl;
    //while(!run_sot_)
    while(!GetStatus())
    {
        std::cout<<" WaitSot WHILE "<<boost::this_thread::get_id()<<std::endl;
        cond_.wait(guard);
    }
    // Integrate control
    try
    {
        std::cout<<" WaitSot INCREMENT "<<boost::this_thread::get_id()<<" run_sot_ "<<GetStatus()<<std::endl;
        increment(0.001); // TODO: Now dt is hardcoded...
        setState(state_);
        //run_sot_ = false;
    }
    catch (...)
    {}
}

void SotReemDevice::RunSot() {
    {
        boost::lock_guard<mutex_t> guard(mtx_run_);
        std::cout<<" RunSot "<<boost::this_thread::get_id()<<std::endl;
        //run_sot_ = true;
        SetStatus(true);
    }
    cond_.notify_one();
}

bool SotReemDevice::GetStatus() {
    boost::lock_guard<mutex_t> guard(mtx_status_);
    return run_sot_;
}

void SotReemDevice::SetStatus(bool status) {
    boost::lock_guard<mutex_t> guard(mtx_status_);
    run_sot_ = status;
}


void SotReemDevice::update(const ros::Time& time, const ros::Duration& period){

    while(true){
        WaitSot();
        SetStatus(false);
    }

}

