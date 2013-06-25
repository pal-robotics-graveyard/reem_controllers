# include "sot_controller/sot_reem_device.h"

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/debug.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-factory.hh>
#include <dynamic-graph/all-commands.h>

#include <dynamic-graph/all-commands.h>
#include "sot/core/api.hh"

namespace sot_reem_device {

using dynamicgraph::sot::ExceptionFactory;

SotReemDevice::SotReemDevice(const std::string& entityName):
    dynamicgraph::sot::Device(entityName),
    //timestep_ (0.001),
    previousState_ ()
    //lastTime_ ()
{}


SotReemDevice::~SotReemDevice(){}


bool SotReemDevice::init()
{
    return true;
}

void SotReemDevice::starting(const ros::Time& time,joints_t& joints_){

    //lastTime_ = time;

    // Read state from motor command
    int t = stateSOUT.getTime () + 1;
    maal::boost::Vector state = stateSOUT.access (t);

    sotDEBUG (25) << "stateSOUT.access (" << t << ") = " << state << std::endl;
    sotDEBUG (25) << "state.size () = " << state.size () << std::endl;
    sotDEBUG (25) << "joints_.size () = " << joints_.size () << std::endl;

    if (state.size() == joints_.size() + offset)
        for (unsigned int i = 0 ; i < joints_.size() ; i++)
            state(i+offset) = joints_[i].getPosition();
    else{
        std::stringstream err;
        err << "state.size() ("<< state.size() <<") is different from joints_.size() + offset ("<< (joints_.size()) <<").";
        //err = ss.str();
        throw std::range_error(err.str());
    }

    previousState_ = state;
    stateSOUT.setConstant (state);

}

void SotReemDevice::update(const ros::Time& time, const ros::Duration& period, joints_t& joints_){

    // Integrate control
    try
    {
        increment (period.toSec());
    }
    catch (...)
    {}

    sotDEBUG (25) << "state = " << state_ << std::endl;
    sotDEBUG (25) << "diff  = " << state_ - previousState_ << std::endl;

    //HACK
    //previousState_ = state_;
    //ros::Duration dt = robot_->getTime () - lastTime_;
    //lastTime_ = time;

    //double gain = 10.0;
    for (unsigned int i = 0; i<joints_.size(); i++){
        // Note: this is an open loop control...
        // double command_ = gain * (state_(i+offset) - joints_[i].getPosition());
        //double command_ = state_(i);
        joints_[i].setCommand(state_(i));
    }

    previousState_ = state_;

}


}

