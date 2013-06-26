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
    dynamicgraph::sot::Device(entityName)
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

    if (state.size() == joints_.size() + offset)
        for (unsigned int i = 0 ; i < joints_.size() ; i++){
            state(i+offset) = joints_[i].getPosition();
            std::cout<<"joint: "<<joints_[i].getName()<<" init state: "<<state(i+offset)<<std::endl;
        }
    else{
        std::stringstream err;
        err << "state.size() ("<< state.size() <<") is different from joints_.size() + offset ("<< (joints_.size()) <<").";
        throw std::range_error(err.str());
    }

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

    for (unsigned int i = 0; i<joints_.size(); i++){
        joints_[i].setCommand(state_(i+offset));
        std::cout<<"Joint: "<<i+1<<" name: "<<joints_[i].getName()<<" effort: "<<state_(i+offset)<<std::endl;
    }

}


