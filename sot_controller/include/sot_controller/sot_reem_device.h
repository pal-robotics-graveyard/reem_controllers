# ifndef SOT_REEM_DEVICE_H
# define SOT_REEM_DEVICE_H

# include <sot/core/device.hh>
# include <forward_command_controller/forward_command_controller.h>

/**
 * \brief Interface controller for the stack of tasks. It is wrapped by sot_reem_controller.
 *
 * This class works as interface for ROS ... why?
 *
 */

namespace sot_reem_device
{

//typedef std::map<std::string, std::pair<boost::shared_ptr<const urdf::Joint>, boost::shared_ptr<pr2_mechanism_model::JointState>>> jointMap_t;

//typedef std::map<std::string,int> stateMap_t;

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

    /// \brief Non real-time entity start.
    bool init();

    /// \brief Called when plug-in is started.
    void starting(const ros::Time& time);

    /// \brief Called at each control loop.
    void update(const ros::Time& time, const ros::Duration& period);

    /// \}

private:
    /// \brief Default integration step (i.e. 1ms for REEM).
    static const double TIMESTEP_DEFAULT;

    double timestep_;
    //stateMap_t stateMap;
    ml::Vector previousState_;
    //std::vector<boost::shared_ptr<control_toolbox::Pid> > pids_;
    ros::Time lastTime_;
    //pr2_mechanism_model::RobotState* robot_;
};
} // end of namespace sot_pr2


# endif
