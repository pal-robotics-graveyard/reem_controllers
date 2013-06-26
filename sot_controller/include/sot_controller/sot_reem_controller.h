# ifndef SOT_REEM_CONTROLLER_H
# define SOT_REEM_CONTROLLER_H

# include <forward_command_controller/forward_command_controller.h>
# include <dynamic_graph_bridge/ros_interpreter.hh>
# include "sot_controller/sot_reem_device.h"

/**
 * \brief Position controller for the REEM robot. It is wrapping sot_reem_device.
 *
 * This class is interfaced with ROS ... should be...
 *
 */

namespace sot_reem_controller
{
  class SotReemController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
  {

  private:
    /// Embedded python interpreter accessible via a ROS service.
    dynamicgraph::Interpreter interpreter_;
    /// Pointer to Entity StackOfTasks
    sot_reem_device::SotReemDevice* device_;

  public:
    SotReemController();
    ~SotReemController();

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

    joints_t joints_;

  };
}

# endif
