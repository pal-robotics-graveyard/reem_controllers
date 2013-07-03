# include <sot_controller/sot_reem_controller.h>
# include <pluginlib/class_list_macros.h>

# include <strings.h>
# include <Python.h>
# include <dynamic_graph_bridge/ros_init.hh>
# include <dynamic_graph_bridge/RunCommand.h>

const std::string out_python_file("/tmp/sot_reem_controller.out");

namespace sot_reem_controller {

static void
runPython(std::ostream& file, const std::string& command, dynamicgraph::Interpreter& interpreter)
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
        }
        else
            file << lres << std::endl;
    }
}

SotReemController::SotReemController():
    interpreter_ (dynamicgraph::rosInit (false)),
    device_ (new SotReemDevice("robot_device")) {}

SotReemController::~SotReemController() {
    for (int i = 0; i < joints_.size(); ++i){
        ROS_INFO("Current joint_%d position: %f64\n",i+1,joints_[i].getPosition());
    }
}

bool SotReemController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{

    // Call prologue
    try
    {

        std::ofstream aof(out_python_file.c_str());

        //runPython (aof,"from IPython import embed", interpreter_);
        //runPython (aof,"embed()", interpreter_);
        //runPython (aof, "import sys, os", interpreter_);
        //runPython (aof, "pythonpath = '" + pythonpath + "'", interpreter_);

        runPython (aof, "import sys, os", interpreter_);
        runPython (aof, "pythonpath = os.environ['PYTHONPATH']", interpreter_);
        runPython (aof, "path = []", interpreter_);
        runPython (aof,
                   "for p in pythonpath.split(':'):\n"
                   "  if p not in sys.path:\n"
                   "    path.append(p)", interpreter_);
        runPython (aof, "path.extend(sys.path)", interpreter_);
        runPython (aof, "sys.path = path", interpreter_);
        //runPython(aof,"import roslib; roslib.load_manifest('sot_controller')", interpreter_);
        //runPython(aof,"import tf", interpreter_);
        //int nDofs = actuatedJointsNamesList.size();
        //runPython (aof,"initConf = (0.,) * "+static_cast<std::ostringstream*>( &(std::ostringstream() << nDofs) )->str(),interpreter_);
        runPython(aof,"from dynamic_graph.sot.reem.prologue import robot, solver", interpreter_);
        aof.close();
    }

    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("failed to initialize controller: " << e.what());
        return false;
    }
    catch(...)
    {
        ROS_ERROR_STREAM
                ("unknown exception catched during controller initialization");
        return false;
    }

    // Get joint names from the parameter server
    using namespace XmlRpc;
    XmlRpcValue joint_names;
    ros::NodeHandle nh;
    if (!nh.getParam("jrl_map", joint_names)) //TODO: root_nh or a new global named node?
    {
        ROS_ERROR_STREAM("No joints given (namespace:" << nh.getNamespace() << ").");
        return false;
    }
    if (joint_names.getType() != XmlRpcValue::TypeArray)
    {
        ROS_ERROR_STREAM("Malformed joint specification (namespace:" << nh.getNamespace() << ").");
        return false;
    }

    std::vector<hardware_interface::JointHandle> joints_tmp; // Temporary container of joint handles

    joints_tmp.resize(joint_names.size());
    std::fill(joints_tmp.begin(),joints_tmp.end(),hardware_interface::JointHandle()); // Maybe it is not so efficient
    for (int i = 0; i < joint_names.size(); ++i)
    {
        XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpcValue::TypeString)
        {
            ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << nh.getNamespace() << ").");
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
    }

    // Member list of joint handles is updated only once all resources have been claimed
    joints_ = joints_tmp;

    // Call init inside sot_reem_device
    device_->init();

    return true;
}

void SotReemController::starting(const ros::Time& time) {

    try{
        device_->starting(time,joints_);
    }
    catch(const std::range_error& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

void SotReemController::update(const ros::Time& time, const ros::Duration& period) {

    device_->update(time,period,joints_);

}


}// namespace

PLUGINLIB_DECLARE_CLASS(sot_reem_controller,
                        SotReemController,
                        sot_reem_controller::SotReemController,
                        controller_interface::ControllerBase)
