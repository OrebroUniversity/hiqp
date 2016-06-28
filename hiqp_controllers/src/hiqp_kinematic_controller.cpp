#include <hiqp_kinematic_controller.h>

#include <pluginlib/class_list_macros.h> // to allow the controller to be loaded as a plugin

#include <iostream>

namespace hiqp
{
	









HiQP_Kinematic_Controller::HiQP_Kinematic_Controller()
: n_joints_(0), is_active_(true)
{
}










HiQP_Kinematic_Controller::~HiQP_Kinematic_Controller() noexcept
{
}










bool HiQP_Kinematic_Controller::init
(
	hardware_interface::VelocityJointInterface *hw, 
	ros::NodeHandle &controller_nh
)
{
	// Store the handle of the node that runs this controller
	controller_nh_ = controller_nh;

	// Load the names of all joints specified in the .yaml file
	std::string param_name = "joints";
	if (!controller_nh.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("In HiQP_Kinematic_Controller: Call to getParam('" 
        	<< param_name 
        	<< "') in namespace '" 
        	<< controller_nh.getNamespace() 
        	<< "' failed.");
        return false;
    }

    // Store teh number of joints for convenience
    n_joints_ = joint_names_.size();
    ROS_INFO_STREAM("In HiQP_Kinematic_Controller: Found " 
    	<< n_joints_ << " joints.");

    // Load all joint handles for all joint name references
	for (auto&& name : joint_names_)
	{
		try
		{
			joint_handles_.push_back(hw->getHandle(name));
		}
		catch (const hardware_interface::HardwareInterfaceException& e)
		{
			ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
		}
	}

	// Set the position and control vectors to all zero
	kdl_joint_pos_ = KDL::JntArray(n_joints_);
	output_controls_ = std::vector<double>(n_joints_, 0.0);

	// Load the urdf-formatted robot description to build a KDL tree
	std::string full_parameter_path;
    std::string robot_urdf;
    if (controller_nh_.searchParam("robot_description", full_parameter_path))
    {
        controller_nh_.getParam(full_parameter_path, robot_urdf);
        ROS_ASSERT(kdl_parser::treeFromString(robot_urdf, kdl_tree_));
    }
    else
    {
        ROS_ERROR_STREAM("In HiQP_Kinematic_Controller: Could not find"
        	<< " parameter 'robot_description' on the parameter server.");
        return false;
    }

	return true;
}










void HiQP_Kinematic_Controller::starting
(
	const ros::Time& time
)
{}










void HiQP_Kinematic_Controller::update
(
	const ros::Time& time, 
	const ros::Duration& period
)
{
	// If the controller is inactive just return
	if (!is_active_) return;



	// Lock the mutex and read all joint positions from the handles
	handles_mutex_.lock();
	unsigned int i = 0;
	for (auto&& handle : joint_handles_)
		kdl_joint_pos_(i++) = handle.getPosition();
	handles_mutex_.unlock();



	// Calculate the kinematic controls
	task_manager_.getKinematicControls(kdl_tree_, 
									   kdl_joint_pos_,
									   n_joints_, 
									   output_controls_);



	// Lock the mutex and write the controls to the joint handles
	handles_mutex_.lock();
	i = 0;
	for (auto&& handle : joint_handles_)
		handle.setCommand(output_controls_.at(i++));
	handles_mutex_.unlock();



	return;
}










void HiQP_Kinematic_Controller::stopping
(
	const ros::Time& time
)
{}






























/*** PRIVATE ***/



} // namespace HiQP_Kinematic_Controller


// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp::HiQP_Kinematic_Controller, controller_interface::ControllerBase)
