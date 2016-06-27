#include <hiqp_kinematic_controller.h>
#include <pluginlib/class_list_macros.h> // to allow the controller to be loaded as a plugin

#include <iostream>

namespace hiqp
{
	


HiQP_Kinematic_Controller::HiQP_Kinematic_Controller()
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
	controller_nh_ = controller_nh;

	std::string param_name = "joints";
	if (!controller_nh.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name 
        	<< "' (namespace: " << controller_nh.getNamespace() << ").");
        return false;
    }

	std::cout << "Joint names: ";
    for (auto&& name : joint_names_)
    {
    	std::cout << name << ", ";
    }
    std::cout << "\n";
}



void HiQP_Kinematic_Controller::starting
(
	const ros::Time& time
)
{

}



void HiQP_Kinematic_Controller::update
(
	const ros::Time& time, 
	const ros::Duration& period
)
{

}



void HiQP_Kinematic_Controller::stopping
(
	const ros::Time& time
)
{

}




















/*** PRIVATE ***/



} // namespace HiQP_Kinematic_Controller


// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp::HiQP_Kinematic_Controller, controller_interface::ControllerBase)
