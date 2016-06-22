#include <mycontroller.h>

#include <iostream>

namespace mycontroller
{
	


MyController::MyController()
{
}



MyController::~MyController() noexcept
{
}



bool MyController::init
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



void MyController::starting
(
	const ros::Time& time
)
{

}



void MyController::update
(
	const ros::Time& time, 
	const ros::Duration& period
)
{

}



void MyController::stopping
(
	const ros::Time& time
)
{

}




















/*** PRIVATE ***/



} // namespace mycontroller