/*!
 * \file   hiqp_kinematic_controller.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/hiqp_kinematic_controller.h>
#include <hiqp/hiqp_utils.h>

#include <pluginlib/class_list_macros.h> // to allow the controller to be loaded as a plugin

#include <iostream>

namespace hiqp
{
	









HiQPKinematicController::HiQPKinematicController()
: is_active_(true)
{
}










HiQPKinematicController::~HiQPKinematicController() noexcept
{
}










bool HiQPKinematicController::init
(
	hardware_interface::VelocityJointInterface *hw, 
	ros::NodeHandle &controller_nh
)
{
	// Store the handle of the node that runs this controller
	controller_nh_ = controller_nh;




	// Load the names of all joints specified in the .yaml file
	std::string param_name = "joints";
	std::vector< std::string > joint_names;
	if (!controller_nh.getParam(param_name, joint_names))
    {
        ROS_ERROR_STREAM("In HiQPKinematicController: Call to getParam('" 
        	<< param_name 
        	<< "') in namespace '" 
        	<< controller_nh.getNamespace() 
        	<< "' failed.");
        return false;
    }




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
        ROS_ERROR_STREAM("In HiQPKinematicController: Could not find"
        	<< " parameter 'robot_description' on the parameter server.");
        return false;
    }
    std::cout << "KDL Tree loaded successfully! Printing it below.\n\n"
    	<< kdl_tree_ << "\n\n";




    // Load all joint handles for all joint name references
	for (auto&& name : joint_names)
	{
		try
		{
			unsigned int q_nr = kdl_getQNrFromJointName(kdl_tree_, name);
			joint_handles_map_.insert( 
				JointHandleMapEntry(q_nr, hw->getHandle(name))
			);
		}
		catch (const hardware_interface::HardwareInterfaceException& e)
		{
			ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
		}
		// catch (MAP INSERT FAIL EXCEPTION)
		// catch (HIQP Q_NR NOT AVAILABLE EXCEPTION)
	}




	// Set the joint position and velocity and the control vectors to all zero
	unsigned int n_joint_names = joint_names.size();
	unsigned int n_kdl_joints = kdl_tree_.getNrOfJoints();
	if (n_joint_names > n_kdl_joints)
	{
		ROS_ERROR_STREAM("In HiQPKinematicController: The .yaml file"
			<< " includes more joint names than specified in the .urdf file."
			<< " Could not succeffully initialize controller. Aborting!\n");
		return false;
	}
	kdl_joint_pos_vel_.resize(n_kdl_joints);
	output_controls_ = std::vector<double>(n_kdl_joints, 0.0);




	// Advertise available ROS services and link the callback functions
	add_task_service_ = controller_nh_.advertiseService
	(
		"addTask",
		&HiQPKinematicController::addTask,
		this
	);




	return true;
}










void HiQPKinematicController::starting
(
	const ros::Time& time
)
{}










void HiQPKinematicController::update
(
	const ros::Time& time, 
	const ros::Duration& period
)
{
	// If the controller is inactive just return
	if (!is_active_) return;



	// Lock the mutex and read all joint positions from the handles
	KDL::JntArray& q = kdl_joint_pos_vel_.q;
	KDL::JntArray& qdot = kdl_joint_pos_vel_.qdot;
	handles_mutex_.lock();
	for (auto&& handle : joint_handles_map_)
	{
		q(handle.first) = handle.second.getPosition();
		qdot(handle.first) = handle.second.getVelocity();
	}
	handles_mutex_.unlock();



	// Calculate the kinematic controls
	task_manager_.getKinematicControls(kdl_tree_, 
									   kdl_joint_pos_vel_,
									   output_controls_);



	// Lock the mutex and write the controls to the joint handles
	handles_mutex_.lock();
	for (auto&& handle : joint_handles_map_)
		handle.second.setCommand(output_controls_.at(handle.first));
	handles_mutex_.unlock();



	return;
}










void HiQPKinematicController::stopping
(
	const ros::Time& time
)
{}







bool HiQPKinematicController::addTask
(
	hiqp_msgs_srvs::AddTask::Request& req, 
    hiqp_msgs_srvs::AddTask::Response& res
)
{
	std::cout << "HiQPKinematicController::addTask\n";

	res.id = task_manager_.addTask(req.task_spec.task, 
								   req.task_spec.behaviour,
								   req.task_spec.behaviour_parameters,
								   req.task_spec.priority,
								   req.task_spec.visibility,
								   req.task_spec.parameters);
	res.success = (res.id < 0 ? false : true);

	return true;
}




























/*** PRIVATE ***/



} // namespace HiQPKinematicController


// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp::HiQPKinematicController, controller_interface::ControllerBase)
