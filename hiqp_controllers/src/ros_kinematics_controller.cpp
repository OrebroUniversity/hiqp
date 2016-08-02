// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.




/*!
 * \file   ros_kinematics_controller.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/ros_kinematics_controller.h>
#include <hiqp/hiqp_utils.h>

#include <hiqp_msgs_srvs/PerfMeasMsg.h>
#include <hiqp_msgs_srvs/MonitorDataMsg.h>

#include <pluginlib/class_list_macros.h> // to allow the controller to be loaded as a plugin

#include <XmlRpcValue.h>  

#include <iostream>

namespace hiqp
{
	









ROSKinematicsController::ROSKinematicsController()
: is_active_(true), monitoring_active_(false), task_manager_(&ros_visualizer_)
{
}










ROSKinematicsController::~ROSKinematicsController() noexcept
{
}










bool ROSKinematicsController::init
(
	hardware_interface::VelocityJointInterface *hw, 
	ros::NodeHandle &controller_nh
)
{
	// Store the handle of the node that runs this controller
	controller_nh_ = controller_nh;
	ros_visualizer_.init(&controller_nh_);



	// Load the names of all joints specified in the .yaml file
	std::string param_name = "joints";
	std::vector< std::string > joint_names;
	if (!controller_nh.getParam(param_name, joint_names))
    {
        ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('" 
        	<< param_name 
        	<< "') in namespace '" 
        	<< controller_nh.getNamespace() 
        	<< "' failed.");
        return false;
    }




    // Load the monitoring setup specified in the .yaml file
	XmlRpc::XmlRpcValue task_monitoring;
	if (!controller_nh.getParam("task_monitoring", task_monitoring))
    {
        ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('" 
        	<< "task_monitoring" 
        	<< "') in namespace '" 
        	<< controller_nh.getNamespace() 
        	<< "' failed.");
        return false;
    }
    int active = static_cast<int>(task_monitoring["active"]);
    monitoring_active_ = (active == 1 ? true : false);
    monitoring_publish_rate_ = 
    	static_cast<double>(task_monitoring["publish_rate"]);
    monitoring_pub_ = controller_nh_.advertise<hiqp_msgs_srvs::MonitorDataMsg>
		("monitoring_data", 1);



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
        ROS_ERROR_STREAM("In ROSKinematicsController: Could not find"
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
		ROS_ERROR_STREAM("In ROSKinematicsController: The .yaml file"
			<< " includes more joint names than specified in the .urdf file."
			<< " Could not succeffully initialize controller. Aborting!\n");
		return false;
	}
	kdl_joint_pos_vel_.resize(n_kdl_joints);
	output_controls_ = std::vector<double>(n_kdl_joints, 0.0);
	task_manager_.setNumControls(n_kdl_joints);




	// Advertise available ROS services and link the callback functions
	add_task_service_ = controller_nh_.advertiseService
	(
		"addTask",
		&ROSKinematicsController::addTask,
		this
	);

	remove_task_service_ = controller_nh_.advertiseService
	(
		"removeTask",
		&ROSKinematicsController::removeTask,
		this
	);

	add_geomprim_service_ = controller_nh_.advertiseService
	(
		"addGeomPrim",
		&ROSKinematicsController::addGeometricPrimitive,
		this
	);




	return true;
}










void ROSKinematicsController::starting
(
	const ros::Time& time
)
{}










void ROSKinematicsController::update
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
	{
		handle.second.setCommand(output_controls_.at(handle.first));
	}
	handles_mutex_.unlock();



	// If monitoring is turned on, generate monitoring data and publish it
	if (monitoring_active_)
	{
		ros::Time now = ros::Time::now();
		ros::Duration d = now - last_monitoring_update_;
		if (d.toSec() >= 1.0/monitoring_publish_rate_)
		{
			last_monitoring_update_ = now;
			std::vector<TaskMonitoringData> data;
			task_manager_.getTaskMonitoringData(data);

			hiqp_msgs_srvs::MonitorDataMsg mon_msg;
			mon_msg.ts = now;

			std::vector<TaskMonitoringData>::iterator it = data.begin();
			while (it != data.end())
			{
				hiqp_msgs_srvs::PerfMeasMsg per_msg;
				
				per_msg.task_id = it->task_id_;
				per_msg.task_name = it->task_name_;
				per_msg.data.insert(per_msg.data.begin(),
					                it->performance_measures_.cbegin(),
					                it->performance_measures_.cend());

				mon_msg.data.push_back(per_msg);
				++it;
			}
			
			monitoring_pub_.publish(mon_msg);

/*
			std::cout << "Monitoring performance:\n";
			std::vector<TaskMonitoringData>::iterator it = data.begin();
			while (it != data.end())
			{
				std::size_t t = it->task_id_;

				std::cout << it->task_id_ << "   "
				          << it->task_name_ << "   ";
				std::vector<double>::const_iterator it2 = it->performance_measures_.begin();
				while (it2 != it->performance_measures_.end())
				{
					std::cout << *it2 << ",";
					it2++;
				}
				std::cout << std::endl;
				it++;
			}
			*/
		}

	}



	return;
}










void ROSKinematicsController::stopping
(
	const ros::Time& time
)
{}







bool ROSKinematicsController::addTask
(
	hiqp_msgs_srvs::AddTask::Request& req, 
    hiqp_msgs_srvs::AddTask::Response& res
)
{
	res.task_id = task_manager_.addTask(req.name, req.type, req.behaviour,
									    req.priority, req.visibility, 
									    req.parameters);

	res.success = (res.task_id < 0 ? false : true);

	if (res.success)
	{
		ROS_INFO_STREAM("Added task of type '" 
			<< req.type << "'"
			<< " with priority " << req.priority
			<< " and identifier " << res.task_id);
	}

	return true;
}


bool ROSKinematicsController::removeTask
(
	hiqp_msgs_srvs::RemoveTask::Request& req, 
    hiqp_msgs_srvs::RemoveTask::Response& res
)
{
	res.success = false;
	if (task_manager_.removeTask(req.task_id) == 0)
		res.success = true;

	if (res.success)
	{
		ROS_INFO_STREAM("Removed task '" << req.task_id << "' successfully!");
	}
	else
	{
		ROS_INFO_STREAM("Couldn't remove task '" << req.task_id << "'!");	
	}

	return true;
}


bool ROSKinematicsController::addGeometricPrimitive
(
    hiqp_msgs_srvs::AddGeometricPrimitive::Request& req, 
    hiqp_msgs_srvs::AddGeometricPrimitive::Response& res
)
{
	int retval = task_manager_.addGeometricPrimitive(
		req.name, req.type, req.frame_id, req.visible, req.color, req.parameters
	);

	res.success = (retval == 0 ? true : false);

	if (res.success)
	{
		ROS_INFO_STREAM("Added geometric primitive of type '" 
			<< req.type << "'"
			<< " with name " << req.name);
	}

	return true;
}




























/*** PRIVATE ***/



} // namespace hiqp


// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp::ROSKinematicsController, controller_interface::ControllerBase)
