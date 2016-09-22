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

/*
 * \file   ros_kinematics_controller.cpp
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <pluginlib/class_list_macros.h> // to allow the controller to be loaded as a plugin

#include <iostream>
#include <string>
#include <unistd.h> // usleep()

#include <XmlRpcValue.h>  
#include <XmlRpcException.h> 

#include <hiqp/ros_kinematics_controller.h>
#include <hiqp/hiqp_utils.h>

#include <hiqp_msgs_srvs/PerfMeasMsg.h>
#include <hiqp_msgs_srvs/MonitorDataMsg.h>
#include <hiqp_msgs_srvs/Vector3d.h>
 #include <hiqp_msgs_srvs/StringArray.h>

#include <geometry_msgs/PoseStamped.h> // teleoperation magnet sensors





namespace hiqp
{





////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
////////////////////////////////////////////////////////////////////////////////
//
//              R O S   C O N T R O L L E R   I N T E R F A C E
//
////////////////////////////////////////////////////////////////////////////////

ROSKinematicsController::ROSKinematicsController()
: is_active_(true), monitoring_active_(false), task_manager_(&ros_visualizer_)
{
}





ROSKinematicsController::~ROSKinematicsController() noexcept {}

void ROSKinematicsController::starting(const ros::Time& time) {
  logfile_.open("hiqp_log.txt", std::ios::out);
}

void ROSKinematicsController::stopping(const ros::Time& time) {
  logfile_.close();
}





bool ROSKinematicsController::init
(
  hardware_interface::VelocityJointInterface *hw,
  ros::NodeHandle &controller_nh
)
{
  controller_nh_ = controller_nh;

  hardware_interface_ = hw;

  ros_visualizer_.init(&controller_nh_);

  if (loadFps() != 0) return false;

  if (loadAndSetupTaskMonitoring() != 0) return false;

  if (loadUrdfAndSetupKdlTree() != 0) return false;

  if (loadJointsAndSetJointHandlesMap() != 0) return false;

  sampleJointValues();

  addAllTopicSubscriptions();

  advertiseAllServices();

  task_manager_.init(n_controls_);

  loadJointLimitsFromParamServer();

  loadGeometricPrimitivesFromParamServer();

  loadTasksFromParamServer();

  return true;
}





void ROSKinematicsController::update
(
  const ros::Time& time, 
  const ros::Duration& period
)
{
  if (!is_active_) return;

  time_since_last_sampling_ += period.toSec();
  if (time_since_last_sampling_ >= 1/fps_)
  {
    sampleJointValues();

    task_manager_.getKinematicControls(sampling_time_,
                       kdl_tree_, 
                       kdl_joint_pos_vel_,
                       output_controls_);

    setControls();

    // logfile_ << sampling_time_.toSec() << ",qdot";
    // for (auto&& c : output_controls_)
    // {
    //   logfile_ << "," << c;
    // }
    // logfile_ << "\n";

    // std::vector<TaskMonitoringData> data;
    // task_manager_.getTaskMonitoringData(data);

    // for (auto&& d : data)
    // {
    //   if (d.task_name_.compare("minjerk_task") == 0 && 
    //       d.measure_tag_.compare("J") == 0)
    //   {
    //     logfile_ << sampling_time_.toSec() << ",J";
    //     for (auto&& pm : d.performance_measures_)
    //     {
    //       logfile_ << "," << pm;
    //     }
    //     logfile_ << "\n";
    //   }
    // }

    time_since_last_sampling_ = 0;
  }

  task_manager_.getGeometricPrimitiveMap()->redrawAllPrimitives();

  performMonitoring();

  return;
}









////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
////////////////////////////////////////////////////////////////////////////////
//
//                R O S   S E R V I C E   C A L L B A C K S
//
////////////////////////////////////////////////////////////////////////////////





bool ROSKinematicsController::addTask
(
  hiqp_msgs_srvs::AddTask::Request& req, 
    hiqp_msgs_srvs::AddTask::Response& res
)
{
  int retval = task_manager_.addTask(
    req.name, req.type, req.behaviour,
    req.priority, req.visibility, req.active, 
    req.parameters,
    sampling_time_,
    kdl_tree_,
    kdl_joint_pos_vel_
  );

  res.success = (retval < 0 ? false : true);

  return true;
}





bool ROSKinematicsController::updateTask
(
    hiqp_msgs_srvs::UpdateTask::Request& req, 
    hiqp_msgs_srvs::UpdateTask::Response& res
)
{
  
  int retval = task_manager_.updateTask(req.name, req.type, req.behaviour,
                        req.priority, req.visibility, req.active, 
                        req.parameters,
                        sampling_time_,
                        kdl_tree_,
                        kdl_joint_pos_vel_);

  res.success = (retval < 0 ? false : true);

  return true;
}





bool ROSKinematicsController::removeTask
(
  hiqp_msgs_srvs::RemoveTask::Request& req, 
    hiqp_msgs_srvs::RemoveTask::Response& res
)
{
  res.success = false;
  if (task_manager_.removeTask(req.task_name) == 0)
    res.success = true;

  if (res.success)
  {
    printHiqpInfo("Removed task '" + req.task_name + "'.");
  }
  else
  {
    printHiqpInfo("Couldn't remove task '" + req.task_name + "'!");  
  }

  return true;
}





bool ROSKinematicsController::removeAllTasks
(
    hiqp_msgs_srvs::RemoveAllTasks::Request& req, 
    hiqp_msgs_srvs::RemoveAllTasks::Response& res
)
{
  task_manager_.removeAllTasks();
  printHiqpInfo("Removed all tasks successfully!");
  res.success = true;
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
    printHiqpInfo("Added geometric primitive of type '" 
      + req.type + "' with name '" + req.name + "'.");
  }

  return true;
}





bool ROSKinematicsController::removeGeometricPrimitive
(
    hiqp_msgs_srvs::RemoveGeometricPrimitive::Request& req, 
    hiqp_msgs_srvs::RemoveGeometricPrimitive::Response& res
)
{
  res.success = false;
  if (task_manager_.removeGeometricPrimitive(req.name) == 0)
    res.success = true;

  if (res.success)
  {
    printHiqpInfo("Removed primitive '" + req.name + "' successfully!");
  }
  else
  {
    printHiqpInfo("Couldn't remove primitive '" + req.name + "'!");  
  }

  return true;
}





bool ROSKinematicsController::removeAllGeometricPrimitives
(
    hiqp_msgs_srvs::RemoveAllGeometricPrimitives::Request& req, 
    hiqp_msgs_srvs::RemoveAllGeometricPrimitives::Response& res
)
{
  task_manager_.removeAllGeometricPrimitives();
  printHiqpInfo("Removed all primitives successfully!");
  res.success = true;
  return true;
}









////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
////////////////////////////////////////////////////////////////////////////////
//
//                      P R I V A T E   M E T H O D S
//
////////////////////////////////////////////////////////////////////////////////





void ROSKinematicsController::sampleJointValues()
{
  // Lock the mutex and read all joint positions from the handles

  KDL::JntArray& q = kdl_joint_pos_vel_.q;
  KDL::JntArray& qdot = kdl_joint_pos_vel_.qdot;
  handles_mutex_.lock();

  for (auto&& handle : joint_handles_map_)
  {
    q(handle.first) = handle.second.getPosition();
    qdot(handle.first) = handle.second.getVelocity();
  }
  ros::Time t = ros::Time::now();
  sampling_time_.setTimePoint(t.sec, t.nsec);

  handles_mutex_.unlock();
}




void ROSKinematicsController::setControls()
{
  // Lock the mutex and write the controls to the joint handles
  //std::cout << "q = ";
  handles_mutex_.lock();
    for (auto&& handle : joint_handles_map_)
    {
      handle.second.setCommand(output_controls_.at(handle.first));
      //std::cout << output_controls_.at(handle.first) << ", ";
    }
  handles_mutex_.unlock();
  //std::cout << "\n";
}




void ROSKinematicsController::performMonitoring()
{
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

/*
      hiqp_msgs_srvs::PerfMeasMsg q_msg;
      q_msg.task_name = "_q_";
      q_msg.measure_tag = "_q_";
      for (int i=0; i<q.columns(); ++i)
        q_msg.data.push_back( q(i) );
      mon_msg.data.push_back(q_msg);
*/

      hiqp_msgs_srvs::PerfMeasMsg qdot_msg;
      qdot_msg.task_name = "_qdot_";
      qdot_msg.measure_tag = "_qdot_";
      qdot_msg.data.insert(qdot_msg.data.begin(),
                        output_controls_.cbegin(),
                        output_controls_.cend());
      mon_msg.data.push_back(qdot_msg);

      std::vector<TaskMonitoringData>::iterator it = data.begin();
      while (it != data.end())
      {
        hiqp_msgs_srvs::PerfMeasMsg per_msg;
        
        //per_msg.task_id = it->task_id_;
        per_msg.task_name = it->task_name_;
        per_msg.measure_tag = it->measure_tag_;
        per_msg.data.insert(per_msg.data.begin(),
                          it->performance_measures_.cbegin(),
                          it->performance_measures_.cend());

        mon_msg.data.push_back(per_msg);
        ++it;
      }
      
      monitoring_pub_.publish(mon_msg);
    }
  }
}






void ROSKinematicsController::addAllTopicSubscriptions()
{
  // Setup topic subscription
  topic_subscriber_.init( task_manager_.getGeometricPrimitiveMap() );
  
  /*
  topic_subscriber_.addSubscription<geometry_msgs::PoseStamped>(
    controller_nh_, "/wintracker/pose", 100
  );

  topic_subscriber_.addSubscription<hiqp_msgs_srvs::Vector3d>(
    controller_nh_, "/yumi/hiqp_controllers/vector3d", 100
  );
  */

  topic_subscriber_.addSubscription<hiqp_msgs_srvs::StringArray>(
    controller_nh_, "/yumi/hiqp_kinematics_controller/experiment_commands", 100
  );
}





void ROSKinematicsController::advertiseAllServices()
{
  // Advertise available ROS services and link the callback functions
  add_task_service_ = controller_nh_.advertiseService
  (
    "add_task",
    &ROSKinematicsController::addTask,
    this
  );

  update_task_service_ = controller_nh_.advertiseService
  (
    "update_task",
    &ROSKinematicsController::updateTask,
    this
  );

  remove_task_service_ = controller_nh_.advertiseService
  (
    "remove_task",
    &ROSKinematicsController::removeTask,
    this
  );

  remove_all_tasks_service_ = controller_nh_.advertiseService
  (
    "remove_all_tasks",
    &ROSKinematicsController::removeAllTasks,
    this
  );

  add_geomprim_service_ = controller_nh_.advertiseService
  (
    "add_primitive",
    &ROSKinematicsController::addGeometricPrimitive,
    this
  );

  remove_geomprim_service_ = controller_nh_.advertiseService
  (
    "remove_primitive",
    &ROSKinematicsController::removeGeometricPrimitive,
    this
  );

  remove_all_geomprims_service_ = controller_nh_.advertiseService
  (
    "remove_all_primitives",
    &ROSKinematicsController::removeAllGeometricPrimitives,
    this
  );
}





int ROSKinematicsController::loadJointsAndSetJointHandlesMap()
{
  // Load the names of all joints specified in the .yaml file
  std::string param_name = "joints";
  std::vector< std::string > joint_names;
  if (!controller_nh_.getParam(param_name, joint_names))
    {
        ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('" 
          << param_name 
          << "') in namespace '" 
          << controller_nh_.getNamespace() 
          << "' failed.");
        return -1;
    }

    // Load all joint handles for all joint name references
  for (auto&& name : joint_names)
  {
    try
    {
      unsigned int q_nr = kdl_getQNrFromJointName(kdl_tree_, name);
      joint_handles_map_.insert( 
        JointHandleMapEntry(q_nr, hardware_interface_->getHandle(name))
      );
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return -2;
    }
    // catch (MAP INSERT FAIL EXCEPTION)
    // catch (HIQP Q_NR NOT AVAILABLE EXCEPTION)
  }

  // Set the joint position and velocity and the control vectors to all zero
  unsigned int n_joint_names = joint_names.size();
  n_controls_ = kdl_tree_.getNrOfJoints();
  if (n_joint_names > n_controls_)
  {
    ROS_ERROR_STREAM("In ROSKinematicsController: The .yaml file"
      << " includes more joint names than specified in the .urdf file."
      << " Could not succeffully initialize controller. Aborting!\n");
    return -3;
  }
  kdl_joint_pos_vel_.resize(n_controls_);
  output_controls_ = std::vector<double>(n_controls_, 0.0);

  return 0;
}





int ROSKinematicsController::loadFps()
{
  // Load the fps specified in the .yaml file

  if (!controller_nh_.getParam("fps", fps_))
  {
      ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('" 
        << "fps" 
        << "') in namespace '" 
        << controller_nh_.getNamespace() 
        << "' failed.");
      return -1;
  }

  time_since_last_sampling_ = 0;

  return 0;
}





int ROSKinematicsController::loadAndSetupTaskMonitoring()
{
  // Load the monitoring setup specified in the .yaml file

  XmlRpc::XmlRpcValue task_monitoring;
  if (!controller_nh_.getParam("task_monitoring", task_monitoring))
  {
      ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('" 
        << "task_monitoring" 
        << "') in namespace '" 
        << controller_nh_.getNamespace() 
        << "' failed.");
      return -1;
  }

  int active = static_cast<int>(task_monitoring["active"]);
  monitoring_active_ = (active == 1 ? true : false);
  monitoring_publish_rate_ = 
    static_cast<double>(task_monitoring["publish_rate"]);

  monitoring_pub_ = controller_nh_.advertise<hiqp_msgs_srvs::MonitorDataMsg>
  ("monitoring_data", 1);

  return 0;
}





int ROSKinematicsController::loadUrdfAndSetupKdlTree()
{
  // Load the urdf-formatted robot description from the parameter server
  // and build a KDL tree from it

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
        return -1;
    }
    printHiqpInfo("Loaded the robot's urdf model and initialized the KDL tree successfully");
    std::cout << kdl_tree_ << "\n";

    return 0;
}





void ROSKinematicsController::loadJointLimitsFromParamServer()
{
  XmlRpc::XmlRpcValue hiqp_preload_jnt_limits;
  if (!controller_nh_.getParam("hiqp_preload_jnt_limits", hiqp_preload_jnt_limits))
    {
      ROS_WARN_STREAM("No hiqp_preload_jnt_limits parameter found on "
        << "the parameter server. No joint limits were loaded!");
  }
  else
  {
    bool parsing_success = true;
      for (int i=0; i<hiqp_preload_jnt_limits.size(); ++i)
      {
        try {
          std::string link_frame = static_cast<std::string>(
            hiqp_preload_jnt_limits[i]["link_frame"] );

          XmlRpc::XmlRpcValue& limitations = 
            hiqp_preload_jnt_limits[i]["limitations"];

          std::vector<std::string> parameters;
          parameters.push_back(link_frame);
          parameters.push_back( std::to_string(
            static_cast<double>(limitations[0]) ) );
          parameters.push_back( std::to_string(
            static_cast<double>(limitations[1]) ) );
          parameters.push_back( std::to_string(
            static_cast<double>(limitations[2]) ) );

          task_manager_.addTask(
            link_frame + "_jntlimits",
            "TaskJntLimits",
            std::vector<std::string>(),
            1,
            1,
            false,
            parameters,
            sampling_time_,
            kdl_tree_,
            kdl_joint_pos_vel_
          );
        }
        catch (const XmlRpc::XmlRpcException& e)
        {
          ROS_WARN_STREAM("Error while loading "
            << "hiqp_preload_jnt_limits parameter from the "
            << "parameter server. XmlRcpException thrown with message: "
            << e.getMessage());
          parsing_success = false;
          break;
        }
    }

      if (parsing_success)
        ROS_INFO_STREAM("Loaded and initiated joint limit tasks from .yaml "
          << "file successfully!");
  }
}





void ROSKinematicsController::loadGeometricPrimitivesFromParamServer()
{
  XmlRpc::XmlRpcValue hiqp_preload_geometric_primitives;
  if (!controller_nh_.getParam(
    "hiqp_preload_geometric_primitives", 
    hiqp_preload_geometric_primitives)
    )
    {
      ROS_WARN_STREAM("No hiqp_preload_geometric_primitives parameter "
        << "found on the parameter server. No geometric primitives "
        << "were loaded!");
  }
  else
  {
    bool parsing_success = true;
    for (int i=0; i<hiqp_preload_geometric_primitives.size(); ++i)
      {
        try {
          std::string name = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["name"] );

          std::string type = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["type"] );

          std::string frame_id = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["frame_id"] );

          bool visible = static_cast<bool>(
            hiqp_preload_geometric_primitives[i]["visible"] );

          XmlRpc::XmlRpcValue& color_xml = 
            hiqp_preload_geometric_primitives[i]["color"];

          XmlRpc::XmlRpcValue& parameters_xml = 
            hiqp_preload_geometric_primitives[i]["parameters"];

          std::vector<double> color;
          color.push_back( static_cast<double>(color_xml[0]) );
          color.push_back( static_cast<double>(color_xml[1]) );
          color.push_back( static_cast<double>(color_xml[2]) );
          color.push_back( static_cast<double>(color_xml[3]) );

          std::vector<double> parameters;
          for (int j=0; j<parameters_xml.size(); ++j)
          {
            parameters.push_back( 
              static_cast<double>(parameters_xml[j]) 
            );
          }

          task_manager_.addGeometricPrimitive(
            name, type, frame_id, visible, color, parameters
          );

        }
        catch (const XmlRpc::XmlRpcException& e)
        {
          ROS_WARN_STREAM("Error while loading "
            << "hiqp_preload_geometric_primitives parameter from the "
            << "parameter server. XmlRcpException thrown with message: "
            << e.getMessage());
          parsing_success = false;
          break;
        }
      }

      if (parsing_success)
        ROS_INFO_STREAM("Loaded and initiated geometric primitives from "
          << ".yaml file successfully!");
  }
}





void ROSKinematicsController::loadTasksFromParamServer()
{
  XmlRpc::XmlRpcValue hiqp_preload_tasks;
  if (!controller_nh_.getParam("hiqp_preload_tasks", hiqp_preload_tasks))
    {
      ROS_WARN_STREAM("No hiqp_preload_tasks parameter found on "
        << "the parameter server. No joint limits were loaded!");
  }
  else
  {
    bool parsing_success = true;
      for (int i=0; i<hiqp_preload_tasks.size(); ++i)
      {
        try {

          std::string name = static_cast<std::string>(
            hiqp_preload_tasks[i]["name"] );

          std::string type = static_cast<std::string>(
            hiqp_preload_tasks[i]["type"] );

          XmlRpc::XmlRpcValue& behaviour_xml = 
            hiqp_preload_tasks[i]["behaviour"];

          std::vector<std::string> behaviour;

          for (int j=0; j<behaviour_xml.size(); ++j)
          {
            behaviour.push_back(
              static_cast<std::string>( behaviour_xml[j] ));
          }

          // std::cout << "behaviour_xml.size() = " << behaviour_xml.size() << "\n";
          // std::cout << "behaviour_xml[1] = " << static_cast<std::string>(behaviour_xml[1]) << "\n";
          // std::cout << "behaviour = ";
          // for (auto&& s : behaviour) std::cout << s << ", ";
          // std::cout << "\n";

          unsigned int priority = static_cast<int>(
            hiqp_preload_tasks[i]["priority"] );

          int visilibity__ = static_cast<int>(
            hiqp_preload_tasks[i]["visibility"] );

          bool visibility = (visilibity__ == 0 ? false : true);

          int active__ = static_cast<int>(
            hiqp_preload_tasks[i]["active"] );

          bool active = (active__ == 0 ? false : true);

          XmlRpc::XmlRpcValue& parameters_xml = 
            hiqp_preload_tasks[i]["parameters"];

          std::vector<std::string> parameters;

          for (int j=0; j<parameters_xml.size(); ++j)
          {
            parameters.push_back(
              static_cast<std::string>( parameters_xml[j] ));
          }

          // std::cout << "parameters_xml.size() = " << parameters_xml.size() << "\n";
          // std::cout << "parameters_xml[2] = " << static_cast<std::string>(parameters_xml[2]) << "\n";
          // std::cout << "parameters = ";
          // for (auto&& s : parameters) std::cout << s << ", ";
          // std::cout << "\n";



          task_manager_.addTask(
            name,
            type,
            behaviour,
            priority,
            visibility,
            active,
            parameters,
            sampling_time_,
            kdl_tree_,
            kdl_joint_pos_vel_
          );

        }
        catch (const XmlRpc::XmlRpcException& e)
        {
          ROS_WARN_STREAM("Error while loading "
            << "hiqp_preload_tasks parameter from the "
            << "parameter server. XmlRcpException thrown with message: "
            << e.getMessage());
          parsing_success = false;
          break;
        }
    }

      if (parsing_success)
        ROS_INFO("Loaded and initiated tasks from .yaml file successfully!");
  }
}











} // namespace hiqp


// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp::ROSKinematicsController, controller_interface::ControllerBase)
