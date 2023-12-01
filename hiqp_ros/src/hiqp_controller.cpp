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

#include "pluginlib/class_list_macros.hpp"

#include <unistd.h>  // usleep()
#include <iostream>
#include <string>

#include <hiqp_ros/hiqp_controller.h>
#include <hiqp_ros/utilities.h>

#include <hiqp_msgs/msg/string_array.hpp>
#include <hiqp_msgs/msg/vector3d.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

#include <controller_interface/controller_interface_base.hpp>
#include "controller_interface/helpers.hpp"
#include<chrono>

using hiqp::TaskMeasure;
using namespace hiqp_ros;
using namespace std::chrono_literals;

////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
////////////////////////////////////////////////////////////////////////////////
//
//              R O S   C O N T R O L L E R   I N T E R F A C E
//
////////////////////////////////////////////////////////////////////////////////

//=====================================================================================
controller_interface::CallbackReturn HiqpController::on_init() {
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "HiQP controller initializing");

  if(!getRobotDescriptionFromServer()) 
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not fetch robot_description from server");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}
        
bool HiqpController::getRobotDescriptionFromServer() {
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), "/robot_state_publisher");
  while (!param_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Service not available, waiting again...");
  }

  auto parameters = param_client->get_parameters({ "robot_description" });
  for (auto& parameter : parameters)
  {
    if (parameter.get_name() == "robot_description")
    {
      urdf_ = parameter.value_to_string();
      break;
    }
  }
  return true;
}

//called during configuration of command interfaces
controller_interface::InterfaceConfiguration HiqpController::command_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "HiQP controller claiming command interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  //conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  //HIQP assumes uniform interface type
  conf.type = controller_interface::interface_configuration_type::ALL;
  if (n_joints_ == 0)
  {
    fprintf(
      stderr,
      "During ros2_control interface configuration, degrees of freedom is not valid;"
      " it should be positive. Actual DOF is %zu\n",
      n_joints_);
    std::exit(EXIT_FAILURE);
  }
  conf.names.reserve(n_joints_ * params_.command_interfaces.size());
  for (const auto & joint_name : command_joint_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

//called during configuration of state interfaces
controller_interface::InterfaceConfiguration HiqpController::state_interface_configuration() const 
{
  RCLCPP_INFO(get_node()->get_logger(), "HiQP controller claiming state interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(n_state_joints_ * params_.state_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf; 
}

//configure parameters
controller_interface::CallbackReturn HiqpController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {

  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(get_node()->get_logger(), "HiQP controller configuring");

  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(logger, "'joints' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  joint_names_ =  params_.joints;
  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty())
  {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }
  else if (command_joint_names_.size() > params_.joints.size())
  {
    RCLCPP_ERROR(
      logger, "Cannot 'command_joints' that are not read in the 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }
  
  // get degrees of freedom
  n_state_joints_ = joint_names_.size();
  n_joints_ = command_joint_names_.size();


  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());

  robot_state_ptr_ = RobotStatePtr(new RobotState()); //.reset(&robot_state_data_);

  rclcpp::Time t = get_node()->get_clock()->now();
  int64_t sec = t.nanoseconds()*1e-9;
  int64_t nsec = t.nanoseconds() - sec*1e9;
  last_sampling_time_point_.setTimePoint(sec, nsec);

  //load urdf into the robot pointer
  if(loadUrdfToKdlTree()<0) {
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger,"Loaded KDL tree");

  //check if all interfaces are of a given type
  auto check_ifce_type = [](const std::vector<std::string> & interface_type_list, const std::string & interface_type) {
    bool has=true;
    for( auto ifce : interface_type_list) has = has && (ifce == interface_type);
    return has;
  };

  is_velocity_ = check_ifce_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  is_acceleration_ = check_ifce_type(params_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
 
  if(!(is_velocity_ || is_acceleration_)) {
    RCLCPP_ERROR(logger, "'command_interfaces' should be either velocity or acceleration for all joints.");
    return CallbackReturn::FAILURE;
  }

  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());

  auto has_ifce_type = [](const std::vector<std::string> & interface_type_list, const std::string & interface_type) {
    return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
  };
  bool has_pos = has_ifce_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  bool has_vel = has_ifce_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);

  if(has_pos && has_vel) {
    RCLCPP_INFO(logger,"We have both position and velocity state info, all good.\n");
  } else {
    RCLCPP_ERROR(logger, "'state_interfaces' need to contain both position and velocity");
    return CallbackReturn::FAILURE;
  }
  
  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };


  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    logger, "Command interfaces are [%s] and state interfaces are [%s].",
    get_interface_list(params_.command_interfaces).c_str(),
    get_interface_list(params_.state_interfaces).c_str());

  //initialize realtime publisher
  c_state_pub_ = std::shared_ptr<RTPublisher> (new RTPublisher(get_node()->create_publisher<hiqp_msgs::msg::JointControllerState>("hiqp_controller_state",1)));
  last_c_state_update_ = get_node()->get_clock()->now();
 
  monitoring_pub_ = std::shared_ptr<MonitorPublisher> (new MonitorPublisher(get_node()->create_publisher<hiqp_msgs::msg::TaskMeasures>("/hiqp_controller/task_measures",1)));
  monitoring_active_ = params_.monitor; 
  monitoring_publish_rate_ = params_.monitor_rate;
  last_monitoring_update_ = get_node()->get_clock()->now();

  c_state_publish_rate_ = params_.state_publish_rate;

  c_state_pub_->msg_.joints.resize(n_state_joints_);
  //c_state_pub_->msg_.sensors.resize(n_sensors_);

  for (auto &&it : robot_state_ptr_->kdl_tree_.getSegments()) {
    if(it.second.q_nr < n_state_joints_) {
      c_state_pub_->msg_.joints.at(it.second.q_nr).name = it.second.segment.getJoint().getName();
    } else {
      RCLCPP_WARN(logger,"Ignoring joint %s with joint number %d", it.second.segment.getJoint().getName().c_str(), it.second.q_nr);
    }
  }

  //initialize topic subscribers, visualization and service handlers
  visualizer_->init(get_node());
  service_handler_.init(get_node(), task_manager_ptr_, this->getRobotState());

  loadRenderingParameters();

  if(params_.load_tf) {
  //  addTfTopicSubscriptions();
  }

  service_handler_.advertiseAll();

  task_manager_ptr_->init(getNJoints(), true);

  //loadJointLimitsFromParamServer();
  //loadGeometricPrimitivesFromParamServer();
  //loadTasksFromParamServer();

  u_vel_ = Eigen::VectorXd::Zero(getNJoints());
  RCLCPP_INFO(logger, "HiQP controller configured");
  return CallbackReturn::SUCCESS;

}

//clears state and will start control after this
controller_interface::CallbackReturn HiqpController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {

  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "HiQP controller activating");

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  //order all joints in storage
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", n_joints_,
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces)
  {
    auto it =
      std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface);
    auto index = std::distance(allowed_state_interface_types_.begin(), it);
    if (it==allowed_state_interface_types_.end()) {
      RCLCPP_ERROR(logger, "Could not find interface of type %s in allowed interfaces",interface.c_str());
      return CallbackReturn::ERROR;
    }
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", n_state_joints_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    } 
  }

  //create the handles in hiqp 
  if(loadJointsAndSetJointHandlesMap() < 0) {
    RCLCPP_ERROR(logger, "failure in setting up joints map");
    return CallbackReturn::FAILURE;
  }

  //read current state
  sampleJointValues();

  return CallbackReturn::SUCCESS;
}

//clean-up back to a state from which we can start the controller
controller_interface::CallbackReturn HiqpController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {

  RCLCPP_INFO(get_node()->get_logger(), "HiQP controller deactivating");
  return CallbackReturn::SUCCESS;
}

//called once every cycle to update --> use realtime tools within this
controller_interface::return_type HiqpController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) {

  period_ = period;
  sampleJointValues();
  //sampleSensorValues();    
  updateControls(ddq_, u_);
  setControls();
  publishControllerState();
  return controller_interface::return_type::OK;
} 

void HiqpController::updateControls(Eigen::VectorXd& dq, Eigen::VectorXd& u) {
  //if (!is_active_) return;

  std::vector<double> _dq(dq.size());

  // Time the acceleration control computation
  auto t_begin = std::chrono::high_resolution_clock::now();
  task_manager_ptr_->getVelocityControls(this->getRobotState(), _dq);
  auto t_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> opt_time = t_end - t_begin;

  int i = 0;
  for (auto&& oc : _dq) {
    dq(i++) = oc;
  }
  u=dq; //u_vel_+ddq*period_.toSec();
        //u_vel_=u; //store the computed velocity controls for the next integration step

  renderPrimitives();
  monitorTasks(static_cast<double>(opt_time.count()));

  return;
}

//=====================================================================================
int HiqpController::loadUrdfToKdlTree() {

  bool success =
    kdl_parser::treeFromString(urdf_, robot_state_ptr_->kdl_tree_);
  if(!success) {
    RCLCPP_ERROR(get_node()->get_logger(),"Could not parse urdf to kdl tree");
    return -1;
  }
  return 0;
}

//=====================================================================================
int HiqpController::loadJointsAndSetJointHandlesMap() {
  
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger,"Setting up joint handle map...");

  auto n_joints_kdl_ = robot_state_ptr_->kdl_tree_.getNrOfJoints();

  if(n_joints_kdl_ != n_state_joints_) {
    RCLCPP_ERROR(logger, "Controller needs state interface access to all joints in the URDF. URDF has %d joints, controller claims %d",
        n_joints_kdl_, n_state_joints_);
    return -1;
  }

  std::vector<unsigned int> qnrs;

  qnrs.clear();
  KDL::SegmentMap all_segments = robot_state_ptr_->kdl_tree_.getSegments();
  std::cerr<<"KDL tree has "<<all_segments.size()<<" elements\n";
  for (KDL::SegmentMap::const_iterator element=all_segments.cbegin(); 
      element!=all_segments.cend(); element++ ) {
    qnrs.push_back(element->second.q_nr);
    std::cerr<<element->first<<" added joint "<<element->second.q_nr
      <<" name "<<element->second.segment.getJoint().getName() 
      <<" for segment "<<element->second.segment.getName()<<std::endl;
  }

  robot_state_ptr_->joint_handle_info_.clear();

  for (auto name = command_joint_names_.begin(); name!=command_joint_names_.end(); name++) {
    unsigned int q_nr =
      hiqp::kdl_getQNrFromJointName(robot_state_ptr_->kdl_tree_, *name);
    RCLCPP_INFO_STREAM(logger, "Command joint found: '" << *name << "', qnr: " << q_nr);
    joint_handles_map_.emplace(q_nr, name-command_joint_names_.begin());
    //robot_state_ptr_->joint_handle_info_.push_back(
    //    hiqp::JointHandleInfo(q_nr, *name, true, true));
  }

  for (auto name = joint_names_.begin(); name!=joint_names_.end(); name++) {
    unsigned int q_nr =
      hiqp::kdl_getQNrFromJointName(robot_state_ptr_->kdl_tree_, *name);
    RCLCPP_INFO_STREAM(logger, "State joint found: '" << *name << "', qnr: " << q_nr);
    joint_state_handles_map_.emplace(q_nr, name-joint_names_.begin());
    
    bool controlled = (std::find(command_joint_names_.begin(), command_joint_names_.end(), *name) != 
        command_joint_names_.end());
    robot_state_ptr_->joint_handle_info_.push_back(
        hiqp::JointHandleInfo(q_nr, *name, true, controlled));
    qnrs.erase(std::remove(qnrs.begin(), qnrs.end(), q_nr), qnrs.end());
  }

  if(qnrs.size()>0) {
    RCLCPP_ERROR(get_node()->get_logger(),"FATAL: There are joints we can't read in the model");
    return -1;
  }

  std::cout << "Joint handle info:\n";
  for (auto &&jhi : robot_state_ptr_->joint_handle_info_) {
    std::cout << jhi.q_nr_ << ", " << jhi.joint_name_ << ", " << jhi.readable_
      << ", " << jhi.writable_ << "\n";
  }

  //state has dimension n_state_joints_
  robot_state_ptr_->kdl_jnt_array_vel_.resize(n_state_joints_);
  KDL::SetToZero(robot_state_ptr_->kdl_jnt_array_vel_.q);
  KDL::SetToZero(robot_state_ptr_->kdl_jnt_array_vel_.qdot);
  robot_state_ptr_->kdl_effort_.resize(n_state_joints_);
  KDL::SetToZero(robot_state_ptr_->kdl_effort_);
  ddq_ = Eigen::VectorXd::Zero(n_state_joints_);
  //commands have dimension n_joints_
  u_ = Eigen::VectorXd::Zero(n_state_joints_);
  return 0;
}
//=====================================================================================
void HiqpController::sampleJointValues() {
  robot_state_ptr_->sampling_time_ = period_.nanoseconds()*1e-9;

  KDL::JntArray &q = robot_state_ptr_->kdl_jnt_array_vel_.q;
  KDL::JntArray &qdot = robot_state_ptr_->kdl_jnt_array_vel_.qdot;
  //KDL::JntArray &effort = robot_state_ptr_->kdl_effort_;
  q.data.setZero();
  qdot.data.setZero();
  //effort.data.setZero();
  //double alpha = 0.05;

  //handles_mutex_.lock();
  //handles_mutex_.unlock();
  
  for (auto &&handle : joint_state_handles_map_) {
    q(handle.first) = joint_state_interface_[0][handle.second].get().get_value();
    qdot(handle.first) = joint_state_interface_[1][handle.second].get().get_value(); 
  }

}
//=====================================================================================
void HiqpController::setControls() {
  //RCLCPP_WARN_STREAM(get_node()->get_logger(),"u = ["<<u_.transpose()<<"]");
  //handles_mutex_.lock();
  for (auto &&handle : joint_handles_map_) {
    joint_command_interface_[0][handle.second].get().set_value(u_(handle.first));
  }
  //handles_mutex_.unlock();
}
//=====================================================================================

void HiqpController::monitorTasks(double acc_ctl_comp_time) {
  if (monitoring_active_) {
    rclcpp::Time now = get_node()->get_clock()->now();
    rclcpp::Duration d = now - last_monitoring_update_;
    if (d.seconds() >= 1.0 / monitoring_publish_rate_) {
      std::vector<TaskMeasure> measures;

      if(monitoring_pub_->trylock()) {
        last_monitoring_update_ = now;
        task_manager_ptr_->getTaskMeasures(measures);

        //std::cerr<<"Generating task measure message with "<<measures.size()<<" tasks\n";
        hiqp_msgs::msg::TaskMeasures msgs;
        msgs.stamp = now;
        for (auto&& measure : measures) {
          hiqp_msgs::msg::TaskMeasure msg;
          msg.task_name = measure.task_name_;
          msg.task_sign = measure.task_sign_;
          msg.e = std::vector<double>(
              measure.e_.data(),
              measure.e_.data() + measure.e_.rows() * measure.e_.cols());
          msg.de = std::vector<double>(
              measure.de_.data(),
              measure.de_.data() + measure.de_.rows() * measure.de_.cols());
          msg.dde_star = std::vector<double>(
              measure.dde_star_.data(),
              measure.dde_star_.data() + measure.dde_star_.rows() * measure.dde_star_.cols());
          msg.pm = std::vector<double>(
              measure.pm_.data(),
              measure.pm_.data() + measure.pm_.rows() * measure.pm_.cols());
          msgs.task_measures.push_back(msg);
        }
        msgs.acc_ctl_comp_time = acc_ctl_comp_time;
        monitoring_pub_->msg_ = msgs;
        monitoring_pub_->unlockAndPublish();
      }
    }
  }
}

//=====================================================================================
void HiqpController::publishControllerState() {

  rclcpp::Time now = get_node()->get_clock()->now();
  rclcpp::Duration d = now - last_c_state_update_;
  if (d.seconds() >= 1.0 / c_state_publish_rate_) {
    last_c_state_update_ = now;

    if (c_state_pub_->trylock()) {

      KDL::JntArray q = robot_state_ptr_->kdl_jnt_array_vel_.q;
      KDL::JntArray qdot = robot_state_ptr_->kdl_jnt_array_vel_.qdot;
      //KDL::JntArray effort = robot_state_ptr_->kdl_effort_;

      c_state_pub_->msg_.header.stamp = now;

      for (unsigned int i = 0; i < n_state_joints_; i++) {
        c_state_pub_->msg_.joints[i].command = u_(i);
        c_state_pub_->msg_.joints[i].position = q(i);
        c_state_pub_->msg_.joints[i].velocity = qdot(i);
        //c_state_pub_->msg_.joints[i].effort = effort(i);
      }
#if 0
      for (unsigned int i = 0; i < n_sensors_; i++) {
        c_state_pub_->msg_.sensors[i].force.clear();
        c_state_pub_->msg_.sensors[i].force.push_back(robot_state_ptr_->sensor_handle_info_[i].force_(0));
        c_state_pub_->msg_.sensors[i].force.push_back(robot_state_ptr_->sensor_handle_info_[i].force_(1));
        c_state_pub_->msg_.sensors[i].force.push_back(robot_state_ptr_->sensor_handle_info_[i].force_(2));
        c_state_pub_->msg_.sensors[i].torque.clear();
        c_state_pub_->msg_.sensors[i].torque.push_back(robot_state_ptr_->sensor_handle_info_[i].torque_(0));
        c_state_pub_->msg_.sensors[i].torque.push_back(robot_state_ptr_->sensor_handle_info_[i].torque_(1));
        c_state_pub_->msg_.sensors[i].torque.push_back(robot_state_ptr_->sensor_handle_info_[i].torque_(2));
      }
#endif
      c_state_pub_->unlockAndPublish();
    }
  }
}

void HiqpController::renderPrimitives() {
  rclcpp::Time now = get_node()->get_clock()->now();
  rclcpp::Duration d = now - last_rendering_update_;
  if (d.seconds() >= 1.0 / rendering_publish_rate_) {
    last_rendering_update_ = now;
    task_manager_ptr_->renderPrimitives();
  }
}


void HiqpController::loadRenderingParameters() {
  rendering_publish_rate_ = params_.visualization_publish_rate;  // defaults to 1 kHz
  last_rendering_update_ = get_node()->get_clock()->now();
}
#if 0

//=====================================================================================
void HiqpController::update(const ros::Time &time,
    const ros::Duration &period) {
  period_ = period;
  sampleJointValues();
  //sampleSensorValues();    
  updateControls(ddq_, u_);
  setControls();
  publishControllerState();
}
#if 0
int HiqpController::loadSensorsAndSetSensorHandlesMap() {
  if (!fts_hw_){
    n_sensors_ = 0;
    return -1;
  }

  const std::vector<std::string> &sensor_names = fts_hw_->getNames();
  n_sensors_= sensor_names.size();
  for (unsigned i = 0; i < n_sensors_; i++) {
    ROS_DEBUG("Got sensor %s", sensor_names[i].c_str());
    sensor_handles_map_.emplace(sensor_names[i],
        fts_hw_->getHandle(sensor_names[i]));

    robot_state_ptr_->sensor_handle_info_.push_back(hiqp::SensorHandleInfo(sensor_names[i], fts_hw_->getHandle(sensor_names[i]).getFrameId())); 
  }
  return 0;
}
#endif
//=====================================================================================
#if 0
void HiqpController::sampleSensorValues() {

  if (!fts_hw_)
    return;

  handles_mutex_.lock();

  for( unsigned int i=0; i<robot_state_ptr_->sensor_handle_info_.size();i++){
    hiqp::SensorHandleInfo &h = robot_state_ptr_->sensor_handle_info_[i];
    h.force_=Eigen::Map<Eigen::Vector3d>(const_cast<double*>(sensor_handles_map_.at(h.sensor_name_).getForce()));
    h.torque_=Eigen::Map<Eigen::Vector3d>(const_cast<double*>(sensor_handles_map_.at(h.sensor_name_).getTorque()));
  }

  handles_mutex_.unlock();
}
#endif

void HiqpController::initialize() {
  std::shared_ptr<ROSVisualizer> ros_visualizer = std::static_pointer_cast<ROSVisualizer>(visualizer_);
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


/// \todo Task monitoring should publish an array of all task infos at each
/// publication time step, rather than indeterministacally publishing single
/// infos on the same topic
int HiqpController::loadAndSetupTaskMonitoring() {
  XmlRpc::XmlRpcValue task_monitoring;
  if (!this->getControllerNodeHandle().getParam("task_monitoring",
        task_monitoring)) {
    ROS_ERROR_STREAM("In HiqpController: Call to getParam('"
        << "task_monitoring"
        << "') in namespace '"
        << this->getControllerNodeHandle().getNamespace()
        << "' failed.");
    return -1;
  }

  int active = static_cast<int>(task_monitoring["active"]);
  monitoring_active_ = (active == 1 ? true : false);
  monitoring_publish_rate_ =
    static_cast<double>(task_monitoring["publish_rate"]);


  return 0;
}

/// \bug Having both, joint limits and avoidance tasks at the highest hierarchy
/// level can cause an infeasible problem (e.g., via starting with
/// yumi_hiqp_preload.yaml tasks)
void HiqpController::loadJointLimitsFromParamServer() {
  XmlRpc::XmlRpcValue hiqp_preload_jnt_limits;
  if (!this->getControllerNodeHandle().getParam("hiqp_preload_jnt_limits",
        hiqp_preload_jnt_limits)) {
    ROS_WARN_STREAM("No hiqp_preload_jnt_limits parameter found on "
        << "the parameter server. No joint limits were loaded!");
  } else {
    bool parsing_success = true;
    for (int i = 0; i < hiqp_preload_jnt_limits.size(); ++i) {
      try {
        std::string link_frame =
          static_cast<std::string>(hiqp_preload_jnt_limits[i]["link_frame"]);

        XmlRpc::XmlRpcValue& limitations =
          hiqp_preload_jnt_limits[i]["limitations"];

        std::vector<std::string> def_params;
        def_params.push_back("TDefJntLimits");
        def_params.push_back(link_frame);
        def_params.push_back(
            std::to_string(static_cast<double>(limitations[0])));
        def_params.push_back(
            std::to_string(static_cast<double>(limitations[1])));
        def_params.push_back(
            std::to_string(static_cast<double>(limitations[2])));
        def_params.push_back(
            std::to_string(static_cast<double>(limitations[3])));


        std::vector<std::string> dyn_params;
        dyn_params.push_back("TDynJntLimits");
        dyn_params.push_back(
            std::to_string(static_cast<double>(limitations[4])));
        dyn_params.push_back(
            std::to_string(static_cast<double>(limitations[5])));

        if(task_manager_ptr_->setTask(link_frame + "_jntlimits", 1, true, true, false,
              def_params, dyn_params, this->getRobotState()) !=0){
          ROS_WARN_STREAM(
              "Error while loading "
              << "hiqp_preload_jnt_limits parameter from the "
              << "parameter server. Could not set task.");
          parsing_success = false;
        }

      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM(
            "Error while loading "
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

void HiqpController::loadGeometricPrimitivesFromParamServer() {
  XmlRpc::XmlRpcValue hiqp_preload_geometric_primitives;
  if (!this->getControllerNodeHandle().getParam(
        "hiqp_preload_geometric_primitives",
        hiqp_preload_geometric_primitives)) {
    ROS_WARN_STREAM("No hiqp_preload_geometric_primitives parameter "
        << "found on the parameter server. No geometric primitives "
        << "were loaded!");
  } else {
    bool parsing_success = true;
    for (int i = 0; i < hiqp_preload_geometric_primitives.size(); ++i) {
      try {
        std::string name = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["name"]);
        std::string type = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["type"]);
        std::string frame_id = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["frame_id"]);
        bool visible =
          static_cast<bool>(hiqp_preload_geometric_primitives[i]["visible"]);

        XmlRpc::XmlRpcValue& color_xml =
          hiqp_preload_geometric_primitives[i]["color"];
        XmlRpc::XmlRpcValue& parameters_xml =
          hiqp_preload_geometric_primitives[i]["parameters"];

        std::vector<double> color;
        color.push_back(static_cast<double>(color_xml[0]));
        color.push_back(static_cast<double>(color_xml[1]));
        color.push_back(static_cast<double>(color_xml[2]));
        color.push_back(static_cast<double>(color_xml[3]));

        std::vector<double> parameters;
        for (int j = 0; j < parameters_xml.size(); ++j) {
          parameters.push_back(static_cast<double>(parameters_xml[j]));
        }

        task_manager_ptr_->setPrimitive(name, type, frame_id, visible, color,
            parameters);
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM(
            "Error while loading "
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

void HiqpController::loadTasksFromParamServer() {
  XmlRpc::XmlRpcValue hiqp_preload_tasks;
  if (!this->getControllerNodeHandle().getParam("hiqp_preload_tasks",
        hiqp_preload_tasks)) {
    ROS_WARN_STREAM("No hiqp_preload_tasks parameter found on "
        << "the parameter server. No tasks were loaded!");
  } else {
    bool parsing_success = true;
    for (int i = 0; i < hiqp_preload_tasks.size(); ++i) {
      try {
        std::string name =
          static_cast<std::string>(hiqp_preload_tasks[i]["name"]);

        XmlRpc::XmlRpcValue& def_params_xml =
          hiqp_preload_tasks[i]["def_params"];
        std::vector<std::string> def_params;
        for (int j = 0; j < def_params_xml.size(); ++j) {
          def_params.push_back(static_cast<std::string>(def_params_xml[j]));
        }

        XmlRpc::XmlRpcValue& dyn_params_xml =
          hiqp_preload_tasks[i]["dyn_params"];
        std::vector<std::string> dyn_params;
        for (int j = 0; j < dyn_params_xml.size(); ++j) {
          dyn_params.push_back(static_cast<std::string>(dyn_params_xml[j]));
        }

        unsigned int priority =
          static_cast<int>(hiqp_preload_tasks[i]["priority"]);
        bool visible = static_cast<bool>(hiqp_preload_tasks[i]["visible"]);
        bool active = static_cast<bool>(hiqp_preload_tasks[i]["active"]);
        bool monitored = static_cast<bool>(hiqp_preload_tasks[i]["monitored"]);

        task_manager_ptr_->setTask(name, priority, visible, active, monitored,
            def_params, dyn_params, this->getRobotState());
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM(
            "Error while loading "
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

void HiqpController::addTfTopicSubscriptions()
{
  topic_subscriber_.init( task_manager_ptr_, this->getRobotState() );

  topic_subscriber_.addSubscription<tf::tfMessage>(
      this->getControllerNodeHandle(), "/tf", 100
      );
}

#endif

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp_ros::HiqpController,
    controller_interface::ControllerInterface)

