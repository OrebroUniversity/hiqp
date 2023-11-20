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

//#include <XmlRpcException.h>
//#include <XmlRpcValue.h>

#include <hiqp_ros/hiqp_controller.h>
#include <hiqp_ros/utilities.h>

#include <hiqp_msgs/msg/string_array.hpp>
#include <hiqp_msgs/msg/task_measures.hpp>
#include <hiqp_msgs/msg/vector3d.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

using hiqp::TaskMeasure;
using namespace hiqp_ros;

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
  return CallbackReturn::SUCCESS;
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
  conf.names.reserve(n_joints_ * params_.state_interfaces.size());
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

  // get degrees of freedom
  n_joints_ = params_.joints.size();

  if (params_.joints.empty())
  {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty())
  {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }
  else if (command_joint_names_.size() != params_.joints.size())
  {
    RCLCPP_ERROR(
      logger, "'command_joints' parameter has to have the same size as 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }

  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());

#if 0
  if (!reset())
  {
    return CallbackReturn::FAILURE;
  }

  has_position_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);

  // if there is only velocity or if there is effort command interface
  // then use also PID adapter
  use_closed_loop_pid_adapter_ =
    (has_velocity_command_interface_ && params_.command_interfaces.size() == 1 &&
     !params_.open_loop_control) ||
    has_effort_command_interface_;

  if (use_closed_loop_pid_adapter_)
  {
    pids_.resize(dof_);
    ff_velocity_scale_.resize(dof_);
    tmp_command_.resize(dof_, 0.0);

    update_pids();
  }

  // Configure joint position error normalization from ROS parameters (angle_wraparound)
  joints_angle_wraparound_.resize(dof_);
  for (size_t i = 0; i < dof_; ++i)
  {
    const auto & gains = params_.gains.joints_map.at(params_.joints[i]);
    joints_angle_wraparound_[i] = gains.angle_wraparound;
  }

  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  // Note: 'effort' storage is also here, but never used. Still, for this is OK.
  joint_state_interface_.resize(allowed_interface_types_.size());

  has_position_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  // Validation of combinations of state and velocity together have to be done
  // here because the parameter validators only deal with each parameter
  // separately.
  if (
    has_velocity_command_interface_ && params_.command_interfaces.size() == 1 &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      logger,
      "'velocity' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
    return CallbackReturn::FAILURE;
  }

  // effort is always used alone so no need for size check
  if (
    has_effort_command_interface_ &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      logger,
      "'effort' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
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

  // parse remaining parameters
  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ = interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    logger, "Using '%s' interpolation method.",
    interpolation_methods::InterpolationMethodMap.at(interpolation_method_).c_str());

  // prepare hold_position_msg
  init_hold_position_msg();

  // create subscriber and publishers
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTrajectoryController::topic_callback, this, std::placeholders::_1));

  publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.joint_names = params_.joints;
  state_publisher_->msg_.reference.positions.resize(dof_);
  state_publisher_->msg_.reference.velocities.resize(dof_);
  state_publisher_->msg_.reference.accelerations.resize(dof_);
  state_publisher_->msg_.feedback.positions.resize(dof_);
  state_publisher_->msg_.error.positions.resize(dof_);
  if (has_velocity_state_interface_)
  {
    state_publisher_->msg_.feedback.velocities.resize(dof_);
    state_publisher_->msg_.error.velocities.resize(dof_);
  }
  if (has_acceleration_state_interface_)
  {
    state_publisher_->msg_.feedback.accelerations.resize(dof_);
    state_publisher_->msg_.error.accelerations.resize(dof_);
  }
  if (has_position_command_interface_)
  {
    state_publisher_->msg_.output.positions.resize(dof_);
  }
  if (has_velocity_command_interface_)
  {
    state_publisher_->msg_.output.velocities.resize(dof_);
  }
  if (has_acceleration_command_interface_)
  {
    state_publisher_->msg_.output.accelerations.resize(dof_);
  }
  if (has_effort_command_interface_)
  {
    state_publisher_->msg_.output.effort.resize(dof_);
  }

  state_publisher_->unlock();

  // action server configuration
  if (params_.allow_partial_joints_goal)
  {
    RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
  }

  RCLCPP_INFO(
    logger, "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&JointTrajectoryController::goal_received_callback, this, _1, _2),
    std::bind(&JointTrajectoryController::goal_cancelled_callback, this, _1),
    std::bind(&JointTrajectoryController::goal_accepted_callback, this, _1));

  resize_joint_trajectory_point(state_current_, dof_);
  resize_joint_trajectory_point_command(command_current_, dof_);
  resize_joint_trajectory_point(state_desired_, dof_);
  resize_joint_trajectory_point(state_error_, dof_);
  resize_joint_trajectory_point(last_commanded_state_, dof_);

  query_state_srv_ = get_node()->create_service<control_msgs::srv::QueryTrajectoryState>(
    std::string(get_node()->get_name()) + "/query_state",
    std::bind(&JointTrajectoryController::query_state_service, this, _1, _2));
#endif
  return CallbackReturn::SUCCESS;

}

//clears state and will start control after this
controller_interface::CallbackReturn HiqpController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {

  RCLCPP_INFO(get_node()->get_logger(), "HiQP controller activating");

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  //read current state

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

  return controller_interface::return_type::OK;
} 

void HiqpController::updateControls(Eigen::VectorXd& dq, Eigen::VectorXd& u) {
  if (!is_active_) return;

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

  //renderPrimitives();
  //monitorTasks(static_cast<double>(opt_time.count()));

  return;
}

#if 0
  robot_state_ptr_ = RobotStatePtr(new RobotState()); //.reset(&robot_state_data_);

  ROS_INFO("Set up all handles");

  ros::Time t = ros::Time::now();
  last_sampling_time_point_.setTimePoint(t.sec, t.nsec);

  loadUrdfToKdlTree();
  ROS_INFO("Loaded KDL tree");

  sampleJointValues();
  //ROS_INFO("Sampled joint values");
  //sampleSensorValues();  
  //ROS_INFO("Calling initialize to derived instance");
  initialize();

  c_state_pub_.init(controller_nh, "joint_controller_state", 1);
  last_c_state_update_ = ros::Time::now();
  controller_nh.param("c_state_publish_rate", c_state_publish_rate_, 100.0);
  c_state_pub_.msg_.joints.resize(n_joints_);
  c_state_pub_.msg_.sensors.resize(n_sensors_);

  for (auto &&it : robot_state_ptr_->kdl_tree_.getSegments()) {
    c_state_pub_.msg_.joints.at(it.second.q_nr).name = it.second.segment.getJoint().getName();
  }


  for (unsigned int i=0; i<n_sensors_;i++){
    c_state_pub_.msg_.sensors.at(i).name=robot_state_ptr_->sensor_handle_info_[i].sensor_name_;
    c_state_pub_.msg_.sensors.at(i).frame_id=robot_state_ptr_->sensor_handle_info_[i].frame_id_;    
  }

  return CallbackReturn::SUCCESS;
}

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
//=====================================================================================
int HiqpController::loadUrdfToKdlTree() {
  std::string full_parameter_path;
  std::string robot_urdf;
  if (controller_nh_.searchParam("robot_description", full_parameter_path)) {
    controller_nh_.getParam(full_parameter_path, robot_urdf);
    bool success =
      kdl_parser::treeFromString(robot_urdf, robot_state_ptr_->kdl_tree_);
    ROS_ASSERT(success);
    ROS_INFO("Loaded the robot's urdf model and initialized the KDL tree "
        "successfully");
  } else {
    ROS_ERROR("Could not find parameter 'robot_description' on the parameter "
        "server.");
    return -1;
  }
  return 0;
}

//=====================================================================================
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
int HiqpController::loadJointsAndSetJointHandlesMap() {
  ROS_INFO("Setting up joint handle map...");
  std::string param_name = "joints";
  std::vector<std::string> joint_names;
  if (!controller_nh_.getParam(param_name, joint_names)) {
    ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('"
        << param_name << "') in namespace '"
        << controller_nh_.getNamespace() << "' failed.");
    return -1;
  }

  unsigned int n_joint_names = joint_names.size();
  n_joints_ = robot_state_ptr_->kdl_tree_.getNrOfJoints();

  //ROS_INFO_STREAM("We have " << n_joint_names << " joints in the controller parameter and " << n_joints_ << " joints in the robot model");
  if (n_joint_names > n_joints_) {
    ROS_ERROR_STREAM(
        "In ROSKinematicsController: The .yaml file"
        << " includes more joint names (" << n_joint_names
        << ") than specified in the .urdf file (" << n_joints_
        << ") Could not successfully initialize controller. Aborting!\n");
    return -3;
  }

  std::vector<unsigned int> qnrs;

  qnrs.clear();
  KDL::SegmentMap all_segments = robot_state_ptr_->kdl_tree_.getSegments();
  std::cerr<<"KDL tree has "<<all_segments.size()<<" elements\n";
  for (KDL::SegmentMap::const_iterator element=all_segments.cbegin(); 
      element!=all_segments.cend(); element++ ) {
    qnrs.push_back(element->second.q_nr);
    std::cerr<<element->first<<" added joint "<<element->second.q_nr
      <<" for segment "<<element->second.segment.getName()<<std::endl;
  }

  //FIXME: this breaks, code duplication above instead.
  //hiqp::kdl_getAllQNrFromTree(robot_state_data_.kdl_tree_, qnrs);

  robot_state_ptr_->joint_handle_info_.clear();

  for (auto &&name : joint_names) {
    try {
      unsigned int q_nr =
        hiqp::kdl_getQNrFromJointName(robot_state_ptr_->kdl_tree_, name);
      ROS_INFO_STREAM("Joint found: '" << name << "', qnr: " << q_nr);
      joint_handles_map_.emplace(q_nr, jnt_hw_->getHandle(name));
      qnrs.erase(std::remove(qnrs.begin(), qnrs.end(), q_nr), qnrs.end());
      robot_state_ptr_->joint_handle_info_.push_back(
          hiqp::JointHandleInfo(q_nr, name, true, true));
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return -2;
    }
    // catch (MAP INSERT FAIL EXCEPTION)
    // catch (HIQP Q_NR NOT AVAILABLE EXCEPTION)
  }

  for (auto &&qnr : qnrs) {
    std::string joint_name =
      hiqp::kdl_getJointNameFromQNr(robot_state_ptr_->kdl_tree_, qnr);
    hiqp::JointHandleInfo jhi(qnr, joint_name, true, false);
    robot_state_ptr_->joint_handle_info_.push_back(jhi);
  }

  std::cout << "Joint handle info:\n";
  for (auto &&jhi : robot_state_ptr_->joint_handle_info_) {
    std::cout << jhi.q_nr_ << ", " << jhi.joint_name_ << ", " << jhi.readable_
      << ", " << jhi.writable_ << "\n";
  }

  robot_state_ptr_->kdl_jnt_array_vel_.resize(n_joints_);
  KDL::SetToZero(robot_state_ptr_->kdl_jnt_array_vel_.q);
  KDL::SetToZero(robot_state_ptr_->kdl_jnt_array_vel_.qdot);
  robot_state_ptr_->kdl_effort_.resize(n_joints_);
  KDL::SetToZero(robot_state_ptr_->kdl_effort_);
  ddq_ = Eigen::VectorXd::Zero(n_joints_);
  u_ = Eigen::VectorXd::Zero(n_joints_);
  return 0;
}
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
//=====================================================================================
void HiqpController::sampleJointValues() {
  robot_state_ptr_->sampling_time_ = period_.toSec();

  KDL::JntArray &q = robot_state_ptr_->kdl_jnt_array_vel_.q;
  KDL::JntArray &qdot = robot_state_ptr_->kdl_jnt_array_vel_.qdot;
  KDL::JntArray &effort = robot_state_ptr_->kdl_effort_;
  q.data.setZero();
  qdot.data.setZero();
  effort.data.setZero();
  //double alpha = 0.05;

  handles_mutex_.lock();
  for (auto &&handle : joint_handles_map_) {
    q(handle.first) = handle.second.getPosition();
    qdot(handle.first) = handle.second.getVelocity(); 
    //qdot(handle.first) = (1-alpha)*qdot(handle.first) +alpha*handle.second.getVelocity(); //FIXME!! low-pass on velocity
    effort(handle.first) = handle.second.getEffort();
  }

  handles_mutex_.unlock();
}
//=====================================================================================
void HiqpController::setControls() {
  handles_mutex_.lock();
  for (auto &&handle : joint_handles_map_) {
    handle.second.setCommand(u_(handle.first));
  }
  handles_mutex_.unlock();
}
//=====================================================================================
void HiqpController::publishControllerState() {

  ros::Time now = ros::Time::now();
  ros::Duration d = now - last_c_state_update_;
  if (d.toSec() >= 1.0 / c_state_publish_rate_) {
    last_c_state_update_ = now;

    if (c_state_pub_.trylock()) {

      KDL::JntArray q = robot_state_ptr_->kdl_jnt_array_vel_.q;
      KDL::JntArray qdot = robot_state_ptr_->kdl_jnt_array_vel_.qdot;
      KDL::JntArray effort = robot_state_ptr_->kdl_effort_;

      c_state_pub_.msg_.header.stamp = ros::Time::now();

      for (unsigned int i = 0; i < n_joints_; i++) {
        c_state_pub_.msg_.joints[i].command = u_(i);
        c_state_pub_.msg_.joints[i].position = q(i);
        c_state_pub_.msg_.joints[i].velocity = qdot(i);
        c_state_pub_.msg_.joints[i].effort = effort(i);
      }
      for (unsigned int i = 0; i < n_sensors_; i++) {
        c_state_pub_.msg_.sensors[i].force.clear();
        c_state_pub_.msg_.sensors[i].force.push_back(robot_state_ptr_->sensor_handle_info_[i].force_(0));
        c_state_pub_.msg_.sensors[i].force.push_back(robot_state_ptr_->sensor_handle_info_[i].force_(1));
        c_state_pub_.msg_.sensors[i].force.push_back(robot_state_ptr_->sensor_handle_info_[i].force_(2));
        c_state_pub_.msg_.sensors[i].torque.clear();
        c_state_pub_.msg_.sensors[i].torque.push_back(robot_state_ptr_->sensor_handle_info_[i].torque_(0));
        c_state_pub_.msg_.sensors[i].torque.push_back(robot_state_ptr_->sensor_handle_info_[i].torque_(1));
        c_state_pub_.msg_.sensors[i].torque.push_back(robot_state_ptr_->sensor_handle_info_[i].torque_(2));
      }

      c_state_pub_.unlockAndPublish();
    }
  }
}

HiqpController::HiqpController()
  : is_active_(true),
  monitoring_active_(false),
  visualizer_(new ROSVisualizer()),
  task_manager_ptr_(new hiqp::TaskManager(visualizer_)) {}

HiqpController::~HiqpController() noexcept {}

void HiqpController::initialize() {
  std::shared_ptr<ROSVisualizer> ros_visualizer = std::static_pointer_cast<ROSVisualizer>(visualizer_);
  ros_visualizer->init(this->controller_nh_);

  service_handler_.init(this->getControllerNodeHandlePtr(), task_manager_ptr_, this->getRobotState());

  loadRenderingParameters();

  if (loadAndSetupTaskMonitoring() != 0) return;

  bool tf_primitives = false;
  if (!this->getControllerNodeHandle().getParam("load_primitives_from_tf",
        tf_primitives)) {
    ROS_WARN(
        "Couldn't find parameter 'load_primitives_from_tf' on parameter "
        "server, defaulting to no tf primitive tracking.");
  }
  if(tf_primitives) {
    addTfTopicSubscriptions();
  }

  service_handler_.advertiseAll();

  task_manager_ptr_->init(getNJoints(), true);

  loadJointLimitsFromParamServer();

  loadGeometricPrimitivesFromParamServer();

  loadTasksFromParamServer();

  u_vel_ = Eigen::VectorXd::Zero(getNJoints());
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

void HiqpController::renderPrimitives() {
  ros::Time now = ros::Time::now();
  ros::Duration d = now - last_rendering_update_;
  if (d.toSec() >= 1.0 / rendering_publish_rate_) {
    last_rendering_update_ = now;
    task_manager_ptr_->renderPrimitives();
  }
}

void HiqpController::monitorTasks(double acc_ctl_comp_time) {
  if (monitoring_active_) {
    ros::Time now = ros::Time::now();
    ros::Duration d = now - last_monitoring_update_;
    if (d.toSec() >= 1.0 / monitoring_publish_rate_) {
      last_monitoring_update_ = now;
      std::vector<TaskMeasure> measures;
      task_manager_ptr_->getTaskMeasures(measures);

      hiqp_msgs::TaskMeasures msgs;
      msgs.stamp = now;
      for (auto&& measure : measures) {
        hiqp_msgs::TaskMeasure msg;
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
      if (!msgs.task_measures.empty()) monitoring_pub_.publish(msgs);
    }
  }
}

void HiqpController::loadRenderingParameters() {
  rendering_publish_rate_ = 1000;  // defaults to 1 kHz
  if (!this->getControllerNodeHandle().getParam("visualization_publish_rate",
        rendering_publish_rate_)) {
    ROS_WARN(
        "Couldn't find parameter 'visualization_publish_rate' on parameter "
        "server, defaulting to 1 kHz.");
  }
  last_rendering_update_ = ros::Time::now();
}

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

  monitoring_pub_ =
    this->getControllerNodeHandle().advertise<hiqp_msgs::TaskMeasures>(
        "task_measures", 1);

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

