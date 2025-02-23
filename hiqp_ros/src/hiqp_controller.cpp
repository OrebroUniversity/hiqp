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

  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}
        
bool HiqpController::getRobotDescriptionFromServer() {
  //auto param_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), "/robot_state_publisher");
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), params_.robot_state_publisher);
  
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
  //RCLCPP_INFO(get_node()->get_logger(), "HiQP controller claiming command interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  //HIQP assumes uniform interface type
  //conf.type = controller_interface::interface_configuration_type::ALL;
  if (n_joints_ == 0)
  {
    fprintf(
      stderr,
      "During ros2_control interface configuration, degrees of freedom is not valid;"
      " it should be positive. Actual DOF is %u\n",
      n_joints_);
    std::exit(EXIT_FAILURE);
  }
  conf.names.reserve(n_joints_ * params_.command_interfaces.size());
  for (const auto & joint_name : command_joint_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
     // std::cerr<<"HIQP claiming command interface "<<joint_name<<"/"<<interface_type<<std::endl;
    }
  }
  return conf;
}

//called during configuration of state interfaces
controller_interface::InterfaceConfiguration HiqpController::state_interface_configuration() const 
{
  //RCLCPP_INFO(get_node()->get_logger(), "HiQP controller claiming state interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(n_state_joints_ * params_.state_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
      //std::cerr<<"HIQP claiming STATE interface "<<joint_name<<"/"<<interface_type<<std::endl;
    }
  }
  return conf; 
}

//configure parameters
controller_interface::CallbackReturn HiqpController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {

  (void) previous_state; //clears warning 
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
  prev_output.resize(n_state_joints_, 0.0);
  prev_derivative.resize(n_state_joints_, 0.0);

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
  is_effort_ = check_ifce_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);
  
  if(!(is_velocity_ || is_effort_)) {
    RCLCPP_ERROR(logger, "'command_interfaces' should be either velocity or acceleration for all joints.");
    return CallbackReturn::FAILURE;
  }
  //set the command interface to the correct dimension
  cmd_ifce_ = is_velocity_ ? 0 : 1;

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

  //load up parameters for effort controller
  if(is_effort_) {
    std::cerr<<"number of actuated joints is "<<n_joints_<<std::endl;

    //get impedance parameters from config file
    auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

    if(k_gains.size() != d_gains.size() || k_gains.size() != n_joints_) {
      RCLCPP_ERROR_STREAM(logger, "In HiQPController: k and d gains not correct size. Expected "
		     << n_joints_ << " Got "<<k_gains.size() <<" and "<<d_gains.size());
      return CallbackReturn::FAILURE;
    }

    Kp = Eigen::MatrixXd::Identity(n_joints_,n_joints_);// Eigen::Matrix<double,7,7>::Identity();
    Kd = Eigen::MatrixXd::Identity(n_joints_,n_joints_);//Eigen::Matrix<double, 7, 7>::Identity();

    for(int i=0; i<n_joints_; i++) {
      Kp(i,i) = k_gains.at(i);
      Kd(i,i) = d_gains.at(i);
    }
    /*
    controller_nh.param("alpha_vel", alpha_vel_, 0.99);
    controller_nh.param("delta_tau_max", delta_tau_max_, 0.1);
    alpha_vel_ = std::max(std::min(alpha_vel_, 1.0), 0.0);
    */

    std::cerr<<"Kp = "<<Kp<<std::endl;

    //setup KDL related parameters
    gravity_vector_kdl = KDL::Vector(0.0,0.0,-9.81);

    std::string chain_root, chain_tip;
    chain_root = get_node()->get_parameter("chain_root").as_string();
    chain_tip = get_node()->get_parameter("chain_tip").as_string();
    RCLCPP_DEBUG(get_node()->get_logger(), "configuring for root %s and tip %s", chain_root.c_str(), chain_tip.c_str());

    if(this->getRobotState()->kdl_tree_.getChain(chain_root, chain_tip, robot_chain)) {
      std::cerr<<"Got chain: "<<robot_chain<<std::endl;
      std::cerr<<"Chain has "<<robot_chain.getNrOfJoints()<<" joints and "
	       <<robot_chain.getNrOfSegments()<< " segments\n";
    } else {
      RCLCPP_WARN(logger, "Could not get KDL chain, quitting");
      return CallbackReturn::FAILURE;
    }
    u_vel_ = Eigen::VectorXd::Zero(n_joints_);
    q_int_ = Eigen::VectorXd::Zero(n_joints_);
    //sample initial joint values
    for (auto &&handle : joint_state_handles_map_) {
      q_int_(handle.first) = joint_state_interface_[0][handle.second].get().get_value();
    }
    //std::cerr<<"Initial joint config is "<<q_int_<<std::endl;

  }

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

  //FIXME this should come through the type of interfaces in config file!
  task_manager_ptr_->init(getNJoints(), true);//true: velocity control; false: effort control

  //loadJointLimitsFromParamServer();
  //loadGeometricPrimitivesFromParamServer();
  //loadTasksFromParamServer();

  u_vel_ = Eigen::VectorXd::Zero(getNJoints());

  dead_band_ = params_.dead_band;
  filter_alpha_ = params_.filter_alpha;

  RCLCPP_INFO(logger, "HiQP controller configured");
  return CallbackReturn::SUCCESS;

}

//clears state and will start control after this
controller_interface::CallbackReturn HiqpController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
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
        get_node()->get_logger(), "Expected %u '%s' command interfaces, got %lu.", n_joints_,
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
        get_node()->get_logger(), "Expected %u '%s' state interfaces, got %lu.", n_state_joints_,
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

  RCLCPP_INFO(logger, "HiQP controller activated");
  return CallbackReturn::SUCCESS;
}

//clean-up back to a state from which we can start the controller
controller_interface::CallbackReturn HiqpController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
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

  if(is_velocity_) {
    std::vector<double> _dq(dq.size());
    //constexpr double alpha_lpf = 0.001;
    // Time the acceleration control computation
    auto t_begin = std::chrono::high_resolution_clock::now();
    task_manager_ptr_->getVelocityControls(this->getRobotState(), _dq);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> opt_time = t_end - t_begin;

    for (size_t i = 0; i < _dq.size(); ++i) {
      //FIXME: this should only be done if filtering is enabled?
      u[i] = second_order_lpf(_dq[i], i);
    }
    renderPrimitives();
    monitorTasks(static_cast<double>(opt_time.count()));
  }

  //here the fun begins, model-based computed torque control
  if(is_effort_) {
  
    std::vector<double> _ddq(dq.size());
    Eigen::VectorXd ddq(dq.size());
    
    // Time the acceleration control computation
    auto t_begin = std::chrono::high_resolution_clock::now();
    task_manager_ptr_->getAccelerationControls(this->getRobotState(), _ddq);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> opt_time = t_end - t_begin;

    int i = 0;
    for (auto&& oc : _ddq) {
      ddq(i++) = oc;
    }

    double dt = this->getRobotState()->sampling_time_;
    Eigen::MatrixXd q_d; 	  //q desired					//<double, 7, 1>
    Eigen::MatrixXd dq_d;	  //q dot desired     //<double, 7, 1>
    Eigen::MatrixXd ddq_d = ddq.head(n_joints_); //q dot dot desired     <double, 7, 1>
    dq_d  = u_vel_ + dt*ddq_d; //computed velocity target
    u_vel_= dq_d;              //store the computed velocity controls for the next integration step
    q_d = q_int_ + dt*dq_d;    //compute joint target
    q_int_= q_d;               //store the computed desired q for the next integration step

    KDL::ChainDynParam id_solver(robot_chain,gravity_vector_kdl);
    
    //setup varriables
    KDL::JntArray q_actuated(n_joints_),
                  dq_actuated(n_joints_),
                  ddq_desired(n_joints_), 
                  torques(n_joints_);

    q_actuated.data = this->getRobotState()->kdl_jnt_array_vel_.q.data.head(n_joints_);
    dq_actuated.data = this->getRobotState()->kdl_jnt_array_vel_.qdot.data.head(n_joints_);

    //ddq_desired.data = ddq.head(n_actuated_joints_);
    
    KDL::JntArray coriolis_kdl(n_joints_), 
	          gravity_kdl(n_joints_);
    KDL::JntSpaceInertiaMatrix mass_kdl(n_joints_);

    //get gravity torque
    int error_number = id_solver.JntToGravity(q_actuated, gravity_kdl);
    //std::cerr<<"Gravity errno "<<error_number<<" value: "<<gravity_kdl.data.transpose()<<std::endl;

    error_number = id_solver.JntToCoriolis(q_actuated, dq_actuated, coriolis_kdl);
    //std::cerr<<"Coriolis errno "<<error_number<<" value: "<<coriolis_kdl.data.transpose()<<std::endl;

    error_number = id_solver.JntToMass(q_actuated, mass_kdl);
    //std::cerr<<"Mass errno "<<error_number<<" value:\n" <<mass_kdl.data<<std::endl; 

    Eigen::MatrixXd tau (n_joints_, 1);
    Eigen::MatrixXd tau_ (n_joints_, 1);
  
    //NOTE: below old formula used previously
    //computed torque control: forward model + impedance term
    //tau_ = mass_kdl.data*ddq_d + coriolis_kdl.data + 
	    //Kd*(dq_d-dq_actuated.data) + Kp*position_error;
      //for (size_t i = 0; i < tau_.size(); ++i) {
      //  tau(i,0) = second_order_lpf(tau_(i,0), i);
    //}

    // Check if the errors of all joints are within the dead zone range
    int kz = 1;
    bool all_within_deadzone = true;  // check if all axes are within the deadband
    for (int i = 0; i < q_d.size(); i++) {
      if (std::abs(q_d(i,0)) >= dead_band_) {
          all_within_deadzone = false;
          break;//As long as one axis error exceeds the dead band, the check will be exited
      }
    }

    // If all errors are within the deadband, set kz to zero.
    if (all_within_deadzone) {
      kz=0;
    }
    
    tau =  mass_kdl.data*ddq_d + coriolis_kdl.data + kz*3*Kp*(q_d-q_actuated.data) + kz*6*Kd*(dq_d-dq) ;
    //saturate torque
    tau_ << saturateTorqueRate(tau, tau_);

    u.head(n_joints_) = tau;

 	     /*  
    std::cerr<<"Setting model-based commands: "
             <<"\n q   = "<<q_actuated.data.transpose()
             <<"\n q_d   = "<<q_d.transpose()
             <<"\n dq  = "<<dq_actuated.data.transpose()
             <<"\n dq_d  = "<<dq_d.transpose()
             <<"\n ddq_d = "<<ddq_d.transpose()
             <<"\n imp_t = "<<(Kp*(dq_d-dq_actuated.data) + Kd*(q_d-q_actuated.data)).transpose()
             <<"\n tau = "<<tau.transpose()<<std::endl;
 */
    renderPrimitives();
    monitorTasks(static_cast<double>(opt_time.count()));
  }
  return;
}
// **Quadratic low-pass filter function**
double HiqpController::second_order_lpf(double input, size_t index) {
  // Calculate filter parameters
  double alpha_0 = omega_n * omega_n;
  double alpha_1 = 2 * zeta * omega_n;
  double alpha_2 = omega_n * omega_n;
  //double dt = this->getRobotState()->sampling_time_;

  //Compute first-order differences (discretized derivatives)
  double derivative = (input - prev_output[index]) / dt;
  
  //Calculate the filtered output
  double output = prev_output[index] + dt * prev_derivative[index];

  // Update stored value
  prev_derivative[index] = alpha_0 * input + alpha_1 * derivative - alpha_2 * output;
  prev_output[index] = output;

  return output;
}
//=====================================================================================

Eigen::VectorXd  HiqpController::saturateTorqueRate(
  const Eigen::VectorXd& tau_d_calculated,
  const Eigen::VectorXd& tau_J_d) {
  assert(tau_d_calculated.size() == tau_J_d.size() && "Size mismatch in torque saturation!");
  Eigen::VectorXd tau_d_saturated(tau_d_calculated.size());
  for (size_t i = 0; i < n_joints_; i++) {
     double difference = tau_d_calculated[i] - tau_J_d[i];
     tau_d_saturated[i] =
         tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
   }
   return tau_d_saturated;
}

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
  KDL::JntArray qdot_current = robot_state_ptr_->kdl_jnt_array_vel_.qdot;
  //KDL::JntArray &effort = robot_state_ptr_->kdl_effort_;
  q.data.setZero();
  qdot_current.data.setZero();
  //qdot.data.setZero();
  //effort.data.setZero();

  //handles_mutex_.lock();
  //handles_mutex_.unlock();
  
  for (auto &&handle : joint_state_handles_map_) {
    q(handle.first) = joint_state_interface_[0][handle.second].get().get_value();
    qdot_current(handle.first) = joint_state_interface_[1][handle.second].get().get_value();
  }
  qdot.data = filter_alpha_*qdot_current.data + (1-filter_alpha_)*qdot.data;
  ddq_ = Eigen::Map<Eigen::VectorXd>(qdot.data.data(), qdot.rows());
}
//=====================================================================================
void HiqpController::setControls() {
  //RCLCPP_WARN_STREAM(get_node()->get_logger(),"u = ["<<u_.transpose()<<"]");
  //handles_mutex_.lock();
  //std::cerr<<"commands "<<joint_command_interface_.size();
  //std::cerr<<" for njoints "<<joint_command_interface_[cmd_ifce_].size()<<std::endl;
  for (auto &&handle : joint_handles_map_) {
    //if(fabs(u_(handle.first))<dead_band_) u_(handle.first)=0.0; 
    joint_command_interface_[cmd_ifce_][handle.second].get().set_value(u_(handle.first));
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

///TODO NOTE For future developers: This code was deprecated, as it makes the specification of joint limits obligatory.
///It seems it is difficult to specify an optional array of complex stuff in generate_parameter
///So, be warned, this should not be added back!

/// \bug Having both, joint limits and avoidance tasks at the highest hierarchy
/// level can cause an infeasible problem TODO:replicate

void HiqpController::loadJointLimitsFromParamServer() {
  std::cerr<<"\n\n\n\n\nLOADING JOINT LIMITS\n\n\n\n";
  if (params_.hiqp_preload_jnt_limits.command_joints_map.size() == 0 ) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not find hiqp_preload_jnt_limits in parameters");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Processing joint limits");
    for(auto lmt=params_.hiqp_preload_jnt_limits.command_joints_map.begin(); 
             lmt!=params_.hiqp_preload_jnt_limits.command_joints_map.end(); lmt++) {
      std::vector<std::string> def_params;
      def_params.push_back("TDefJntLimits");
      std::string link_frame = lmt->second.link_frame; //look up link frame from joint name
      def_params.push_back(link_frame);
      def_params.push_back(std::to_string(static_cast<double>(lmt->second.dq_max)));
      def_params.push_back(std::to_string(lmt->second.ddq_max));
      std::cerr<<"Task "<<link_frame<<"_jntlimits ";
      std::cerr<<"def_params "<<def_params[0]<<" "<<def_params[1]<<std::endl;
/*          
      def_params.push_back(
          std::to_string(static_cast<double>(lmt->second.limitations[2])));
      def_params.push_back(
          std::to_string(static_cast<double>(lmt->second.limitations[3])));


      std::vector<std::string> dyn_params;
      dyn_params.push_back("TDynJntLimits");
      dyn_params.push_back(
          std::to_string(static_cast<double>(lmt->second.limitations[4])));
      dyn_params.push_back(
          std::to_string(static_cast<double>(lmt->second.limitations[5])));
      std::cerr<<"Task "<<link_frame<<"_jntlimits ";
      std::cerr<<"def_params "<<def_params[0]<<" "<<def_params[1]<<" "<<def_params[2]<<" "<<def_params[3]<<std::endl;
      if(task_manager_ptr_->setTask(link_frame + "_jntlimits", 0, true, true, false,
            def_params, dyn_params, this->getRobotState()) !=0){
        RCLCPP_WARN_STREAM(get_node()->get_logger(), 
            "Error while loading "
            << "hiqp_preload_jnt_limits parameter from the "
            << "parameter server. Could not set task.");
      }
*/
    }
  }

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

