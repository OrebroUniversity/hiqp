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

#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

#include <unistd.h>  // usleep()
#include <iostream>
#include <string>

#include <XmlRpcException.h>
#include <XmlRpcValue.h>

#include <hiqp_ros/hiqp_joint_effort_controller.h>
#include <hiqp_ros/utilities.h>

#include <hiqp_msgs/StringArray.h>
#include <hiqp_msgs/TaskMeasures.h>
#include <hiqp_msgs/Vector3d.h>

#include <geometry_msgs/PoseStamped.h>  // teleoperation magnet sensors
#include <tf/tfMessage.h>

using hiqp::TaskMeasure;

namespace hiqp_ros {

////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
////////////////////////////////////////////////////////////////////////////////
//
//              R O S   C O N T R O L L E R   I N T E R F A C E
//
////////////////////////////////////////////////////////////////////////////////

HiQPJointEffortController::HiQPJointEffortController()
    : is_active_(true),
      monitoring_active_(false),
      be_model_based_(false),
      visualizer_(&ros_visualizer_),
      task_manager_(visualizer_),
      task_manager_ptr_(&task_manager_) {}

HiQPJointEffortController::~HiQPJointEffortController() noexcept {}

void HiQPJointEffortController::initialize() {
  ROS_INFO("HiQPJointEffortController initializing");
  
  service_handler_.init(this->getControllerNodeHandlePtr(), task_manager_ptr_,
                        this->getRobotState());
  //ros_visualizer_.init(this->getControllerNodeHandle());
  ros_visualizer_.init(this->controller_nh_);

  loadRenderingParameters();
  if (loadAndSetupTaskMonitoring() != 0) return;

  bool tf_primitives = false;
  if (!this->getControllerNodeHandle().getParam("load_primitives_from_tf",
                                                tf_primitives)) {
    ROS_WARN(
        "Couldn't find parameter 'load_primitives_from_tf' on parameter "
        "server, defaulting to no tf primitive tracking.");
  }
  
  if (!this->getControllerNodeHandle().getParam("model_based",
                                                be_model_based_)) {
    ROS_WARN(
        "Couldn't find parameter 'model_based' on parameter "
        "server, defaulting to not using the inverse dynamics model.");
    be_model_based_=false;
  }

  service_handler_.advertiseAll();

  task_manager_.init(getNJoints());
  loadJointLimitsFromParamServer();
  loadGeometricPrimitivesFromParamServer();
  loadTasksFromParamServer();

  //initialize dynamics solver
  if(be_model_based_) {
    //get the number of actuated joints right
    n_actuated_joints_=0;
    for (int i=0; i<getNJoints(); i++) {
      if(this->getRobotState()->isQNrWritable(i)) n_actuated_joints_++;
    }
    std::cerr<<"number of actuated joints is "<<n_actuated_joints_<<std::endl;

    //get impedance parameters from config file
    std::string param_name = "kv";
    std::vector<double> kv_gains, kd_gains;
    if (!this->getControllerNodeHandle().getParam(param_name, kv_gains)) {
      ROS_ERROR_STREAM("In HiQPJointEffortController: Call to getParam('"
                     << param_name << "') in namespace '"
                     << this->getControllerNodeHandle().getNamespace() << "' failed.");
      return;
    }
    param_name = "kd";
    if (!this->getControllerNodeHandle().getParam(param_name, kd_gains)) {
      ROS_ERROR_STREAM("In HiQPJointEffortController: Call to getParam('"
                     << param_name << "') in namespace '"
                     << this->getControllerNodeHandle().getNamespace() << "' failed.");
      return;
    }
    if(kv_gains.size() != kd_gains.size() || kv_gains.size() != n_actuated_joints_) {
      ROS_ERROR_STREAM("In HiQPJointEffortController: kv and kd gains are not the correct size. Expected "
		     << n_actuated_joints_ << " Got "<<kv_gains.size() <<" and "<<kd_gains.size());
      return;
    }

    Kv = Eigen::MatrixXd::Identity(n_actuated_joints_,n_actuated_joints_);// Eigen::Matrix<double,7,7>::Identity();
    Kd = Eigen::MatrixXd::Identity(n_actuated_joints_,n_actuated_joints_);//Eigen::Matrix<double, 7, 7>::Identity();

    for(int i=0; i<n_actuated_joints_; i++) {
      Kv(i,i) = kv_gains[i];
      Kd(i,i) = kd_gains[i];
    }
    /*
    controller_nh.param("alpha_vel", alpha_vel_, 0.99);
    controller_nh.param("delta_tau_max", delta_tau_max_, 0.1);
    alpha_vel_ = std::max(std::min(alpha_vel_, 1.0), 0.0);
    */

    std::cerr<<"Kv = "<<Kv<<std::endl;

    //setup KDL related parameters
    gravity_vector_kdl = KDL::Vector(0.0,0.0,-9.81);

    std::string chain_root, chain_tip;
    if (!this->getControllerNodeHandle().getParam("chain_root",
                      chain_root)) {
      ROS_WARN(
              "Couldn't find parameter 'chain_root' on parameter "
              "server, defaulting to not using the inverse dynamics model.");
      be_model_based_=false;
    }
    if (!this->getControllerNodeHandle().getParam("chain_tip",
                      chain_tip)) {
      ROS_WARN(
              "Couldn't find parameter 'chain_tip' on parameter "
              "server, defaulting to not using the inverse dynamics model.");
      be_model_based_=false;
    }

    if(this->getRobotState()->kdl_tree_.getChain(chain_root, chain_tip, robot_chain) && be_model_based_) {
      std::cerr<<"Got chain: "<<robot_chain<<std::endl;
      std::cerr<<"Chain has "<<robot_chain.getNrOfJoints()<<" joints and "
	       <<robot_chain.getNrOfSegments()<< " segments\n";
    } else {
      ROS_WARN("Could not get chain, defaulting to no use of inverse dynamics model");
      be_model_based_=false;
    }
    u_vel_ = Eigen::VectorXd::Zero(n_actuated_joints_);
    q_int_ = Eigen::VectorXd::Zero(n_actuated_joints_);

  }
}

void HiQPJointEffortController::updateControls(Eigen::VectorXd& ddq, Eigen::VectorXd& u) {

  if (!is_active_) return;

  std::vector<double> _ddq(ddq.size());
  
  // Time the acceleration control computation
  auto t_begin = std::chrono::high_resolution_clock::now();
  task_manager_.getAccelerationControls(this->getRobotState(), _ddq);
  auto t_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> opt_time = t_end - t_begin;

  int i = 0;
  for (auto&& oc : _ddq) {
    ddq(i++) = oc;
  }

  //std::cerr<<"ddq = "<<ddq.transpose()<<std::endl;
  if(!be_model_based_) {
    u = ddq;
  } else {

    double dt = period_.toSec();
    Eigen::MatrixXd q_d; 	  //q desired					//<double, 7, 1>
    Eigen::MatrixXd dq_d;	  //q dot desired                                 <double, 7, 1>
    Eigen::MatrixXd ddq_d = ddq.head(n_actuated_joints_); //q dot dot desired     <double, 7, 1>
    //double alpha = 0.9;
    dq_d  = u_vel_ + dt*ddq_d; //computed velocity target
    u_vel_= dq_d; //store the computed velocity controls for the next integration step
    q_d = q_int_ + dt*dq_d; //compute joint target
    q_int_= q_d;   //store the computed desired q for the next integration step

    KDL::ChainDynParam id_solver(robot_chain,gravity_vector_kdl);
    
    //setup varriables
    KDL::JntArray q_actuated(n_actuated_joints_),
                  dq_actuated(n_actuated_joints_),
                  ddq_desired(n_actuated_joints_), 
                  torques(n_actuated_joints_);

    q_actuated.data = this->getRobotState()->kdl_jnt_array_vel_.q.data.head(n_actuated_joints_);
    dq_actuated.data = this->getRobotState()->kdl_jnt_array_vel_.qdot.data.head(n_actuated_joints_);

    //ddq_desired.data = ddq.head(n_actuated_joints_);

    
    KDL::JntArray coriolis_kdl(n_actuated_joints_), 
	          gravity_kdl(n_actuated_joints_);
    KDL::JntSpaceInertiaMatrix mass_kdl(n_actuated_joints_);

    //get gravity torque
    int error_number = id_solver.JntToGravity(q_actuated, gravity_kdl);
    //std::cerr<<"Gravity errno "<<error_number<<" value: "<<gravity_kdl.data.transpose()<<std::endl;

    error_number = id_solver.JntToCoriolis(q_actuated, dq_actuated, coriolis_kdl);
    //std::cerr<<"Coriolis errno "<<error_number<<" value: "<<coriolis_kdl.data.transpose()<<std::endl;

    error_number = id_solver.JntToMass(q_actuated, mass_kdl);
    //std::cerr<<"Mass errno "<<error_number<<" value:\n" <<mass_kdl.data<<std::endl; 

    Eigen::MatrixXd tau (n_actuated_joints_, 1);
    //computed torque control: forward model + impedance term
    tau = mass_kdl.data*ddq_d + coriolis_kdl.data + gravity_kdl.data + 
	    Kv*(dq_d-dq_actuated.data) + Kd*(q_d-q_actuated.data);
    //tau = gravity_kdl.data;

    //TODO: here saturate torques?

    u.head(n_actuated_joints_) = tau;

    /*
    std::cerr<<"Setting model-based commands: "
             //<<"\n q   = "<<q_actuated.data.transpose()
             //<<"\n dq  = "<<dq_actuated.data.transpose()
             <<"\n ddq_d = "<<ddq_d.transpose()
	     <<"\n imp_t = "<<(Kv*(dq_d-dq_actuated.data) + Kd*(q_d-q_actuated.data)).transpose()
             <<"\n tau = "<<tau.transpose()<<std::endl;
	     */
  }

  renderPrimitives();

  monitorTasks(static_cast<double>(opt_time.count()));

  return;
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
#if 0
Eigen::MatrixXd HiQPJointEffortController::saturateTorqueRate(  //<double, 7, 1>
    const Eigen::MatrixXd& tau_d_calculated,        //<double, 7, 1>
    const Eigen::MatrixXd& tau_J_d) {               //<double, 7, 1>

  Eigen::MatrixXd tau_d_saturated = Eigen::MatrixXd::Zero(n_actuated_joints_,1);
  for (size_t i = 0; i < n_actuated_joints_; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}
#endif

void HiQPJointEffortController::renderPrimitives() {
  ros::Time now = ros::Time::now();
  ros::Duration d = now - last_rendering_update_;
  if (d.toSec() >= 1.0 / rendering_publish_rate_) {
    last_rendering_update_ = now;
    task_manager_.renderPrimitives();
  }
}

void HiQPJointEffortController::monitorTasks(double acc_ctl_comp_time) {
  if (monitoring_active_) {
    ros::Time now = ros::Time::now();
    ros::Duration d = now - last_monitoring_update_;
    if (d.toSec() >= 1.0 / monitoring_publish_rate_) {
      last_monitoring_update_ = now;
      std::vector<TaskMeasure> measures;
      task_manager_.getTaskMeasures(measures);

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


// void HiQPJointEffortController::addAllTopicSubscriptions()
// {
//   topic_subscriber_.init( &task_manager_ );

//   topic_subscriber_.addSubscription<geometry_msgs::PoseStamped>(
//     this->getControllerNodeHandle(), "/wintracker_rebase/pose", 100
//   );

// topic_subscriber_.addSubscription<hiqp_msgs::Vector3d>(
//  controller_nh_, "/yumi/hiqp_controllers/vector3d", 100
//);

// topic_subscriber_.addSubscription<hiqp_msgs::StringArray>(
//  controller_nh_, "/yumi/hiqp_kinematics_controller/experiment_commands", 100
//);
//}

void HiQPJointEffortController::loadRenderingParameters() {
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
int HiQPJointEffortController::loadAndSetupTaskMonitoring() {
  XmlRpc::XmlRpcValue task_monitoring;
  if (!this->getControllerNodeHandle().getParam("task_monitoring",
                                                task_monitoring)) {
    ROS_ERROR_STREAM("In HiQPJointEffortController: Call to getParam('"
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
void HiQPJointEffortController::loadJointLimitsFromParamServer() {
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
            std::to_string(static_cast<double>(limitations[1])));
        def_params.push_back(
            std::to_string(static_cast<double>(limitations[2])));

        std::vector<std::string> dyn_params;
        dyn_params.push_back("TDynJntLimits");
        dyn_params.push_back(
            std::to_string(static_cast<double>(limitations[0])));

        task_manager_.setTask(link_frame + "_jntlimits", 1, true, true, false,
                              def_params, dyn_params, this->getRobotState());
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

void HiQPJointEffortController::loadGeometricPrimitivesFromParamServer() {
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

        task_manager_.setPrimitive(name, type, frame_id, visible, color,
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

void HiQPJointEffortController::loadTasksFromParamServer() {
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

        task_manager_.setTask(name, priority, visible, active, monitored,
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

}  // namespace hiqp_ros

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp_ros::HiQPJointEffortController,
                       controller_interface::ControllerBase)
