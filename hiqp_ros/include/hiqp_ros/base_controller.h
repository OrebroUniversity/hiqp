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

#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <hiqp/robot_state.h>
#include <hiqp_msgs/JointControllerState.h>
#include <hiqp_ros/utilities.h>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <realtime_tools/realtime_publisher.h>

namespace hiqp_ros {

using hiqp::HiQPTimePoint;
using hiqp::RobotState;
using hiqp::RobotStatePtr;

/*! \brief A base controller class whose controller type and hardware interface
 * type are set as template parameters. This controller includes an internal KDL
 * tree that is got from the robot_description on the parameter server. Also all
 * claimed joint resources are read from the yaml file. Only joints specified in
 * the yaml file are claimed, the others are not claimed but still read from.
 *  \author Marcus A Johansson */
template <typename HardwareInterfaceT>
class BaseController
    : public controller_interface::MultiInterfaceController<
          HardwareInterfaceT, hardware_interface::ForceTorqueSensorInterface> {
public:
  BaseController()
      : controller_interface::MultiInterfaceController<
            HardwareInterfaceT, hardware_interface::ForceTorqueSensorInterface>(
            true){};
  ~BaseController() noexcept = default;

  bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &controller_nh);

  void starting(const ros::Time &time) {}

  void update(const ros::Time &time, const ros::Duration &period);

  void stopping(const ros::Time &time) {}

  virtual void initialize() = 0;

  /*! \brief Implement this to compute the desired joint accelerations ddq and
   * corresponding joint controls u (e.g., via integration of ddq). Do not
   * resize ddq! */
  virtual void updateControls(Eigen::VectorXd &ddq, Eigen::VectorXd &u) = 0;

protected:
  inline ros::NodeHandle &getControllerNodeHandle() { return controller_nh_; }
  inline std::shared_ptr<ros::NodeHandle> getControllerNodeHandlePtr() {
    return controller_nh_ptr_;
  }
  inline unsigned int getNJoints() { return n_joints_; }
  inline RobotStatePtr getRobotState() { return robot_state_ptr_; }
  //  inline void setDesiredSamplingTime(double desired_sampling_time) {
  //   desired_sampling_time_ = desired_sampling_time;
  //}

  ros::Duration period_;

private:
  BaseController(const BaseController &other) = delete;
  BaseController(BaseController &&other) = delete;
  BaseController &operator=(const BaseController &other) = delete;
  BaseController &operator=(BaseController &&other) noexcept = delete;

  // void loadDesiredSamplingTime();
  int loadUrdfToKdlTree();
  int loadJointsAndSetJointHandlesMap();
  int loadSensorsAndSetSensorHandlesMap();
  void sampleJointValues();
  void sampleSensorValues();  
  void setControls();
  void publishControllerState();

  typedef std::map<unsigned int, hardware_interface::JointHandle>
      JointHandleMap;
  typedef std::map<std::string, hardware_interface::ForceTorqueSensorHandle>
      SensorHandleMap;

  RobotState robot_state_data_;
  RobotStatePtr robot_state_ptr_;
  HiQPTimePoint last_sampling_time_point_;
  double desired_sampling_time_;
  Eigen::VectorXd ddq_, u_;

  ros::NodeHandle controller_nh_;
  std::shared_ptr<ros::NodeHandle> controller_nh_ptr_;
  HardwareInterfaceT *jnt_hw_;
  hardware_interface::ForceTorqueSensorInterface *fts_hw_;
  JointHandleMap joint_handles_map_;
  SensorHandleMap sensor_handles_map_;

  std::mutex handles_mutex_;
  unsigned int n_joints_;
  unsigned int n_sensors_;  

  realtime_tools::RealtimePublisher<hiqp_msgs::JointControllerState>
      c_state_pub_;
  ros::Time last_c_state_update_;
  double c_state_publish_rate_;
};

//////////////////////////////////////////////////////////////////////////////
//
//                      I M P L E M E N T A T I O N
//
//////////////////////////////////////////////////////////////////////////////

//=====================================================================================
template <typename HardwareInterfaceT>
bool BaseController<HardwareInterfaceT>::init(hardware_interface::RobotHW *hw,
                                              ros::NodeHandle &controller_nh) {
  jnt_hw_ = hw->get<HardwareInterfaceT>();
  if (!jnt_hw_) {
    ROS_ERROR("Could not get joint hardware interface. Cannot initialize "
              "controller.");
    return false;
  }
  // try to initialize optional force/torque sensors
  fts_hw_ = hw->get<hardware_interface::ForceTorqueSensorInterface>();
  if (!fts_hw_)
    ROS_WARN("Could not get force/torque sensor hardware interface. No fts "
             "information will be available.");

  controller_nh_ = controller_nh;
  controller_nh_ptr_.reset(&controller_nh_);
  robot_state_ptr_.reset(&robot_state_data_);

  ros::Time t = ros::Time::now();
  last_sampling_time_point_.setTimePoint(t.sec, t.nsec);

  loadUrdfToKdlTree();
  loadJointsAndSetJointHandlesMap();
  loadSensorsAndSetSensorHandlesMap();

  sampleJointValues();
  sampleSensorValues();  
  initialize();

  c_state_pub_.init(controller_nh, "joint_controller_state", 1);
  last_c_state_update_ = ros::Time::now();
  controller_nh.param("c_state_publish_rate", c_state_publish_rate_, 100.0);
  c_state_pub_.msg_.joints.resize(n_joints_);
  c_state_pub_.msg_.sensors.resize(n_sensors_);
  
  for (auto &&it : robot_state_data_.kdl_tree_.getSegments()) {
    c_state_pub_.msg_.joints.at(it.second.q_nr).name = it.second.segment.getJoint().getName();
  }

  for (unsigned int i=0; i<n_sensors_;i++){
    c_state_pub_.msg_.sensors.at(i).name=robot_state_data_.sensor_handle_info_[i].sensor_name_;
    c_state_pub_.msg_.sensors.at(i).frame_id=robot_state_data_.sensor_handle_info_[i].frame_id_;    
  }

  /* for (unsigned i=0; i<sensor_names.size(); i++){ */
  /*   // sensor handle */
  /*   sensors_.push_back(hw_e->getHandle(sensor_names[i])); */

  /* for (auto &&handle : joint_handles_map_) { */
  /*   //    std::cerr<<"at: "<<handle.first<<", name:
   * "<<handle.second.getName()<<std::endl; */
  /*       c_state_pub_.msg_.name.at(handle.first) = handle.second.getName(); */
  /* } */
  return true;
}
//=====================================================================================
template <typename HardwareInterfaceT>
void BaseController<HardwareInterfaceT>::update(const ros::Time &time,
                                                const ros::Duration &period) {
  period_ = period;
  sampleJointValues();
  sampleSensorValues();    
  updateControls(ddq_, u_);
  setControls();
  publishControllerState();
}
//=====================================================================================
template <typename HardwareInterfaceT>
int BaseController<HardwareInterfaceT>::loadUrdfToKdlTree() {
  std::string full_parameter_path;
  std::string robot_urdf;
  if (controller_nh_.searchParam("robot_description", full_parameter_path)) {
    controller_nh_.getParam(full_parameter_path, robot_urdf);
    bool success =
        kdl_parser::treeFromString(robot_urdf, robot_state_data_.kdl_tree_);
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
template <typename HardwareInterfaceT>
int BaseController<HardwareInterfaceT>::loadSensorsAndSetSensorHandlesMap() {
  if (!fts_hw_)
    return -1;

  const std::vector<std::string> &sensor_names = fts_hw_->getNames();
  n_sensors_= sensor_names.size();
  for (unsigned i = 0; i < n_sensors_; i++) {
    ROS_DEBUG("Got sensor %s", sensor_names[i].c_str());
    sensor_handles_map_.emplace(sensor_names[i],
                                fts_hw_->getHandle(sensor_names[i]));
  }

  return 0;
}
//=====================================================================================
template <typename HardwareInterfaceT>
int BaseController<HardwareInterfaceT>::loadJointsAndSetJointHandlesMap() {
  std::string param_name = "joints";
  std::vector<std::string> joint_names;
  if (!controller_nh_.getParam(param_name, joint_names)) {
    ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('"
                     << param_name << "') in namespace '"
                     << controller_nh_.getNamespace() << "' failed.");
    return -1;
  }

  unsigned int n_joint_names = joint_names.size();
  n_joints_ = robot_state_data_.kdl_tree_.getNrOfJoints();

  if (n_joint_names > n_joints_) {
    ROS_ERROR_STREAM(
        "In ROSKinematicsController: The .yaml file"
        << " includes more joint names (" << n_joint_names
        << ") than specified in the .urdf file (" << n_joints_
        << ") Could not successfully initialize controller. Aborting!\n");
    return -3;
  }

  std::vector<unsigned int> qnrs;
  hiqp::kdl_getAllQNrFromTree(robot_state_data_.kdl_tree_, qnrs);
  robot_state_data_.joint_handle_info_.clear();

  for (auto &&name : joint_names) {
    try {
      unsigned int q_nr =
          hiqp::kdl_getQNrFromJointName(robot_state_data_.kdl_tree_, name);
      // std::cout << "Joint found: '" << name << "', qnr: " << q_nr << "\n";
      joint_handles_map_.emplace(q_nr, jnt_hw_->getHandle(name));
      qnrs.erase(std::remove(qnrs.begin(), qnrs.end(), q_nr), qnrs.end());
      robot_state_data_.joint_handle_info_.push_back(
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
        hiqp::kdl_getJointNameFromQNr(robot_state_data_.kdl_tree_, qnr);
    hiqp::JointHandleInfo jhi(qnr, joint_name, true, false);
    robot_state_data_.joint_handle_info_.push_back(jhi);
  }

  std::cout << "Joint handle info:\n";
  for (auto &&jhi : robot_state_data_.joint_handle_info_) {
    std::cout << jhi.q_nr_ << ", " << jhi.joint_name_ << ", " << jhi.readable_
              << ", " << jhi.writable_ << "\n";
  }

  robot_state_data_.kdl_jnt_array_vel_.resize(n_joints_);
  KDL::SetToZero(robot_state_data_.kdl_jnt_array_vel_.q);
  KDL::SetToZero(robot_state_data_.kdl_jnt_array_vel_.qdot);
  robot_state_data_.kdl_effort_.resize(n_joints_);
  KDL::SetToZero(robot_state_data_.kdl_effort_);
  ddq_ = Eigen::VectorXd::Zero(n_joints_);
  u_ = Eigen::VectorXd::Zero(n_joints_);
  return 0;
}
 //=====================================================================================
template <typename HardwareInterfaceT>
void BaseController<HardwareInterfaceT>::sampleSensorValues() {

   if (!fts_hw_)
    return;

  handles_mutex_.lock();

  for( unsigned int i=0; i<robot_state_data_.sensor_handle_info_.size();i++){
    hiqp::SensorHandleInfo &h = robot_state_data_.sensor_handle_info_[i];
    h.force_=Eigen::Map<Eigen::Vector3d>(const_cast<double*>(sensor_handles_map_.at(h.sensor_name_).getForce()));
    h.torque_=Eigen::Map<Eigen::Vector3d>(const_cast<double*>(sensor_handles_map_.at(h.sensor_name_).getTorque()));    
  }

  handles_mutex_.unlock();
}
//=====================================================================================
template <typename HardwareInterfaceT>
void BaseController<HardwareInterfaceT>::sampleJointValues() {
  robot_state_data_.sampling_time_ = period_.toSec();

  KDL::JntArray &q = robot_state_data_.kdl_jnt_array_vel_.q;
  KDL::JntArray &qdot = robot_state_data_.kdl_jnt_array_vel_.qdot;
  KDL::JntArray &effort = robot_state_data_.kdl_effort_;
  q.data.setZero();
  qdot.data.setZero();
  effort.data.setZero();

  handles_mutex_.lock();
  for (auto &&handle : joint_handles_map_) {
    q(handle.first) = handle.second.getPosition();
    qdot(handle.first) = handle.second.getVelocity();
    effort(handle.first) = handle.second.getEffort();
  }

  handles_mutex_.unlock();
}
//=====================================================================================
template <typename HardwareInterfaceT>
void BaseController<HardwareInterfaceT>::setControls() {
  handles_mutex_.lock();
  for (auto &&handle : joint_handles_map_) {
    handle.second.setCommand(u_(handle.first));
  }
  handles_mutex_.unlock();
}
//=====================================================================================
template <typename HardwareInterfaceT>
void BaseController<HardwareInterfaceT>::publishControllerState() {

  ros::Time now = ros::Time::now();
  ros::Duration d = now - last_c_state_update_;
  if (d.toSec() >= 1.0 / c_state_publish_rate_) {
    last_c_state_update_ = now;

    if (c_state_pub_.trylock()) {

      KDL::JntArray q = robot_state_data_.kdl_jnt_array_vel_.q;
      KDL::JntArray qdot = robot_state_data_.kdl_jnt_array_vel_.qdot;
      KDL::JntArray effort = robot_state_data_.kdl_effort_;

      c_state_pub_.msg_.header.stamp = ros::Time::now();

      for (unsigned int i = 0; i < n_joints_; i++) {
        c_state_pub_.msg_.joints[i].command = u_(i);
        c_state_pub_.msg_.joints[i].position = q(i);
        c_state_pub_.msg_.joints[i].velocity = qdot(i);
        c_state_pub_.msg_.joints[i].effort = effort(i);
      }
      for (unsigned int i = 0; i < n_sensors_; i++) {
	c_state_pub_.msg_.sensors[i].force.clear();
	c_state_pub_.msg_.sensors[i].force.push_back(robot_state_data_.sensor_handle_info_[i].force_(0));
	c_state_pub_.msg_.sensors[i].force.push_back(robot_state_data_.sensor_handle_info_[i].force_(1));
	c_state_pub_.msg_.sensors[i].force.push_back(robot_state_data_.sensor_handle_info_[i].force_(2));
	c_state_pub_.msg_.sensors[i].torque.clear();		
	c_state_pub_.msg_.sensors[i].torque.push_back(robot_state_data_.sensor_handle_info_[i].torque_(0));
	c_state_pub_.msg_.sensors[i].torque.push_back(robot_state_data_.sensor_handle_info_[i].torque_(1));
	c_state_pub_.msg_.sensors[i].torque.push_back(robot_state_data_.sensor_handle_info_[i].torque_(2));	
      }

      c_state_pub_.unlockAndPublish();
    }
  }
}

} // namespace force_controllers

#endif // include guard
