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

#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <filters/transfer_function.h> 


#include <hiqp/robot_state.h>
#include <hiqp_ros/utilities.h>
#include <hiqp_ros/kalman/hiqp_kfilter.hpp>

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
    : public controller_interface::Controller<HardwareInterfaceT> {
  public:
    BaseController() = default;
    ~BaseController() noexcept = default;

    bool init(HardwareInterfaceT* hw, ros::NodeHandle& controller_nh);

    void starting(const ros::Time& time) {}

    void update(const ros::Time& time, const ros::Duration& period);

    void stopping(const ros::Time& time) {}

    virtual void initialize() = 0;

    /*! \brief Implement this to compute the desired joint accelerations ddq and corresponding joint controls u (e.g., via integration of ddq). Do not resize ddq! */
    virtual void updateControls(Eigen::VectorXd& ddq, Eigen::VectorXd& u) = 0;

  protected:
    inline ros::NodeHandle& getControllerNodeHandle() { return controller_nh_; }
    inline std::shared_ptr<ros::NodeHandle> getControllerNodeHandlePtr() {
      return controller_nh_ptr_;
    }
    inline unsigned int getNJoints() { return n_joints_; }
    inline RobotStatePtr getRobotState() { return robot_state_ptr_; }
    inline void setDesiredSamplingTime(double desired_sampling_time) {
      desired_sampling_time_ = desired_sampling_time;
    }
  
    ros::Duration period_;
    filters::MultiChannelTransferFunctionFilter<double> input_vel_filter_;
    filters::MultiChannelTransferFunctionFilter<double> input_pos_filter_;   
  
    
  private:
    BaseController(const BaseController& other) = delete;
    BaseController(BaseController&& other) = delete;
    BaseController& operator=(const BaseController& other) = delete;
    BaseController& operator=(BaseController&& other) noexcept = delete;

    // void loadDesiredSamplingTime();
    int loadUrdfToKdlTree();
    int loadJointsAndSetJointHandlesMap();
    void sampleJointValues();
    void setControls();
    void updateStateFilter();
  
    typedef std::map<unsigned int, hardware_interface::JointHandle>
      JointHandleMap;

    RobotState robot_state_data_;
    RobotStatePtr robot_state_ptr_;
    HiQPTimePoint last_sampling_time_point_;
    double desired_sampling_time_;
    Eigen::VectorXd ddq_, u_;

    ros::NodeHandle controller_nh_;
    std::shared_ptr<ros::NodeHandle> controller_nh_ptr_;
    HardwareInterfaceT* hardware_interface_;
    JointHandleMap joint_handles_map_;
    std::mutex handles_mutex_;
    unsigned int n_joints_;

    HiQPKFilter k_filter_;
  };

  //////////////////////////////////////////////////////////////////////////////
  //
  //                      I M P L E M E N T A T I O N
  //
  //////////////////////////////////////////////////////////////////////////////

  template <typename HardwareInterfaceT>
    bool BaseController<HardwareInterfaceT>::init(HardwareInterfaceT* hw,
						  ros::NodeHandle& controller_nh) {
    hardware_interface_ = hw;
    controller_nh_ = controller_nh;
    controller_nh_ptr_.reset(&controller_nh_);
    robot_state_ptr_.reset(&robot_state_data_);

    ros::Time t = ros::Time::now();
    last_sampling_time_point_.setTimePoint(t.sec, t.nsec);

    loadUrdfToKdlTree();
    loadJointsAndSetJointHandlesMap();

    // initialize input velocity filter 
    if(input_vel_filter_.filters::MultiChannelFilterBase<double>::configure(robot_state_ptr_->kdl_jnt_array_vel_.q.rows(),"input_vel_filter", controller_nh_)){ 
      ROS_INFO_STREAM("Configured input velocity filter with name: "<<input_vel_filter_.getName()<<", type: "<<input_vel_filter_.getType()<<", nr. of channels: "<<robot_state_ptr_->kdl_jnt_array_vel_.q.rows()<<".");
    }
    else{
      ROS_WARN("Error in BaseController<HardwareInterfaceT>::init(...): Could not configure input velocity filter - no filtering will be applied!");
    }
    // initialize input position filter 
    if(input_pos_filter_.filters::MultiChannelFilterBase<double>::configure(robot_state_ptr_->kdl_jnt_array_vel_.q.rows(),"input_pos_filter", controller_nh_)){ 
      ROS_INFO_STREAM("Configured input position filter with name: "<<input_pos_filter_.getName()<<", type: "<<input_pos_filter_.getType()<<", nr. of channels: "<<robot_state_ptr_->kdl_jnt_array_vel_.q.rows()<<".");
    }
    else{
      ROS_WARN("Error in BaseController<HardwareInterfaceT>::init(...): Could not configure input position filter - no filtering will be applied!");
    }
      
    sampleJointValues();
    initialize();
    
    // INITIALIZE KALMAN FILTER
    XmlRpc::XmlRpcValue config;
    if (!controller_nh.getParam("kalman_state_filter", config)){
      ROS_WARN("Error in BaseController<HardwareInterfaceT>::init(...): Couldn't load kalman filter parameters from the server!");
    }
    else{
      //get the params map
      XmlRpc::XmlRpcValue _params = config["params"];
      if(_params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
	{
	  ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): Kalman params must be a map");
	}
      else{
        //Load params into map
	std::map< std::string, XmlRpc::XmlRpcValue > params;
        for(XmlRpc::XmlRpcValue::iterator it = _params.begin(); it != _params.end(); ++it)
	  {
	    ROS_DEBUG("Loading param %s\n", it->first.c_str());
	    params[it->first] = it->second;
	  }

	std::map< std::string, XmlRpc::XmlRpcValue >::iterator it = params.find("r_p");
	if (it == params.end())
	  {
	    ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): could not find Kalman parameter r_p");
	  }
	ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);
	double r_p=it->second;
	it = params.find("r_v");
	if (it == params.end())
	  {
	    ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): could not find Kalman parameter r_v");
	  }
	ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);
	double r_v=it->second;
	it = params.find("r_pv");
	if (it == params.end())
	  {
	    ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): could not find Kalman parameter r_pv");
	  }
	ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);
	double r_pv=it->second;
	it = params.find("q_p");
	if (it == params.end())
	  {
	    ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): could not find Kalman parameter q_p");
	  }
	ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);
	double q_p=it->second;
	it = params.find("q_v");
	if (it == params.end())
	  {
	    ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): could not find Kalman parameter q_v");
	  }
	ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);
	double q_v=it->second;
	it = params.find("q_pv");
	if (it == params.end())
	  {
	    ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): could not find Kalman parameter q_pv");
	  }
	ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);
	double q_pv=it->second;
	it = params.find("p0");
	if (it == params.end())
	  {
	    ROS_ERROR("Error in BaseController<HardwareInterfaceT>::init(...): could not find Kalman parameter p0");
	  }
	ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);
	double p0=it->second;
	
	KDL::JntArray q = robot_state_data_.kdl_jnt_array_vel_.q;
	KDL::JntArray qdot = robot_state_data_.kdl_jnt_array_vel_.qdot;
	unsigned int q_nr=q.rows();
	k_filter_.setDim(2*q_nr, q_nr, 2*q_nr, 2*q_nr, 2*q_nr);
	Eigen::MatrixXd _R = Eigen::MatrixXd::Ones(q_nr*2,q_nr*2)*r_pv;
	Eigen::MatrixXd _Q = Eigen::MatrixXd::Ones(q_nr*2,q_nr*2)*q_pv;
	Eigen::MatrixXd _P0 = Eigen::MatrixXd::Identity(q_nr*2,q_nr*2)*p0;    
	Eigen::VectorXd _x0(2*q_nr);
	    
	for(unsigned int i=0; i<q_nr;i++){
	  _x0(2*i)=q.data(i);
	  _x0(2*i+1)=qdot.data(i);
	  _R(2*i,2*i)=r_p;
	  _R(2*i+1,2*i+1)=r_v;
	  _Q(2*i,2*i)=q_p;
	  _Q(2*i+1,2*i+1)=q_v;
	}
	KFVector x0(2*q_nr,_x0.data());
	KFMatrix R(2*q_nr,2*q_nr,_R.data());
	KFMatrix Q(2*q_nr,2*q_nr,_Q.data());
	KFMatrix P0(2*q_nr,2*q_nr,_P0.data());
	
	k_filter_.init(x0,P0,R,Q,period_.toSec());
      }
    }
    return true;
  }

  template <typename HardwareInterfaceT>
    void BaseController<HardwareInterfaceT>::update(const ros::Time& time,
						    const ros::Duration& period) {
    period_=period;
    sampleJointValues();
    updateStateFilter();    //Kalman filter step using zk, uk-1
    updateControls(ddq_, u_);
    setControls();
  }

  template <typename HardwareInterfaceT>
    int BaseController<HardwareInterfaceT>::loadUrdfToKdlTree() {
    std::string full_parameter_path;
    std::string robot_urdf;
    if (controller_nh_.searchParam("robot_description", full_parameter_path)) {
      controller_nh_.getParam(full_parameter_path, robot_urdf);
      ROS_ASSERT(
		 kdl_parser::treeFromString(robot_urdf, robot_state_data_.kdl_tree_));
      ROS_INFO(
	       "Loaded the robot's urdf model and initialized the KDL tree "
	       "successfully");
    } else {
      ROS_ERROR(
		"Could not find parameter 'robot_description' on the parameter "
		"server.");
      return -1;
    }
    return 0;
  }

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
		       << " includes more joint names than specified in the .urdf file."
		       << " Could not successfully initialize controller. Aborting!\n");
      return -3;
    }

    std::vector<unsigned int> qnrs;
    hiqp::kdl_getAllQNrFromTree(robot_state_data_.kdl_tree_, qnrs);
    robot_state_data_.joint_handle_info_.clear();

    for (auto&& name : joint_names) {
      try {
	unsigned int q_nr =
          hiqp::kdl_getQNrFromJointName(robot_state_data_.kdl_tree_, name);
	// std::cout << "Joint found: '" << name << "', qnr: " << q_nr << "\n";
	joint_handles_map_.emplace(q_nr, hardware_interface_->getHandle(name));
	qnrs.erase(std::remove(qnrs.begin(), qnrs.end(), q_nr), qnrs.end());
	robot_state_data_.joint_handle_info_.push_back(
						       hiqp::JointHandleInfo(q_nr, name, true, true));
      } catch (const hardware_interface::HardwareInterfaceException& e) {
	ROS_ERROR_STREAM("Exception thrown: " << e.what());
	return -2;
      }
      // catch (MAP INSERT FAIL EXCEPTION)
      // catch (HIQP Q_NR NOT AVAILABLE EXCEPTION)
    }

    for (auto&& qnr : qnrs) {
      std::string joint_name =
        hiqp::kdl_getJointNameFromQNr(robot_state_data_.kdl_tree_, qnr);
      hiqp::JointHandleInfo jhi(qnr, joint_name, true, false);
      robot_state_data_.joint_handle_info_.push_back(jhi);
    }

    std::cout << "Joint handle info:\n";
    for (auto&& jhi : robot_state_data_.joint_handle_info_) {
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

  template <typename HardwareInterfaceT>
    void BaseController<HardwareInterfaceT>::sampleJointValues() {
    robot_state_data_.sampling_time_ = period_.toSec();
  
    KDL::JntArray& q = robot_state_data_.kdl_jnt_array_vel_.q;
    KDL::JntArray& qdot = robot_state_data_.kdl_jnt_array_vel_.qdot;
    KDL::JntArray& effort = robot_state_data_.kdl_effort_;
    q.data.setZero();
    qdot.data.setZero();
    effort.data.setZero();
	    
    handles_mutex_.lock();
    for (auto&& handle : joint_handles_map_) {
      q(handle.first) = handle.second.getPosition();
      qdot(handle.first) = handle.second.getVelocity();
      effort(handle.first) = handle.second.getEffort();
    }

    handles_mutex_.unlock();
  }

  template <typename HardwareInterfaceT>
    void BaseController<HardwareInterfaceT>::setControls() {
    handles_mutex_.lock();
    for (auto&& handle : joint_handles_map_) {
      handle.second.setCommand(u_(handle.first));
    }
    handles_mutex_.unlock();
  }
 
  template <typename HardwareInterfaceT>
    void BaseController<HardwareInterfaceT>::updateStateFilter() {
    KDL::JntArray& q = robot_state_data_.kdl_jnt_array_vel_.q;
    KDL::JntArray& qdot = robot_state_data_.kdl_jnt_array_vel_.qdot;
    unsigned int q_nr=q.rows();
    
    //KALMAN FILTER UPDATE
    k_filter_.setSamplingTime(period_.toSec());

    Eigen::VectorXd _z(2*q_nr);
    for(unsigned int i=0; i<q_nr;i++){
      _z(2*i)=q.data(i);
      _z(2*i+1)=qdot.data(i);
    }
    KFVector z(2*q_nr,_z.data());
    KFVector ddq(q_nr, ddq_.data());
    k_filter_.step(ddq,z);
    
    KFVector x=k_filter_.getX();
    for(unsigned int i=0; i<q_nr;i++){
      q.data(i)=x(2*i);
      qdot.data(i)=x(2*i+1);

    }

    /* //TRANSFER FUNCTION FILTER UPDATE */
    // std::vector<double> q_in(q_nr), qdot_in(q_nr), q_out(q_nr), qdot_out(q_nr);
    /* for(unsigned int i=0; i<q_nr; i++){ */
    /*   q_in[i]=q.data(i); */
    /*   qdot_in[i]=qdot.data(i); */
    /* } */
    /* if(input_pos_filter_.update(q_in,q_out)){ */
    /*   for(unsigned int i=0; i<q_nr; i++)  q.data(i)=q_out[i]; */
    /* } */
    /* else{ */
    /*   ROS_WARN("BaseController<HardwareInterfaceT>::sampleJointValues(): could not update input position filter!"); */
    /* } */
    /* if(input_vel_filter_.update(qdot_in,qdot_out)){ */
    /*   for(unsigned int i=0; i<q_nr; i++)  qdot.data(i)=qdot_out[i]; */
    /* } */
    /* else{ */
    /*   ROS_WARN("BaseController<HardwareInterfaceT>::sampleJointValues(): could not update input velocity filter!"); */
    /* } */

  /* for (auto&& handle : joint_handles_map_) { */
  /*     q(handle.first) = handle.second.getPosition(); */
  /*     qdot(handle.first) = handle.second.getVelocity(); */
  /*     effort(handle.first) = handle.second.getEffort(); */
  /*   } */
    
  }
  

}  // namespace force_controllers

#endif  // include guard
