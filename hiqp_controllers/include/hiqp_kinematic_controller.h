#ifndef HIQP_KINEMATIC_CONTROLLER_H
#define HIQP_KINEMATIC_CONTROLLER_H

/*!
 * \file HiQP_Kinematic_Controller.h
 * \brief HiQP_Kinematic_Controller is my super-nice controller
 * \author Marcus A Johansson
 * \version 0.1
 * \date 2016-06-21
 */

// STL Includes
#include <string>
#include <vector>
#include <mutex>



// ROS Messages


// ROS Services


// ROS Includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/posvel_command_interface.h>

// Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

// HiQP
#include <task_manager.h>










/*!
 * \namespace hiqp
 * 
 * namespace for HiQP-related stuff
 */
namespace hiqp
{

/** A name for the standard joint-velocity-controller in ROS */
typedef 
controller_interface::Controller<hardware_interface::VelocityJointInterface>
JointVelocityController;

/** A name for the standard joint-velocity hardware interface in ROS */
typedef hardware_interface::VelocityJointInterface JointVelocityInterface;

/*!
 * \class HiQP_Kinematic_Controller
 * \brief This is my awesome controller
 *
 *  It's awesome!
 */	
class HiQP_Kinematic_Controller : public JointVelocityController
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome controller
     */
	HiQP_Kinematic_Controller();

	/*!
     * \brief Destructor
     * Destructs my awesome controller :(
     */
	~HiQP_Kinematic_Controller() noexcept;
	

	/*!
     * \brief Called every time the controller is initialized by the 
     *        ros::controller_manager
     *
     * Does some cool stuff!
     *
     * \param hw : a pointer to the hardware interface used by this controller
     * \param controller_nh : the node handle of this controller
     * \return true if the initialization was successful
     */
	bool init(JointVelocityInterface *hw, 
		     ros::NodeHandle &controller_nh);

	/*!
     * \brief Called every time the controller is started by the 
     *        ros::controller_manager
     *
     * Does some cool stuff!
     *
     * \param time : the current wall-time in ROS
     * \return true if the starting was successful
     */
	void starting(const ros::Time& time);

	/*!
     * \brief Called every time the controller is updated by the 
     *        ros::controller_manager
     *
     * Does some cool stuff!
     *
     * \param time : the current wall-time in ROS
     * \param period : the time between the last update call and this, i.e.
     *                 the sample time.
     * \return true if the update was successful
     */
	void update(const ros::Time& time, const ros::Duration& period);

	/*!
     * \brief Called every time the controller is stopped by the 
     *        ros::controller_manager
     *
     * Does some cool stuff!
     *
     * \param time : the current wall-time in ROS
     * \return true if the stopping was successful
     */
	void stopping(const ros::Time& time);








private:

	// No copying of this class is allowed !
	HiQP_Kinematic_Controller(const HiQP_Kinematic_Controller& other) = delete;
	HiQP_Kinematic_Controller(HiQP_Kinematic_Controller&& other) = delete;
	HiQP_Kinematic_Controller& operator=(const HiQP_Kinematic_Controller& other) = delete;
	HiQP_Kinematic_Controller& operator=(HiQP_Kinematic_Controller&& other) noexcept = delete;




     ros::NodeHandle                                   controller_nh_;
     bool                                              is_active_;

     std::vector< std::string >                        joint_names_;
     unsigned int                                      n_joints_;
     std::vector< hardware_interface::JointHandle >    joint_handles_;

     KDL::Tree                                         kdl_tree_;
     KDL::JntArray                                     kdl_joint_pos_;

     std::mutex                                        handles_mutex_;

     std::vector<double>                               output_controls_;

     TaskManager                                       task_manager_;

};

}

#endif // include guard
