/*!
 * \file   hiqp_kinematic_controller.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_HIQP_KINEMATIC_CONTROLLER_H
#define HIQP_HIQP_KINEMATIC_CONTROLLER_H


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
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>

// HiQP Includes
#include <hiqp/task_manager.h>










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
 * \class HiQPKinematicController
 * \brief This is my awesome controller
 *
 *  It's awesome!
 */	
class HiQPKinematicController : public JointVelocityController
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome controller
     */
	HiQPKinematicController();

	/*!
     * \brief Destructor
     * Destructs my awesome controller :(
     */
	~HiQPKinematicController() noexcept;
	

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
     * The function:
     * <ol>
     *   <li>locks a mutex and reads position and velocity values from the joint handles,</li>
     *   <li>calls getKinematicControls() on its task manager,</li>
     *   <li>and locks a mutex and writes velocity values to the joint handles.</li>
     * </ol>
     * The joint handles are stored as a map between the joints q-number in the KDL::Tree and the joint handles themselves.
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
	HiQPKinematicController(const HiQPKinematicController& other) = delete;
	HiQPKinematicController(HiQPKinematicController&& other) = delete;
	HiQPKinematicController& operator=(const HiQPKinematicController& other) = delete;
	HiQPKinematicController& operator=(HiQPKinematicController&& other) noexcept = delete;




     ros::NodeHandle                                   controller_nh_;
     bool                                              is_active_;

     typedef std::map<unsigned int, hardware_interface::JointHandle >
          JointHandleMap;
     typedef std::pair<unsigned int, hardware_interface::JointHandle >
          JointHandleMapEntry;

     JointHandleMap                                    joint_handles_map_;

     KDL::Tree                                         kdl_tree_;
     KDL::JntArrayVel                                  kdl_joint_pos_vel_;

     std::mutex                                        handles_mutex_;

     std::vector<double>                               output_controls_;

     TaskManager                                       task_manager_;

};

}

#endif // include guard
