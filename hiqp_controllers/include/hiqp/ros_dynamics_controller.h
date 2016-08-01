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
 * \file   ros_kinematic_controller.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_ROS_KINEMATIC_CONTROLLER_H
#define HIQP_ROS_KINEMATIC_CONTROLLER_H


// ROS Includes
#include <ros/ros.h>



/*!
 * \namespace hiqp
 * 
 * namespace for HiQP-related stuff
 */
namespace hiqp
{


/*!
 * \class ROSDynamicsController
 * \brief This is my awesome controller
 *
 *  It's awesome!
 */	
class ROSDynamicsController : public JointEffortController
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome controller
     */
	ROSDynamicsController();

	/*!
     * \brief Destructor
     * Destructs my awesome controller :(
     */
	~ROSDynamicsController() noexcept;

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
     * The joint handles are stored as a map between the joints q-number in the 
     * KDL::Tree and the joint handles themselves.
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
	ROSDynamicsController(const ROSDynamicsController& other) = delete;
	ROSDynamicsController(ROSDynamicsController&& other) = delete;
	ROSDynamicsController& operator=(const ROSDynamicsController& other) = delete;
	ROSDynamicsController& operator=(ROSDynamicsController&& other) noexcept = delete;



};

}

#endif // include guard
