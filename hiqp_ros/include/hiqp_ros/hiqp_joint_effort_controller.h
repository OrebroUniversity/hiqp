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

#ifndef HIQP_JOINT_EFFORT_CONTROLLER_H
#define HIQP_JOINT_EFFORT_CONTROLLER_H

#include <ros/ros.h>

namespace hiqp_ros
{

  /*! \brief A joint effort controller that provides full access to the HiQP control framework
   *  \author Marcus A Johansson */ 
  class HiQPJointEffortController : public JointEffortController {
  public:
    HiQPJointEffortController();
    ~HiQPJointEffortController() noexcept;

    bool init(JointVelocityInterface *hw, ros::NodeHandle &controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);

  private:
    ROSDynamicsController(const ROSDynamicsController& other) = delete;
    ROSDynamicsController(ROSDynamicsController&& other) = delete;
    ROSDynamicsController& operator=(const ROSDynamicsController& other) = delete;
    ROSDynamicsController& operator=(ROSDynamicsController&& other) noexcept = delete;
   };

} // namespace hiqp

#endif // include guard
