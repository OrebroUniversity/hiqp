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

#ifndef HIQP_ROBOT_STATE_H
#define HIQP_ROBOT_STATE_H

#include <memory>
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>
#include <hiqp/hiqp_time_point.h>

namespace hiqp {

  /*! \brief Holds the state of the robot (sampling time, kdl tree, joint positions and velocities) 
   *  \author Marcus A Johansson */
  struct RobotState {
    HiQPTimePoint     sampling_time_;
    KDL::Tree         kdl_tree_;
    KDL::JntArrayVel  kdl_jnt_array_vel_;
    KDL::JntArray     kdl_effort_;
  };

  /*! \brief A const pointer type to a robot state. Used to reference to the current robot state throught the framework.
   *  \author Marcus A Johansson */
  typedef std::shared_ptr<const RobotState> RobotStatePtr;

} // namespace hiqp

#endif // include guard