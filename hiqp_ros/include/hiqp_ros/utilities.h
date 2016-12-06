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

#ifndef HIQP_ROS_UTILITIES_H
#define HIQP_ROS_UTILITIES_H

#include <iostream>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl/framevel.hpp>
//#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
//#include <kdl/jacobian.hpp>

std::ostream& operator<<(std::ostream& os, const KDL::Vector& kdl_vector);
std::ostream& operator<<(std::ostream& os, const KDL::Tree& kdl_tree);
std::ostream& operator<<(std::ostream& os, const KDL::FrameVel& kdl_frame_vel);
std::ostream& operator<<(std::ostream& os, const KDL::JntArrayVel& kdl_joints_vel);
std::ostream& operator<<(std::ostream& os, const KDL::Chain& kdl_chain);

#endif