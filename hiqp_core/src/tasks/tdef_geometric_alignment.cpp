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

#include <hiqp/tasks/tdef_geometric_alignment.h>

#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_frame.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <hiqp/utilities.h>

#include <cmath>
#include <iostream>

namespace hiqp {
namespace tasks {

template <>
int TDefGeometricAlignment<GeometricLine, GeometricLine>::align(
    std::shared_ptr<GeometricLine> line1,
    std::shared_ptr<GeometricLine> line2,
    const RobotStatePtr robot_state) {
  KDL::Vector v1 = pose_a_.M * line1->getDirectionKDL();
  KDL::Vector v2 = pose_b_.M * line2->getDirectionKDL();

  return alignVectors(v1, v2, robot_state);
}

template <>
int TDefGeometricAlignment<GeometricLine, GeometricPlane>::align(
    std::shared_ptr<GeometricLine> line,
    std::shared_ptr<GeometricPlane> plane,
    const RobotStatePtr robot_state) {
  KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();
  KDL::Vector v2 = pose_b_.M * plane->getNormalKDL();

  return alignVectors(v1, v2, robot_state);
}

template <>
int TDefGeometricAlignment<GeometricLine, GeometricCylinder>::align(
    std::shared_ptr<GeometricLine> line,
    std::shared_ptr<GeometricCylinder> cylinder,
    const RobotStatePtr robot_state) {
  KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();

  KDL::Vector p = pose_a_.p + pose_a_.M * line->getOffsetKDL(); //a point on the line expressed in the world frame
  KDL::Vector d = pose_b_.p + pose_b_.M * cylinder->getOffsetKDL();
  KDL::Vector v = pose_b_.M * cylinder->getDirectionKDL();

  KDL::Vector x = KDL::dot((p - d), v) * v;

  KDL::Vector v2 = d + x -p;

  v2.Normalize();

  // DEBUG ===================================
  //std::cerr<<"v1: "<<v1(0)<<" "<<v1(1)<<" "<<v1(2)<<std::endl;
  //std::cerr<<"v2: "<<v2(0)<<" "<<v2(1)<<" "<<v2(2)<<std::endl;  
  // DEBUG END ===============================

  return alignVectors(v1, v2, robot_state);
}

template <>
int TDefGeometricAlignment<GeometricLine, GeometricSphere>::align(
    std::shared_ptr<GeometricLine> line,
    std::shared_ptr<GeometricSphere> sphere,
    const RobotStatePtr robot_state) {
  KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();

  KDL::Vector p = pose_a_.p + pose_a_.M * line->getOffsetKDL();
  KDL::Vector d = pose_b_.p + pose_b_.M * sphere->getCenterKDL();

  KDL::Vector v2 = d - p;
  v2.Normalize();

  return alignVectors(v1, v2, robot_state);
}

  template <>
  int TDefGeometricAlignment<GeometricFrame, GeometricFrame>::align(
								    std::shared_ptr<GeometricFrame> frame1,
								    std::shared_ptr<GeometricFrame> frame2,
								    const RobotStatePtr robot_state) {
    KDL::Vector ax1 = pose_a_.M * frame1->getAxisXKDL();
    KDL::Vector ax2 = pose_b_.M * frame2->getAxisXKDL();
    KDL::Vector ay1 = pose_a_.M * frame1->getAxisYKDL();
    KDL::Vector ay2 = pose_b_.M * frame2->getAxisYKDL();

    //abuse the alignVectors function to compute the relevant quantities to align both axis
    alignVectors(ay1, ay2, robot_state);

    //temporary save the task errors/jacobians
    Eigen::VectorXd ey=e_;
    Eigen::VectorXd ey_dot=e_dot_;  
    Eigen::MatrixXd Jy=J_;
    Eigen::MatrixXd Jy_dot=J_dot_;  
    double q_nr=J_.cols();

    //overwrite the class member errors/jacobians
    alignVectors(ax1, ax2, robot_state);

    //append the previously computed quantities
    e_.conservativeResize(2);
    e_(1)=ey(0);
    e_dot_.conservativeResize(2);
    e_dot_(1)=ey_dot(0);
    J_.conservativeResize(2,q_nr);
    J_.bottomRows<1>()=Jy;
    J_dot_.conservativeResize(2,q_nr);
    J_dot_.bottomRows<1>()=Jy_dot;
  
    return 0;
  }

}  // namespace tasks

}  // namespace hiqp
