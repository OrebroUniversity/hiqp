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

#include <hiqp/tasks/tdef_geometric_projection.h>

#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_frame.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <hiqp/utilities.h>

#include <iostream>
#include <sstream>
#include <string>

namespace hiqp {
namespace tasks {

/// \todo Implement point-capsule projection
/// \todo Implement cylinder-cylinder projection
/// \todo Implement cylinder-sphere projection
/// \todo Implement cylinder-capsule projection
/// \todo Implement sphere-capsule projection
/// \todo Implement capsule-capsule projection
/// \todo Implement activation zones for all tasks

///////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
///////////////////////////////////////////////////////////////////////////////
//
//                                 P O I N T
//
///////////////////////////////////////////////////////////////////////////////

template <>
int TDefGeometricProjection<GeometricPoint, GeometricPoint>::project(
    std::shared_ptr<GeometricPoint> point1,
    std::shared_ptr<GeometricPoint> point2,
    const KDL::JntArrayVel& qqdot) {

  
  KDL::Vector p1__ = pose_a_.M * point1->getPointKDL(); //point 1 from link origin to ee expressed in the world frame
  KDL::Vector p1 = pose_a_.p + p1__;  //absolute ee point 1 expressed in the world frame
  
  KDL::Vector p2__ = pose_b_.M * point2->getPointKDL(); //point 2 from link origin to ee expressed in the world frame
  KDL::Vector p2 = pose_b_.p + p2__; //absolute ee point 2 expressed in the world frame

  KDL::Vector d = p1 - p2; //distance vector expressed in the world frame  

  double q_nr=jacobian_a_.columns();
  KDL::Jacobian J_p2, J_p1p2, J_dot_p2, J_dot_p1p2;
  J_p2.resize(q_nr);
  J_p1p2.resize(q_nr);
  J_dot_p2.resize(q_nr);
  J_dot_p1p2.resize(q_nr);      

  changeJacRefPoint(jacobian_a_, p1__, J_p1p2);
  changeJacRefPoint(jacobian_b_, p2__, J_p2);
  J_p1p2.data = J_p1p2.data - J_p2.data;

  changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,qqdot, p1__, J_dot_p1p2);
  changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,qqdot, p2__, J_dot_p2);
  J_dot_p1p2.data = J_dot_p1p2.data - J_dot_p2.data;
  Eigen::Vector3d d_dot=J_p1p2.data.topRows<3>()*qqdot.qdot.data;
    
  e_(0)=d.Norm();
  J_=J_=Eigen::Map<Eigen::Matrix<double,1,3> >(d.data)/e_(0)*J_p1p2.data.topRows<3>();

  e_dot_=J_*qqdot.qdot.data;
  J_dot_=Eigen::Map<Eigen::Matrix<double,1,3> >(d.data)/e_(0)*J_dot_p1p2.data.topRows<3>()+(d_dot.transpose()/e_(0) - (Eigen::Map<Eigen::Matrix<double,1,3> >(d.data)*d_dot)*Eigen::Map<Eigen::Matrix<double,1,3> >(d.data)/pow(e_(0)*e_(0),1.5))*J_p1p2.data.topRows<3>();
   

  // DEBUG =========================================================
  // std::cerr<<"q_in: "<<qqdot.q.data.transpose()<<std::endl;
  // std::cerr<<"dq_in: "<<qqdot.qdot.data.transpose()<<std::endl;   
  // std::cerr<<"p1: "<<p1.x()<<" "<<p1.y()<<" "<<p1.z()<<std::endl;
  // std::cerr<<"p2: "<<p2.x()<<" "<<p2.y()<<" "<<p2.z()<<std::endl;
  // std::cerr<<"jacobian_a_:"<<std::endl<<jacobian_a_.data<<std::endl;
  // std::cerr<<"jacobian_b_:"<<std::endl<<jacobian_b_.data<<std::endl;
  // std::cerr<<"Jp2p1"<<std::endl<<Jp2p1.data<<std::endl;
  // std::cerr<<"Jdp2p1"<<std::endl<<Jdp2p1.data<<std::endl;  
  // std::cerr<<"d: "<<d(0)<<" "<<d(1)<<" "<<d(2)<<std::endl;    
  // std::cerr<<"d_dot: "<<d_dot.x()<<" "<<d_dot.y()<<" "<<d_dot.z()<<std::endl;
  // std::cerr<<"n: "<<n(0)<<" "<<n(1)<<" "<<n(2)<<std::endl;      
  // std::cerr<<"n_dot: "<<n_dot.x()<<" "<<n_dot.y()<<" "<<n_dot.z()<<std::endl;
  // std::cerr<<"e_: "<<e_.transpose()<<std::endl;
  // std::cerr<<"e_dot_: "<<e_dot_.transpose()<<std::endl;
  // std::cerr<<"J_: "<<std::endl<<J_<<std::endl;
  // std::cerr<<"J_dot_: "<<std::endl<<J_dot_<<std::endl; 
  // DEBUG END =====================================================
  
  return 0;
}

// template <>
// int TDefGeometricProjection<GeometricPoint, GeometricLine>::project(
//     std::shared_ptr<GeometricPoint> point,
//     std::shared_ptr<GeometricLine> line) {
//   KDL::Vector p__ = pose_a_.M * point->getPointKDL();
//   KDL::Vector p = pose_a_.p + p__;

//   KDL::Vector v = pose_b_.M * line->getDirectionKDL();

//   KDL::Vector d__ = pose_b_.M * line->getOffsetKDL();
//   KDL::Vector d = pose_b_.p + d__;

//   KDL::Vector x = p - d;
//   double s = KDL::dot(x, v);

//   e_(0) = KDL::dot(x, x) - s * s;

//   // The task jacobian is J = 2 (p-d)^T (I-vv^T) (Jp-Jd)

//   // As KDL does not provide a KDL::Matrix class, we use KDL::Rotation
//   // although K is not an actual rotation matrix in this context !
//   // K = (I - v v^T)
//   KDL::Rotation K = KDL::Rotation(KDL::Vector(1, 0, 0) - v * v(0),
//                                   KDL::Vector(0, 1, 0) - v * v(1),
//                                   KDL::Vector(0, 0, 1) - v * v(2));

//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jpd = -getRelativeVelocityJacobian(p__, d__, q_nr);
//     KDL::Vector y = K * Jpd;
//     J_(0, q_nr) = 2 * KDL::dot(x, y);
//   }
//   return 0;
// }

  template <>
  int TDefGeometricProjection<GeometricPoint, GeometricPlane>::project(
								       std::shared_ptr<GeometricPoint> point,
								       std::shared_ptr<GeometricPlane> plane,
								       const KDL::JntArrayVel& qqdot) {
  
    KDL::Vector p1__ = pose_a_.M * point->getPointKDL(); //point 1 from link origin to ee expressed in the world frame
    KDL::Vector p1 = pose_a_.p + p1__; //absolute ee point 1 expressed in the world frame

    KDL::Vector n = pose_b_.M * plane->getNormalKDL();  //plane normal expressed in the world frame
    KDL::Vector k__ = plane->getOffset() * n; //point on the plane expressed in the world frame
    KDL::Vector v__ = k__ + n; //normal tip expressed in the world frame
    KDL::Vector k = k__ + pose_b_.p; // absolute point on the plane expressed in the world frame    
    double b = dot(n,k); //plane offset in the world frame

    double q_nr=jacobian_a_.columns();
    KDL::Jacobian J_vk, J_p1k, J_dot_vk, J_dot_p1k, J_k, J_dot_k;
    J_k.resize(q_nr);
    J_dot_k.resize(q_nr);    
    J_vk.resize(q_nr);
    J_p1k.resize(q_nr);
    J_dot_vk.resize(q_nr);
    J_dot_p1k.resize(q_nr);

    changeJacRefPoint(jacobian_b_, v__, J_vk);
    changeJacRefPoint(jacobian_b_, k__, J_k);
    J_vk.data=J_vk.data - J_k.data;

    changeJacRefPoint(jacobian_a_, p1__, J_p1k);
    J_p1k.data=J_p1k.data - J_k.data;

    changeJacDotRefPoint(jacobian_b_, jacobian_dot_b_, qqdot, v__, J_dot_vk);
    changeJacDotRefPoint(jacobian_b_, jacobian_dot_b_, qqdot, k__, J_dot_k);    
    J_dot_vk.data = J_dot_vk.data - J_dot_k.data;

    changeJacDotRefPoint(jacobian_a_, jacobian_dot_a_, qqdot, p1__, J_dot_p1k);
    J_dot_p1k.data = J_dot_p1k.data - J_dot_k.data;	
	
    e_(0)=dot(n,p1)-b;
    J_=Eigen::Map<Eigen::Matrix<double,1,3> >((p1-k).data)*J_vk.data.topRows<3>()+Eigen::Map<Eigen::Matrix<double,1,3> >(n.data)*J_p1k.data.topRows<3>();

    e_dot_=J_*qqdot.qdot.data;
    J_dot_=Eigen::Map<Eigen::Matrix<double,1,3> >((p1-k).data)*J_dot_vk.data.topRows<3>()+Eigen::Map<Eigen::Matrix<double,1,3> >(n.data)*J_dot_p1k.data.topRows<3>()+(J_p1k.data.topRows<3>()*qqdot.qdot.data).transpose()*J_vk.data.topRows<3>()+(J_vk.data.topRows<3>()*qqdot.qdot.data).transpose()*J_p1k.data.topRows<3>();

    // DEBUG =========================================================
    // std::cerr<<"q_in: "<<qqdot.q.data.transpose()<<std::endl;
    // std::cerr<<"dq_in: "<<qqdot.qdot.data.transpose()<<std::endl;   
    // std::cerr<<"p1: "<<p1.x()<<" "<<p1.y()<<" "<<p1.z()<<std::endl;
    // std::cerr<<"jacobian_a_:"<<std::endl<<jacobian_a_.data<<std::endl;
    // std::cerr<<"jacobian_b_:"<<std::endl<<jacobian_b_.data<<std::endl;
    // std::cerr<<"b: "<<b<<std::endl;
    // std::cerr<<"n: "<<n(0)<<" "<<n(1)<<" "<<n(2)<<std::endl;
    // std::cerr<<"n_dot: "<<J_vk.data.topRows<3>()*qqdot.qdot.data<<std::endl;
    // std::cerr<<"e_: "<<e_.transpose()<<std::endl;
    // std::cerr<<"e_dot_: "<<e_dot_.transpose()<<std::endl;
    // std::cerr<<"J_: "<<std::endl<<J_<<std::endl;
    // std::cerr<<"J_dot_: "<<std::endl<<J_dot_<<std::endl<<std::endl; 
    // // // DEBUG END =====================================================

    return 0;
  }

// template <>
// int TDefGeometricProjection<GeometricPoint, GeometricBox>::project(
//     std::shared_ptr<GeometricPoint> point, std::shared_ptr<GeometricBox> box) {
//   KDL::Vector p__ = pose_a_.M * point->getPointKDL();
//   KDL::Vector p = pose_a_.p + p__;
//   KDL::Vector c__ = pose_b_.M * box->getCenterKDL();
//   KDL::Vector c = pose_b_.p + c__;

//   KDL::Rotation S =
//       box->getScalingKDL();  // from the world frame to a unit-box frame
//   KDL::Rotation Sinv = box->getScalingInvertedKDL();
//   KDL::Rotation R = pose_b_.M * box->getRotationKDL();

//   KDL::Vector x =
//       S * R.Inverse() *
//       (p - c);  // vector from c to p in axis-aligned unit-box coordinates

//   double f = absMax({x.x(), x.y(), x.z()});
//   double lambda = 1 / (2 * f);

//   KDL::Vector x_prim = lambda * x;  // the projected point p' on the box in
//                                     // axis-aligned unit-box coordinates
//   KDL::Vector x_prim__ = R * Sinv * x_prim;
//   KDL::Vector p_prim = x_prim__ + c;

//   // Used for debugging
//   // getGeometricPrimitiveMap()->updateGeometricPrimitive<GeometricPoint>("box_center",
//   // {c.x(), c.y(), c.z()});
//   // getGeometricPrimitiveMap()->updateGeometricPrimitive<GeometricPoint>("box_proj",
//   // {p_prim.x(), p_prim.y(), p_prim.z()});
//   // getGeometricPrimitiveMap()->updateGeometricPrimitive<GeometricLine>("box_line",
//   // {c.x()-p.x(), c.y()-p.y(), c.z()-p.z(),
//   //                                                                                  c.x(), c.y(), c.z()});

//   KDL::Vector d = p - p_prim;
//   e_(0) = KDL::dot(d, d);

//   // The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)
//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jp2p1 =
//         getRelativeVelocityJacobian(p__, x_prim__ + c__, q_nr);
//     J_(0, q_nr) = -2 * dot(d, Jp2p1);
//   }
//   return 0;
// }

// template <>
// int TDefGeometricProjection<GeometricPoint, GeometricCylinder>::project(
//     std::shared_ptr<GeometricPoint> point,
//     std::shared_ptr<GeometricCylinder> cylinder) {
//   KDL::Vector p__ = pose_a_.M * point->getPointKDL();
//   KDL::Vector p = pose_a_.p + p__;

//   KDL::Vector v = pose_b_.M * cylinder->getDirectionKDL();

//   KDL::Vector d__ = pose_b_.M * cylinder->getOffsetKDL();
//   KDL::Vector d = pose_b_.p + d__;

//   KDL::Vector x = p - d;
//   double s = KDL::dot(x, v);

//   e_(0) =
//       KDL::dot(x, x) - s * s - cylinder->getRadius() * cylinder->getRadius();

//   // The task jacobian is J = 2 (p-d)^T (I-vv^T) (Jp-Jd)

//   // As KDL does not provide a KDL::Matrix class, we use KDL::Rotation
//   // although K is not an actual rotation matrix in this context !
//   // K = (I - v v^T)
//   KDL::Rotation K = KDL::Rotation(KDL::Vector(1, 0, 0) - v * v(0),
//                                   KDL::Vector(0, 1, 0) - v * v(1),
//                                   KDL::Vector(0, 0, 1) - v * v(2));

//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jpd = -getRelativeVelocityJacobian(p__, d__, q_nr);
//     KDL::Vector y = K * Jpd;
//     J_(0, q_nr) = 2 * KDL::dot(x, y);
//   }
//   return 0;
// }

// template <>
// int TDefGeometricProjection<GeometricPoint, GeometricSphere>::project(
//     std::shared_ptr<GeometricPoint> point,
//     std::shared_ptr<GeometricSphere> sphere) {
//   KDL::Vector p1__ = pose_a_.M * point->getPointKDL();
//   KDL::Vector p1 = pose_a_.p + p1__;

//   KDL::Vector p2__ = pose_b_.M * sphere->getCenterKDL();
//   KDL::Vector p2 = pose_b_.p + p2__;

//   KDL::Vector d = p2 - p1;
//   e_(0) = KDL::dot(d, d) - sphere->getRadius() * sphere->getRadius();

//   // The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)
//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jp2p1 = getRelativeVelocityJacobian(p1__, p2__, q_nr);
//     J_(0, q_nr) = 2 * dot(d, Jp2p1);
//   }
//   return 0;
// }

// ///////////////////////////////////////////////////////////////////////////////
// //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// ///////////////////////////////////////////////////////////////////////////////
// //
// //                                 L I N E
// //
// ///////////////////////////////////////////////////////////////////////////////

// template <>
// int TDefGeometricProjection<GeometricLine, GeometricLine>::project(
//     std::shared_ptr<GeometricLine> line1,
//     std::shared_ptr<GeometricLine> line2) {
//   KDL::Vector v1 = pose_a_.M * line1->getDirectionKDL();
//   KDL::Vector d1__ = pose_a_.M * line1->getOffsetKDL();
//   KDL::Vector d1 = pose_a_.p + d1__;

//   KDL::Vector v2 = pose_b_.M * line2->getDirectionKDL();
//   KDL::Vector d2__ = pose_b_.M * line2->getOffsetKDL();
//   KDL::Vector d2 = pose_b_.p + d2__;

//   // Make a line, line3, that is perpendicular to both line1 and line2

//   // KDL::Vector v3 = v1 * v2; // v3 = v1 x v2 (cross product)

//   KDL::Rotation V = KDL::Rotation(v1, -v2, v1 * v2);

//   // s are the parameter values for line1, line2 and line3 indicating the points
//   // of intersection
//   KDL::Vector s = V.Inverse() * (d2 - d1);

//   KDL::Vector d3 =
//       v1 * s.data[0] +
//       d1;  // the point in world coordinates that lies on line1 and line3
//   KDL::Vector d3_proj =
//       v2 * s.data[1] +
//       d2;  // the point in world coordinates that lies on line2 and line3

//   KDL::Vector d3__ = d3 - pose_a_.p;
//   KDL::Vector d3_proj__ = d3_proj - pose_b_.p;

//   KDL::Vector d = d3_proj - d3;

//   e_(0) = KDL::dot(d, d);

//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jd3d3proj =
//         getRelativeVelocityJacobian(d3__, d3_proj__, q_nr);
//     J_(0, q_nr) = 2 * dot(d, Jd3d3proj);
//   }
//   return 0;
// }

// ///////////////////////////////////////////////////////////////////////////////
// //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// ///////////////////////////////////////////////////////////////////////////////
// //
// //                                 S P H E R E
// //
// ///////////////////////////////////////////////////////////////////////////////

// template <>
// int TDefGeometricProjection<GeometricSphere, GeometricPlane>::project(
//     std::shared_ptr<GeometricSphere> sphere,
//     std::shared_ptr<GeometricPlane> plane) {
//   KDL::Vector c__ = pose_a_.M * sphere->getCenterKDL();
//   KDL::Vector c = pose_a_.p + c__;

//   KDL::Vector n = pose_b_.M * plane->getNormalKDL();

//   KDL::Vector d__ = n * plane->getOffset();
//   KDL::Vector d = d__ + n * KDL::dot(n, pose_b_.p);

//   // double r = sphere->getRadius();
//   // double dist = KDL::dot(c, n) - plane->getOffset() - KDL::dot(n, pose_b_.p)
//   // - r;
//   // e_(0) = dist;

//   double r = sphere->getRadius();
//   double cd = KDL::dot(n, (c - d));

//   e_(0) = (cd < 0 ? cd + r : cd - r);

//   // The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)
//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jpd = -getRelativeVelocityJacobian(c__, d__, q_nr);
//     J_(0, q_nr) = KDL::dot(n, Jpd);
//   }
//   return 0;
// }

// template <>
// int TDefGeometricProjection<GeometricSphere, GeometricSphere>::project(
//     std::shared_ptr<GeometricSphere> sphere1,
//     std::shared_ptr<GeometricSphere> sphere2) {
//   KDL::Vector p1__ = pose_a_.M * sphere1->getCenterKDL();
//   KDL::Vector p1 = pose_a_.p + p1__;

//   KDL::Vector p2__ = pose_b_.M * sphere2->getCenterKDL();
//   KDL::Vector p2 = pose_b_.p + p2__;

//   KDL::Vector d = p2 - p1;
//   double r1 = sphere1->getRadius();
//   double r2 = sphere2->getRadius();
//   e_(0) = KDL::dot(d, d) - (r1 * r1 + 2 * r1 * r2 + r2 * r2);

//   // The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)
//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jp2p1 = getRelativeVelocityJacobian(p1__, p2__, q_nr);

//     J_(0, q_nr) = 2 * dot(d, Jp2p1);
//   }
//   return 0;
// }

// ///////////////////////////////////////////////////////////////////////////////
// //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// ///////////////////////////////////////////////////////////////////////////////
// //
// //                                 F R A M E
// //
// ///////////////////////////////////////////////////////////////////////////////

// template <>
// int TDefGeometricProjection<GeometricFrame, GeometricFrame>::project(
//     std::shared_ptr<GeometricFrame> frame1,
//     std::shared_ptr<GeometricFrame> frame2) {
//   KDL::Vector p1__ = pose_a_.M * frame1->getCenterKDL();
//   KDL::Vector p1 = pose_a_.p + p1__;

//   KDL::Vector p2__ = pose_b_.M * frame2->getCenterKDL();
//   KDL::Vector p2 = pose_b_.p + p2__;

//   KDL::Vector d = p2 - p1;
//   e_(0) = KDL::dot(d, d);

//   // The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)
//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jp2p1 = getRelativeVelocityJacobian(p1__, p2__, q_nr);
//     J_(0, q_nr) = 2 * dot(d, Jp2p1);
//   }

//   return 0;
// }
}  // namespace tasks

}  // namespace hiqp
