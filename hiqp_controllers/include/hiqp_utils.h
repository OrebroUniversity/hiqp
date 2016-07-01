#ifndef HIQP_UTILS_H
#define HIQP_UTILS_H

// STL Includes
#include <iostream>

// Orocos KDL Includes
#include <kdl/tree.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jacobian.hpp>


// Eigen Includes
#include <Eigen/Dense>





namespace hiqp
{






std::ostream& operator<<(std::ostream& os, const KDL::Tree& kdl_tree);
std::ostream& operator<<(std::ostream& os, const KDL::FrameVel& kdl_frame_vel);
std::ostream& operator<<(std::ostream& os, const KDL::JntArrayVel& kdl_joints_vel);
std::ostream& operator<<(std::ostream& os, const KDL::Chain& kdl_chain);





unsigned int kdl_getQNrFromJointName
(
  const KDL::Tree& kdl_tree, 
  const std::string& joint_name
);


int kdl_JntToJac
(
  const KDL::Tree& tree,
  const KDL::JntArrayVel& qqdot, 
  KDL::Jacobian& jac, 
  const std::string& segmentname
);







// Code got from http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
// Written by Cyrille Berger, 2011
template<typename _Matrix_Type_>
bool pseudoInverse
(
  const _Matrix_Type_ &a, 
	_Matrix_Type_ &result, 
	double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon()
)
{
  if (a.rows() < a.cols())
      return false;

  Eigen::JacobiSVD< _Matrix_Type_ > svd = 
  	a.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

  typename _Matrix_Type_::Scalar tolerance = 
  	epsilon * std::max(a.cols(), a.rows()) * 
  	svd.singularValues().array().abs().maxCoeff();
  
  result = svd.matrixV() * _Matrix_Type_( 
  	(svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
      array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
}







} // namespace hiqp






#endif