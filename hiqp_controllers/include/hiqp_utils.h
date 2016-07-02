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
    /*
template<typename _Matrix_Type_>
    bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
    {
      if(a.rows()<a.cols())
          return false;

      Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

      typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

  result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
      array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
  }
*/

// Code got from http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
// Originally written by Cyrille Berger, 2011
// Edited by Marcus A Johansson, 2016
template<typename T>
int pinvSVD
(
    const T& a,
    T& result,
    double epsilon = std::numeric_limits<typename T::Scalar>::epsilon()
)
{
    if (a.rows() != a.cols())
        return -1;

    Eigen::JacobiSVD< T > svd = 
        a.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

    typename T::Scalar tolerance = 
        epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

    result = svd.matrixV() * T( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
        array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();

    return 0;
}





template<typename T>
int dampedLeastSquare
(
    const T& a, 
    T& result, 
    double eta = 0.01
)
{
    unsigned int r = a.rows();
    unsigned int c = a.cols();

    if (r > c) return -1;

    if (r == c)
    {
        pinvSVD(a, result);
        return 0;
    }

    unsigned int d = c-r;

    Eigen::MatrixXd a_ext(c, c);

    a_ext.block(0, 0, r, c) = a;
    
    a_ext.block(r, 0, d, d) = eta * Eigen::MatrixXd::Identity(d, d);

    Eigen::MatrixXd a_ext_inv;
    a_ext_inv.resizeLike(a_ext);
    pinvSVD(a_ext, a_ext_inv);

    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(r+d, r);
    b.block(0, 0, r, r) = Eigen::MatrixXd::Identity(r, r);
    result = a_ext_inv * b;

    return 0;
}










} // namespace hiqp






#endif