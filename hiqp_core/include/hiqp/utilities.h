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

#ifndef HIQP_UTILS_H
#define HIQP_UTILS_H

#include <iostream>

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>

#include <Eigen/Dense>

namespace hiqp {

std::ostream& operator<<(std::ostream& os, const KDL::Vector& kdl_vector);
std::ostream& operator<<(std::ostream& os, const KDL::Tree& kdl_tree);
std::ostream& operator<<(std::ostream& os, const KDL::FrameVel& kdl_frame_vel);
std::ostream& operator<<(std::ostream& os,
                         const KDL::JntArrayVel& kdl_joints_vel);
std::ostream& operator<<(std::ostream& os, const KDL::Chain& kdl_chain);

int kdl_getAllQNrFromTree(const KDL::Tree& kdl_tree,
                          std::vector<unsigned int>& qnrs);

std::string kdl_getJointNameFromQNr(const KDL::Tree& kdl_tree,
                                    unsigned int q_nr);

int kdl_getQNrFromJointName(const KDL::Tree& kdl_tree,
                            const std::string& joint_name);

int kdl_getQNrFromLinkName(const KDL::Tree& kdl_tree,
                           const std::string& link_name);

int kdl_JntToJac(const KDL::Tree& tree, const KDL::JntArrayVel& qqdot,
                 KDL::Jacobian& jac, const std::string& segmentname);

 /*! \brief computes the Jacobian time derivative for a tree with respect to the link given in segmentname
 *
 *Here, the i-th column of the velocity Jacobian derivative (the upper half) is given as
   \f{eqnarray*}{
      \dot{\mathbf{J}}^v_i &=& (\mathbf{\omega}_i \times \mathbf{k}_i) \times \mathbf{r}_i + \mathbf{k}_i \times (\mathbf{v}_n - \mathbf{v}_i), 
   \f}
   *
   * where \f$\mathbf{\omega}}_i\f$ denotes the angular velocity of link \f$i\f$ and \f$\mathbf{k}_i\f$ and \f$\mathbf{r}_i\f$ respectively are the corresponding joint axis unit vector and the vector from joint center \f$ i\f$ to the reference point. Additionally, \f$ \mathbf{v}_n\f$ and \f$ \mathbf{v}_i\f$ indicate the reference point velocity and the velocity of joint center \f$i\f$.
   *
   *Accordingly the i-th columnt of the angular velocity Jacobian derivative (the lower half) is given as
   \f{eqnarray*}{
      \dot{\mathbf{J}}^{\omega}_i &=& (\mathbf{\omega}_i \times \mathbf{k}_i), 
   \f}
   *
   *with \f$\mathbf{\omega}_i=\sum_{j=0}^i \mathbf{k}_j\dot{q}_j \f$
 */
 int treeJntToJacDot(const KDL::Tree& tree, const KDL::Jacobian& jac, const KDL::JntArrayVel& qqdot,
                 KDL::Jacobian& jac_dot, const std::string& segmentname);

void printHiqpInfo(const std::string& msg);
void printHiqpWarning(const std::string& msg);

/// \brief Returns the largest absolute value of all entries in v
double absMax(std::vector<double> v);

   /*! \brief changes the reference point of the jacobian jac. the new reference point p is relative to the reference point of jac and expressed in the world frame.
   */
  void changeJacRefPoint(const KDL::Jacobian& jac, const KDL::Vector& p, KDL::Jacobian& jac_new);
  /*! \brief changes the reference point of the jacobian derivative jac_dot of the jacobian jac. the new reference point p is relative to the reference point of jac and expressed in the world frame.
   */
  void changeJacDotRefPoint(const KDL::Jacobian& jac,
			    const KDL::Jacobian& jac_dot,
			    const KDL::JntArrayVel& qqdot,
                	    const KDL::Vector& p,
			    KDL::Jacobian& jac_dot_new);

  Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& vec);

/*! \brief Calculates the Moore-Penrose Pseudoinverse for any sized matrices.
 * The original source code is got from http://eigendobetter.com/, I edited it
 * to be compilable in this form.
 *  \author Marcus A Johansson
 *  \param a : the matrix to be inverted
 *  \return the inverted matrix */
template <typename Derived>
Derived pinv(const Eigen::MatrixBase<Derived>& a) {
  typedef typename Eigen::MatrixBase<Derived>::RealScalar RealScalar;
  if (a.rows() < a.cols()) {
    auto svd = a.derived().transpose().jacobiSvd(Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
    RealScalar tolerance =
        (RealScalar)std::numeric_limits<RealScalar>::epsilon() *
        std::max((RealScalar)a.cols(), (RealScalar)a.rows()) *
        svd.singularValues().array().abs().maxCoeff();
    return (svd.matrixV() *
            Derived((svd.singularValues().array().abs() > tolerance)
                        .select(svd.singularValues().array().inverse(), 0))
                .asDiagonal() *
            svd.matrixU().adjoint())
        .transpose();
  }
  Eigen::JacobiSVD<Derived> svd =
      a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  RealScalar tolerance =
      (RealScalar)std::numeric_limits<RealScalar>::epsilon() *
      std::max((RealScalar)a.cols(), (RealScalar)a.rows()) *
      svd.singularValues().array().abs().maxCoeff();
  return svd.matrixV() *
         Derived((svd.singularValues().array().abs() > tolerance)
                     .select(svd.singularValues().array().inverse(), 0))
             .asDiagonal() *
         svd.matrixU().adjoint();
}

/*! \brief calculates the Damped-Least-Square matrix.
 *  \author Marcus A Johansson
 *  \param a : the matrix to be inverted
 *  \return the damped-least-square matrix on success,
 *          the given matrix is returned otherwise. */
template <typename Derived>
Derived dls(const Eigen::MatrixBase<Derived>& a, double eta = 0.01) {
  unsigned int r = a.rows();
  unsigned int c = a.cols();

  if (r > c) return a;
  if (r == c) return pinv(a);

  Derived a_ext(r + c, c);
  a_ext.block(0, 0, r, c) = a;
  a_ext.block(r, 0, c, c) = eta * Eigen::MatrixXd::Identity(c, c);
  Derived b = Eigen::MatrixXd::Zero(r + c, r);
  b.block(0, 0, r, r) = Eigen::MatrixXd::Identity(r, r);

  return pinv(a_ext) * b;
}

}  // namespace hiqp

#endif
