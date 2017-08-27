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

#ifndef HIQP_TDEF_TRACKING_H
#define HIQP_TDEF_TRACKING_H

#include <string>
#include <vector>

#include <hiqp/robot_state.h>
#include <hiqp/task_definition.h>
#include <ros/ros.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

namespace hiqp {
  namespace tasks {

    /*! \brief Tracking geometric primitives.
     *  \author Robert Krug */
    template <typename PrimitiveA, typename PrimitiveB>
      class TDefTracking : public TaskDefinition {
    public:
      TDefTracking(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
		   std::shared_ptr<Visualizer> visualizer);
      ~TDefTracking() noexcept = default;

      int init(const std::vector<std::string>& parameters,
	       RobotStatePtr robot_state);

      int update(RobotStatePtr robot_state);

      int monitor();
      
    protected:
      std::shared_ptr<PrimitiveA> primitive_a_;
      std::shared_ptr<PrimitiveB> primitive_b_;
	   
    private:
      TDefTracking(const TDefTracking& other) = delete;
      TDefTracking(TDefTracking&& other) = delete;
      TDefTracking& operator=(const TDefTracking& other) =
	delete;
      TDefTracking& operator=(TDefTracking&& other) noexcept =
	delete;

      int alignUnitVectors(const KDL::Vector& v1, const KDL::Vector v2, const RobotStatePtr robot_state, Eigen::VectorXd& e, Eigen::VectorXd& e_dot, Eigen::MatrixXd& J, Eigen::MatrixXd& J_dot);

      int track(std::shared_ptr<PrimitiveA> first,
		std::shared_ptr<PrimitiveB> second,
		const RobotStatePtr robot_state);

      /// \brief This sets jacobian columns corresponding to non-writable joints to
      /// 0
      void maskJacobian(RobotStatePtr robot_state);
      void maskJacobianDerivative(RobotStatePtr robot_state);  
    
      std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_pos_;
      std::shared_ptr<KDL::TreeJntToJacSolver> fk_solver_jac_;

      KDL::Frame pose_a_;
      KDL::Jacobian jacobian_a_; ///< tree jacobian w.r.t. the center of the frame TDefTracking::pose_a_ 
      KDL::Jacobian jacobian_dot_a_;  
 
      KDL::Frame pose_b_; 
      KDL::Jacobian jacobian_b_; ///< tree jacobian w.r.t. the center of the frame TDefTracking::pose_b_
      KDL::Jacobian jacobian_dot_b_;

      double d_max_;
      double phi_max_;
      
      /* ros::NodeHandle nh_; */
      /*   ros::Publisher marker_pub_; */
  
    };

  }  // namespace tasks

}  // namespace hiqp

#include <hiqp/tasks/tdef_tracking__impl.h>

#endif  // include guard
