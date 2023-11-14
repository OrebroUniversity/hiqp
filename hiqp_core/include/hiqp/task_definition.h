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

#ifndef HIQP_TASK_DEFINITION_H
#define HIQP_TASK_DEFINITION_H

#include <iostream>
#include <memory>
#include <vector>

#include <hiqp/geometric_primitives/geometric_primitive_map.h>
#include <hiqp/robot_state.h>
#include <hiqp/visualizer.h>

#include <Eigen/Dense>

namespace hiqp {

  using geometric_primitives::GeometricPrimitiveMap;

  class Task;
  namespace tasks{
    class TDefMetaTask;
  }
  /*! \brief Defines the task space and the performance value of a task.
   *  \author Marcus A Johansson */
  class TaskDefinition {
  public:
    inline TaskDefinition() {} // std::cerr<<"Definition base constructor (SHOULD NOT APPEAR?)\n"; }
    TaskDefinition(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
		 std::shared_ptr<Visualizer> visualizer)
    : geometric_primitive_map_(geom_prim_map), visualizer_(visualizer) {} // std::cerr<<"Definition base parametrized constructor (SHOULD NOT APPEAR?)\n"; }

    virtual ~TaskDefinition() noexcept {} // std::cerr<<"Definition base destructor (SHOULD NOT APPEAR?)\n"; }

    inline void initializeTaskDefinition(
					 std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
					 std::shared_ptr<Visualizer> visualizer) {
      geometric_primitive_map_ = geom_prim_map;
      visualizer_ = visualizer;
    }

    virtual int init(const std::vector<std::string>& parameters,
		     RobotStatePtr robot_state) = 0;

    virtual int update(RobotStatePtr robot_state) = 0;

    virtual int monitor() = 0;

    unsigned int getDimensions() { return n_dimensions_; }
    Eigen::VectorXd getInitialTaskValue() { return e_initial_; }
    Eigen::MatrixXd getInitialJacobian() { return J_initial_; }
    Eigen::VectorXd getInitialTaskDerivative() { return e_dot_initial_; }
    Eigen::MatrixXd getInitialJacobianDerivative() { return J_dot_initial_; }
    Eigen::VectorXd getInitialExogeneousTaskQuantities() { return f_initial_; }        
    Eigen::VectorXd getExogeneousTaskQuantities() { return f_; }    
    Eigen::VectorXd getTaskValue() { return e_; }
    Eigen::MatrixXd getJacobian() { return J_; }
    Eigen::VectorXd getTaskDerivative() { return e_dot_; }
    Eigen::MatrixXd getJacobianDerivative() { return J_dot_; }

    virtual Eigen::VectorXd getFinalTaskValue(RobotStatePtr robot_state) {
      return Eigen::VectorXd::Zero(e_.rows());
    }
    virtual Eigen::VectorXd getFinalTaskDerivative(RobotStatePtr robot_state) {
      return Eigen::VectorXd::Zero(e_dot_.rows());
    }
    inline unsigned int getPriority() { return priority_; }

  protected:
	
    Eigen::VectorXd e_;            ///< the task function value e(q)
    Eigen::VectorXd e_dot_;        ///< the task function derivative (e_dot(q)=J(q)*q_dot
    Eigen::VectorXd e_prev_;            ///< the task function value e(q) at t-1
    Eigen::VectorXd e_dot_prev_;        ///< the task function derivative (e_dot(q)=J(q)*q_dot at t-1
    Eigen::MatrixXd J_;            ///< the task jacobian J(q)
    Eigen::MatrixXd J_dot_;        ///< the task jacobian derivative J_dot(q)
    std::vector<int> task_signs_;  ///< -1 leq, 0 eq, 1 geq
    Eigen::VectorXd f_;            ///< exogeneous task terms not dependent on q, e.g., forces        
    Eigen::VectorXd performance_measures_;
    unsigned int n_dimensions_;

    inline std::string getTaskName() { return task_name_; }
    inline bool getActive() { return active_; }
    inline bool getVisible() { return visible_; }
    inline std::shared_ptr<Visualizer> getVisualizer() { return visualizer_; }
    inline std::shared_ptr<GeometricPrimitiveMap> getGeometricPrimitiveMap() {
      return geometric_primitive_map_;
    }
    inline void low_pass(double alpha_=0.01) {
	e_ = (1-alpha_)*e_prev_ + alpha_*e_;
	e_dot_ = (1-alpha_)*e_dot_prev_ + alpha_*e_dot_;

	e_prev_=e_;
	e_dot_prev_=e_dot_;
    }

  private:
    friend Task;
    //  friend tasks::TDefMetaTask;

    std::shared_ptr<GeometricPrimitiveMap> geometric_primitive_map_;
    std::shared_ptr<Visualizer> visualizer_;

    Eigen::VectorXd e_initial_;
    Eigen::MatrixXd J_initial_;
    Eigen::VectorXd e_dot_initial_;
    Eigen::MatrixXd J_dot_initial_;
    Eigen::VectorXd f_initial_;
    
    std::string task_name_;
    unsigned int priority_;
    bool visible_;
    bool active_;

    TaskDefinition(const TaskDefinition& other) = delete;
    TaskDefinition(TaskDefinition&& other) = delete;
    TaskDefinition& operator=(const TaskDefinition& other) = delete;
    TaskDefinition& operator=(TaskDefinition&& other) noexcept = delete;

    /*! \brief Calls update() of the child class, and properly sets up the
     * TaskDefinition object. */
    int initialize(const std::vector<std::string>& parameters,
		   RobotStatePtr robot_state) {
      if (init(parameters, robot_state) != 0) return -1;

      update(robot_state);
      e_initial_ = e_;
      J_initial_ = J_;
      e_dot_initial_ = e_dot_;
      J_dot_initial_ = J_dot_;
      f_initial_ = f_;

      return 0;
    }
  };

  typedef std::shared_ptr<TaskDefinition> TaskDefinitionPtr;
}  // namespace hiqp

#endif  // include guard
