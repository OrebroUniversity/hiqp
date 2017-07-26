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

#ifndef HIQP_HIQP_SOLVER_H
#define HIQP_HIQP_SOLVER_H

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>
#include <kdl/jntarray.hpp>

namespace hiqp {

/*! \brief A stage is a compound set of tasks with the same priority level.
 *  \author Marcus A Johansson */
struct HiQPStage {
  int nRows;
  Eigen::VectorXd b_;
  Eigen::MatrixXd B_;
  std::vector<int> constraint_signs_;
};

/*! \brief The base class for a solver for controls from a set of stages. Keeps
 * an internal set of stages that tasks can be appended to.
 *
 * For \f$p=1,\ldots,P\f$ stages, HiQP sequentially solves the QP below with decision variables \f$\mathbf{\omega}_p\f$ and \f$\ddot{\mathbf{q}}\f$ 
 *
   \f{eqnarray*}{
       \mathrm{min} && \frac{1}{2}(||\mathbf{\omega}_p||^2+\mu||\ddot{\mathbf{q}}||^2)\\
          \mathrm{s. t.} && \mathbf{B}_i\ddot{\mathbf{q}} \geq \mathbf{b}_i+\mathbf{\omega}_i^*, \quad i=1,\ldots,p-1 \\
                         && \mathbf{B}_p\ddot{\mathbf{q}} \geq  \mathbf{b}_p+\mathbf{\omega}_p,
   \f}
   *
   * where the slack variables \f$\mathbf{\omega}}_i^*\f$ of preceding solutions are frozen. In the QP above \f$\mu \in \mathbb{R}_+\f$ is a small regularization factor, \f$\mathbf{B}_i=[\mathbf{J}_{i,1} \ldots \mathbf{J}_{i,N}]^T\f$ contains the \f$N\f$ stacked task jacobians for all tasks in the \f$i\f$-th stage and \f$\mathbf{b}_i=[\ddot{\mathbf{e}}^*_{i,1}-\dot{\mathbf{J}}_{i,1}\dot{\mathbf{q}} \ldots \ddot{\mathbf{e}}^*_{i,N}-\dot{\mathbf{J}}_{i,N}\dot{\mathbf{q}}]^T\f$ is the corresponding desired task acceleration adjusted with a joint velocity dependent term. The final solution for \f$\ddot{\mathbf{q}}\f$ is obtained after the \f$P\f$-th step. 
   *
 *  \author Marcus A Johansson */
class HiQPSolver {
 public:
  HiQPSolver() {}
  ~HiQPSolver() noexcept {}

  virtual bool solve(std::vector<double>& solution) = 0;

  int clearStages() {
    stages_map_.clear();
    return 0;
  }

  /*! \brief Appends the internal set of stages with a task. If a stage with the
   * priority is not currently present in the stages map, it is created,
   * otherwise the task is appended to that existing stage. */
  int appendStage(std::size_t priority_level,
		  const Eigen::VectorXd& e_ddot_star,
                  const Eigen::MatrixXd& J,
                  const Eigen::MatrixXd& J_dot,
		  const KDL::JntArray& q_dot,
                  const std::vector<int>& constraint_signs) {
    
      // DEBUG =============================================
    /* std::cerr<<"HiQPSolver::appendStage inputs: "<<std::endl; */
    /* std::cerr<<"e_ddot_star: "<<e_ddot_star.transpose()<<std::endl; */
    /* std::cerr<<"J: "<<std::endl<<J<<std::endl; */
    /* std::cerr<<"J_dot: "<<std::endl<<J_dot<<std::endl; */
    /* std::cerr<<"q_dot: "<<std::endl<<q_dot.data.transpose()<<std::endl; */
    /* std::cerr<<"constraint_signs: "; */
    /* for (int i=0; i<constraint_signs.size(); i++){ */
    /* 	std::cerr<<constraint_signs[i]<<" "; */
    /*   } */
    /* 	std::cerr<<std::endl<<std::endl; */
      // DEBUG END ==========================================
	   
    int dim=e_ddot_star.rows();
    assert((J.rows()==dim) && (J_dot.rows()==dim) && (constraint_signs.size()==dim));
    
    StageMap::iterator it = stages_map_.find(priority_level);

    if (it == stages_map_.end()) {
      HiQPStage stage;
      stage.b_ = e_ddot_star-J_dot*q_dot.data;
      stage.B_ = J;
      stage.constraint_signs_ = constraint_signs;
      stage.nRows = dim;
      stages_map_.emplace(priority_level, stage);
      // DEBUG =============================================
      /* std::cerr<<std::setprecision(2)<<"append new stage: "<<std::endl; */
      /* std::cerr<<"J_t: "<<std::endl<<stage.B_<<std::endl; */
      /* std::cerr<<"signs: "; */
      /* for (unsigned int k=0;k<stage.constraint_signs_.size();k++) */
      /*   std::cerr<<stage.constraint_signs_[k]<<" "; */

      /* std::cerr<<std::endl<<"b:
       * "<<stage.b_.transpose()<<std::endl; */
      // DEBUG END ==========================================
    } else {
      int rows = it->second.b_.rows() + dim;
      // DEBUG =============================================
      /* std::cerr<<std::setprecision(2)<<"HiQPSolver::appendStage - before
       * appending existing stage: "<<std::endl; */
      /* std::cerr<<"J_t: "<<std::endl<<it->second.B_<<std::endl; */
      /* std::cerr<<"signs: "; */
      /* for (unsigned int k=0;k<it->second.constraint_signs_.size();k++) */
      /*  std::cerr<<it->second.constraint_signs_[k]<<" "; */

      /* std::cerr<<std::endl<<"b:
       * "<<it->second.b_.transpose()<<std::endl;  */
      // DEBUG END ==========================================

      Eigen::VectorXd b__(rows);
      b__ << it->second.b_, e_ddot_star-J_dot*q_dot.data;
      it->second.b_ = b__;
      Eigen::MatrixXd B__(rows, it->second.B_.cols());
      B__ << it->second.B_, J;
      it->second.B_ = B__;
      it->second.constraint_signs_.insert(it->second.constraint_signs_.end(),
                                          constraint_signs.begin(),
                                          constraint_signs.end());
      it->second.nRows += dim;
      // DEBUG =============================================
      /* std::cerr<<std::setprecision(2)<<"HiQPSolver::appendStage - after
       * appending existing stage: "<<std::endl; */
      /* std::cerr<<"J_t: "<<std::endl<<it->second.B_<<std::endl; */
      /* std::cerr<<"signs: "; */
      /* for (unsigned int k=0;k<it->second.constraint_signs_.size();k++) */
      /*  std::cerr<<it->second.constraint_signs_[k]<<" "; */

      /* std::cerr<<std::endl<<"b:
       * "<<it->second.b_.transpose()<<std::endl;  */
      // DEBUG END ==========================================
    }

    return 0;
  }

 protected:
  typedef std::map<std::size_t, HiQPStage> StageMap;
  StageMap stages_map_;

 private:
  HiQPSolver(const HiQPSolver& other) = delete;
  HiQPSolver(HiQPSolver&& other) = delete;
  HiQPSolver& operator=(const HiQPSolver& other) = delete;
  HiQPSolver& operator=(HiQPSolver&& other) noexcept = delete;
};

}  // namespace hiqp

#endif  // include guard
