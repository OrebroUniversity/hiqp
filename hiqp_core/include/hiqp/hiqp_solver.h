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

#include <map>
#include <vector>
#include <iostream>

#include <Eigen/Dense>

namespace hiqp
{

  /*! \brief A stage is a compound set of tasks with the same priority level.
   *  \author Marcus A Johansson */
  struct HiQPStage {
    int nRows;
    Eigen::VectorXd e_dot_star_;
    Eigen::MatrixXd J_;
    std::vector<int> constraint_signs_;
  };

  /*! \brief The base class for a solver for controls from a set of stages. Keeps an internal set of stages that tasks can be appended to.
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

    /*! \brief Appends the internal set of stages with a task. If a stage with the priority is not currently present in the stages map, it is created, otherwise the task is appended to that existing stage. */
    int appendStage(std::size_t priority_level, 
                    const Eigen::VectorXd& e_dot_star,
                    const Eigen::MatrixXd& J,
                    const std::vector<int>& constraint_signs) {
      StageMap::iterator it = stages_map_.find(priority_level);

      if (it == stages_map_.end()) {
        HiQPStage stage;
        stage.e_dot_star_ = e_dot_star;
        stage.J_ = J;
        stage.constraint_signs_ = constraint_signs;
        stage.nRows = e_dot_star.rows();
        stages_map_.emplace(priority_level, stage);
      } else {
        int rows = it->second.e_dot_star_.rows() + e_dot_star.rows();
        Eigen::VectorXd edotstar__(rows);
        edotstar__ << it->second.e_dot_star_, e_dot_star;
        it->second.e_dot_star_.resize(rows);
        it->second.e_dot_star_ = edotstar__;
        Eigen::MatrixXd J__(rows, it->second.J_.cols());
        J__ << it->second.J_, J;
        it->second.J_ = J__;
        it->second.constraint_signs_.insert(it->second.constraint_signs_.begin(),
                                            constraint_signs.begin(),
                                            constraint_signs.end() );
        it->second.nRows += e_dot_star.rows();
      }

      return 0;
    }

  protected:
    typedef std::map<std::size_t, HiQPStage> StageMap;
    StageMap    stages_map_; 

  private:
    HiQPSolver(const HiQPSolver& other) = delete;
    HiQPSolver(HiQPSolver&& other) = delete;
    HiQPSolver& operator=(const HiQPSolver& other) = delete;
    HiQPSolver& operator=(HiQPSolver&& other) noexcept = delete;
  };

} // namespace hiqp

#endif // include guard
