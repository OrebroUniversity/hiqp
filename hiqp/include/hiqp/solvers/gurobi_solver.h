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

#ifndef HIQP_GUROBI_SOLVER_H
#define HIQP_GUROBI_SOLVER_H

#include <hiqp/hiqp_solver.h>
#include <gurobi_c++.h>

namespace hiqp
{
  /*! \brief An optimization based solver for a set of stages implemented in CasADi.
   *  \author Robert Krug, Marcus A Johansson */
  class GurobiSolver : public HiQPSolver {
  public:
    GurobiSolver();
    ~GurobiSolver() noexcept {}

    bool solve(std::vector<double>& solution);

  private:
    GurobiSolver(const GurobiSolver& other) = delete;
    GurobiSolver(GurobiSolver&& other) = delete;
    GurobiSolver& operator=(const GurobiSolver& other) = delete;
    GurobiSolver& operator=(GurobiSolver&& other) noexcept = delete;

    void reset();

    GRBEnv             env_;
    Eigen::VectorXd    x_; // HQP solution (updated sequentially when solving)
    Eigen::VectorXd    w_; // slack variables (updated sequentially when solving)
    Eigen::VectorXd    b_;
    Eigen::MatrixXd    A_;
    std::vector<char>  senses_;
  };

} // namespace hiqp

#endif // include guard




