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

/*
 * \file   Gurobi_solver.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_GUROBI_SOLVER_H
#define HIQP_GUROBI_SOLVER_H

// HiQP Includes
#include <hiqp/hiqp_solver.h>
#include <gurobi_c++.h>




namespace hiqp
{
//--------------------------------------------------------------
#define TIKHONOV_FACTOR  1e-3
#define PRESOLVE         -1
#define OPTIMALITY_TOL   1e-6
#define SCALE_FLAG       1
#define TIME_LIMIT       1.0//0.005
#define OUTPUT_FLAG      0
#define DUAL_REDUCTIONS  1
//--------------------------------------------------------------
class GurobiSolver : public HiQPSolver
{
public:

  GurobiSolver(); 

  ~GurobiSolver() noexcept {}

  bool solve(std::vector<double>& solution);



private:
  // No copying of this class is allowed !
  GurobiSolver(const GurobiSolver& other) = delete;
  GurobiSolver(GurobiSolver&& other) = delete;
  GurobiSolver& operator=(const GurobiSolver& other) = delete;
  GurobiSolver& operator=(GurobiSolver&& other) noexcept = delete;
  GRBEnv env_;
    Eigen::VectorXd x_; ///<HQP solution (updated sequentially when solving)
    Eigen::VectorXd w_; ///<slack variables (updated sequentially when solving)
    Eigen::VectorXd b_;
    Eigen::MatrixXd A_;
    std::vector<char> senses_;

 void reset();
}; // class GurobiSolver

} // namespace hiqp

#endif // include guard




