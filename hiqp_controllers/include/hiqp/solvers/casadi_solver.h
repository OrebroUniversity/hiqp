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
 * \file   casadi_solver.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_CASADI_SOLVER_H
#define HIQP_CASADI_SOLVER_H

// HiQP Includes
#include <hiqp/hiqp_solver.h>





namespace hiqp
{

class CasADiSolver : public HiQPSolver
{
public:

  CasADiSolver() {}

  ~CasADiSolver() noexcept {}

  bool solve(std::vector<double>& solution);



private:
  // No copying of this class is allowed !
  CasADiSolver(const CasADiSolver& other) = delete;
  CasADiSolver(CasADiSolver&& other) = delete;
  CasADiSolver& operator=(const CasADiSolver& other) = delete;
  CasADiSolver& operator=(CasADiSolver&& other) noexcept = delete;

}; // class CasADiSolver

} // namespace hiqp

#endif // include guard




