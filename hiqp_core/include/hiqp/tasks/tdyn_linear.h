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

#ifndef HIQP_TDYN_LINEAR_H
#define HIQP_TDYN_LINEAR_H

#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>

namespace hiqp
{
namespace tasks
{

  /*! \brief A general first-order task dynamics implementation that enforces an exponential decay of the task performance value.
   *  \author Marcus A Johansson */  
  class TDynLinear : public TaskDynamics {
  public:
    TDynLinear(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                       std::shared_ptr<Visualizer> visualizer)
     : TaskDynamics(geom_prim_map, visualizer) {}

    ~TDynLinear() noexcept {}

    int init(const std::vector<std::string>& parameters, 
	     RobotStatePtr robot_state, 
	     const Eigen::VectorXd& e_initial, 
	     const Eigen::VectorXd& e_dot_initial, 
	     const Eigen::VectorXd& e_final, 
	     const Eigen::VectorXd& e_dot_final);

    int update(const RobotStatePtr robot_state, 
	       const TaskDefinitionPtr def);

    int monitor();

  private:
    TDynLinear(const TDynLinear& other) = delete;
    TDynLinear(TDynLinear&& other) = delete;
    TDynLinear& operator=(const TDynLinear& other) = delete;
    TDynLinear& operator=(TDynLinear&& other) noexcept = delete;

    double lambda_;
  };

} // namespace tasks

} // namespace hiqp

#endif // include guard
