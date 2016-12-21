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

#ifndef HIQP_DYNAMICS_HYPER_SIN_H
#define HIQP_DYNAMICS_HYPER_SIN_H

#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>

namespace hiqp
{
namespace tasks
{

  /*! \brief A general hyperbolic sin task dynamics. Useful for only influencing the manipulator behaviour for task function values within a certain span.
   *  \author Marcus A Johansson */  
  class DynamicsHyperSin : public TaskDynamics
  {
  public:

    DynamicsHyperSin(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                       std::shared_ptr<Visualizer> visualizer)
     : TaskDynamics(geom_prim_map, visualizer) {}

    ~DynamicsHyperSin() noexcept {}

    int init(const std::vector<std::string>& parameters,
             RobotStatePtr robot_state,
             const Eigen::VectorXd& e_initial,
             const Eigen::VectorXd& e_final);

    int update(RobotStatePtr robot_state,
               const Eigen::VectorXd& e,
               const Eigen::MatrixXd& J);

    int monitor();

  private:
    DynamicsHyperSin(const DynamicsHyperSin& other) = delete;
    DynamicsHyperSin(DynamicsHyperSin&& other) = delete;
    DynamicsHyperSin& operator=(const DynamicsHyperSin& other) = delete;
    DynamicsHyperSin& operator=(DynamicsHyperSin&& other) noexcept = delete;

    double lambda_;
  };

} // namespace tasks

} // namespace hiqp

#endif // include guard