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

#ifndef HIQP_TDYN_RANDOM_H
#define HIQP_TDYN_RANDOM_H

#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>

namespace hiqp
{
namespace tasks
{

  /*! \brief A general first-order task dynamics implementation that enforces an exponential decay of the task performance value.
   *  \author Marcus A Johansson */  
  class TDynRandom : public TaskDynamics {
  public:
    TDynRandom(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                       std::shared_ptr<Visualizer> visualizer)
     : TaskDynamics(geom_prim_map, visualizer) {}

    ~TDynRandom() noexcept {}

    int init(const std::vector<std::string>& parameters,
             RobotStatePtr robot_state,
             const Eigen::VectorXd& e_initial,
             const Eigen::VectorXd& e_final);

    int update(RobotStatePtr robot_state,
               const Eigen::VectorXd& e,
               const Eigen::MatrixXd& J);

    int monitor();

  private:
    TDynRandom(const TDynRandom& other) = delete;
    TDynRandom(TDynRandom&& other) = delete;
    TDynRandom& operator=(const TDynRandom& other) = delete;
    TDynRandom& operator=(TDynRandom&& other) noexcept = delete;

    std::default_random_engine generator;
    std::normal_distribution<double> dist;
  };

} // namespace tasks

} // namespace hiqp

#endif // include guard