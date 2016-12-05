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

#ifndef HIQP_DYNAMICS_JNT_LIMITS_H
#define HIQP_DYNAMICS_JNT_LIMITS_H

#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>

namespace hiqp
{
namespace tasks
{

  /*! \brief A special task dynamics to be used only with the TaskJntLimits task definition.
   *  \author Marcus A Johansson */  
  class DynamicsJntLimits : public TaskDynamics {
  public:
    DynamicsJntLimits(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                      std::shared_ptr<Visualizer> visualizer)
    : TaskDynamics(geom_prim_map, visualizer) {}

    ~DynamicsJntLimits() noexcept = default;

    int init(const std::vector<std::string>& parameters,
             RobotStatePtr robot_state,
             const Eigen::VectorXd& e_initial,
             const Eigen::VectorXd& e_final);

    int update(RobotStatePtr robot_state,
               const Eigen::VectorXd& e,
               const Eigen::MatrixXd& J);

    int monitor();

  private:
    DynamicsJntLimits(const DynamicsJntLimits& other) = delete;
    DynamicsJntLimits(DynamicsJntLimits&& other) = delete;
    DynamicsJntLimits& operator=(const DynamicsJntLimits& other) = delete;
    DynamicsJntLimits& operator=(DynamicsJntLimits&& other) noexcept = delete;

    double             dq_max_;

  };

} // namespace tasks

} // namespace hiqp

#endif // include guard