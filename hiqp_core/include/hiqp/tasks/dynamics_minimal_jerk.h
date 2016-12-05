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

#ifndef HIQP_DYNAMICS_MINIMAL_JERK_H
#define HIQP_DYNAMICS_MINIMAL_JERK_H

#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>

namespace hiqp
{
namespace tasks
{

  /*! \brief A task dynamics that enforces minimal jerk throughout the whole motion.
   *  \author Marcus A Johansson */  
  class DynamicsMinimalJerk : public TaskDynamics {
  public:
    DynamicsMinimalJerk(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                         std::shared_ptr<Visualizer> visualizer)
    : TaskDynamics(geom_prim_map, visualizer) {}

    ~DynamicsMinimalJerk() noexcept = default;

    int init(const std::vector<std::string>& parameters,
             RobotStatePtr robot_state,
             const Eigen::VectorXd& e_initial,
             const Eigen::VectorXd& e_final);

    int update(RobotStatePtr robot_state,
               const Eigen::VectorXd& e,
               const Eigen::MatrixXd& J);

    int monitor();

  private:
    DynamicsMinimalJerk(const DynamicsMinimalJerk& other) = delete;
    DynamicsMinimalJerk(DynamicsMinimalJerk&& other) = delete;
    DynamicsMinimalJerk& operator=(const DynamicsMinimalJerk& other) = delete;
    DynamicsMinimalJerk& operator=(DynamicsMinimalJerk&& other) noexcept = delete;

    HiQPTimePoint                time_start_;
    double                       total_duration_;
    double                       gain_;
    Eigen::VectorXd              e_initial_;
    Eigen::VectorXd              e_final_;
    Eigen::VectorXd              e_diff_;
    double                       f_;
  };

} // namespace tasks

} // namespace hiqp

#endif // include guard