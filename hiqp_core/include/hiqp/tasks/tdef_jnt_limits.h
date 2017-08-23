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

#ifndef HIQP_TDEF_JNT_LIMITS_H
#define HIQP_TDEF_JNT_LIMITS_H

#include <string>

#include <hiqp/robot_state.h>
#include <hiqp/task_definition.h>

namespace hiqp {
namespace tasks {
  

/*! \brief A task definition that sets velocity and position limitations of a
 * specific joint. Designed to be used in conjunction with TDynJntLimits dynamics, avoidance for joint \f$i\f$ is implemented as
 *
   \f{eqnarray*}{
       -\ddot{q}_i & \geq & -K_p(q_{ub_{i}}-q_i)+K_d\dot{q}_i,\\
       -\ddot{q}_i & \leq & -K_p(q_{lb_{i}}-q_i)+K_d\dot{q}_i, \\
       \ddot{q}_i & \leq & \frac{1}{\Delta t}(\dot{q}_{max_{i}}-\dot{q}_i),\\
       \ddot{q}_i & \geq & \frac{1}{\Delta t}(-\dot{q}_{max_{i}}-\dot{q}_i),
   \f}
   *
   *where \f$q_{ub_i}\f$ and \f$q_{lb_i}\f$ are upper and lower joint limit, \f$\dot{q}_{max}\f$ is the (positive) joint velocity maximum and \f$\Delta t\f$ the controller period.
 *  \author Marcus A Johansson */
class TDefJntLimits : public TaskDefinition {
 public:
  TDefJntLimits(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                std::shared_ptr<Visualizer> visualizer)
      : TaskDefinition(geom_prim_map, visualizer) {}

  ~TDefJntLimits() noexcept = default;

  int init(const std::vector<std::string>& parameters,
           RobotStatePtr robot_state);

  int update(RobotStatePtr robot_state);

  int monitor();

 private:
  TDefJntLimits(const TDefJntLimits& other) = delete;
  TDefJntLimits(TDefJntLimits&& other) = delete;
  TDefJntLimits& operator=(const TDefJntLimits& other) = delete;
  TDefJntLimits& operator=(TDefJntLimits&& other) noexcept = delete;

  std::string link_frame_name_;
  std::size_t link_frame_q_nr_;
  double q_lb_;
  double q_ub_;
  double dq_max_;
};

}  // namespace tasks

}  // namespace hiqp

#endif  // include guard
