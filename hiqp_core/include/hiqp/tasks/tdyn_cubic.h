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

#ifndef HIQP_TDYN_CUBIC_H
#define HIQP_TDYN_CUBIC_H

#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>

namespace hiqp {
namespace tasks {

/*! \brief A cubic dynamics similar to first-order dynamics but with slower
 * convergence around e=0.
 *  \author Marcus A Johansson */
class TDynCubic : public TaskDynamics {
 public:
  TDynCubic(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
            std::shared_ptr<Visualizer> visualizer)
      : TaskDynamics(geom_prim_map, visualizer) {}

  ~TDynCubic() noexcept {}

  int init(const std::vector<std::string>& parameters,
           RobotStatePtr robot_state, const Eigen::VectorXd& e_initial,
           const Eigen::VectorXd& e_final);

  int update(RobotStatePtr robot_state, const Eigen::VectorXd& e,
             const Eigen::MatrixXd& J);

  int monitor();

 private:
  TDynCubic(const TDynCubic& other) = delete;
  TDynCubic(TDynCubic&& other) = delete;
  TDynCubic& operator=(const TDynCubic& other) = delete;
  TDynCubic& operator=(TDynCubic&& other) noexcept = delete;

  double lambda_;
};

}  // namespace tasks

}  // namespace hiqp

#endif  // include guard
