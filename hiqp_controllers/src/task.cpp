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

#include <hiqp/task.h>
#include <hiqp/tasks/task_full_pose.h>
#include <hiqp/tasks/dynamics_first_order.h>
#include <hiqp/hiqp_utils.h>

namespace hiqp {

  using tasks::TaskFullPose;
  using tasks::DynamicsFirstOrder;

  Task::Task(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
             std::shared_ptr<Visualizer> visualizer,
             int n_controls)
  : geom_prim_map_(geom_prim_map), visualizer_(visualizer), n_controls_(n_controls)
  {}

  int Task::init(const std::vector<std::string>& def_params,
                 const std::vector<std::string>& dyn_params,
                 RobotStatePtr robot_state)
  {
    if (def_params.size() <= 0) {
      printHiqpWarning("No (zero) task definition parameters found!");
      return -1;
    }
    if (dyn_params.size() <= 0) {
      printHiqpWarning("No (zero) task dynamics parameters found!");
      return -2;
    }

    if (constructDefinition(def_params) != 0) return -3;
    if (constructDynamics(dyn_params) != 0) return -4;

    def_->task_name_ = task_name_;
    def_->priority_ = priority_;
    def_->active_ = active_;
    def_->visible_ = visible_;

    dyn_->task_name_ = task_name_;
    dyn_->priority_ = priority_;
    dyn_->active_ = active_;
    dyn_->visible_ = visible_;

    def_->initialize(def_params, robot_state, n_controls_);

    dyn_->init(dyn_params, robot_state, def_->getInitialValue(), def_->getFinalValue(robot_state));

    return 0;
  }

  void Task::update(RobotStatePtr robot_state)
  {
    if (!def_ || !dyn_) return;
    if (def_->update(robot_state) != 0) return;
    dyn_->update(robot_state, def_->e_, def_->J_);
  }

  int Task::constructDefinition(const std::vector<std::string>& def_params)
  {
    std::string type = def_params.at(0);

    if (type.compare("TDefFullPose") == 0) {
      def_ = std::make_shared<TaskFullPose>(geom_prim_map_, visualizer_);
    } else {
      printHiqpWarning("The task definition type name '" + type + "' was not understood!");
      return -1;
    }

    return 0;
  }

  int Task::constructDynamics(const std::vector<std::string>& dyn_params)
  {
    std::string type = dyn_params.at(0);

    if (type.compare("TDynFirstOrder") == 0) {
      dyn_ = std::make_shared<DynamicsFirstOrder>(geom_prim_map_, visualizer_);
    } else {
      printHiqpWarning("The task dynamics type name '" + type + "' was not understood!");
      return -1;
    }

    return 0;
  }





} // namespace hiqp