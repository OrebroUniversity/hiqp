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

#ifndef HIQP_TASK_DYNAMICS_H
#define HIQP_TASK_DYNAMICS_H

#include <vector>
#include <memory>

#include <hiqp/geometric_primitives/geometric_primitive_map.h>
#include <hiqp/robot_state.h>
#include <hiqp/visualizer.h>

#include <Eigen/Dense>

namespace hiqp
{

  using geometric_primitives::GeometricPrimitiveMap;

  class Task;

  /*! \brief A task dynamics enforces the optimizer to produce controls that results is a certain velocity of the task performance value.
   *  \author Marcus A Johansson */ 
  class TaskDynamics
  {
  public:
    TaskDynamics(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                 std::shared_ptr<Visualizer> visualizer)
     : geometric_primitive_map_(geom_prim_map), visualizer_(visualizer) 
    {}

    ~TaskDynamics() noexcept {}

    virtual int init(const std::vector<std::string>& parameters,
                     RobotStatePtr robot_state,
                     const Eigen::VectorXd& e_initial,
                     const Eigen::VectorXd& e_final) = 0;

    virtual int update(RobotStatePtr robot_state,
                       const Eigen::VectorXd& e,
                       const Eigen::MatrixXd& J) = 0;

    virtual int monitor() = 0;

  protected:
      Eigen::VectorXd          e_dot_star_;
      Eigen::VectorXd          performance_measures_;

      inline std::string  getTaskName()                      { return task_name_; }
      inline unsigned int getPriority()                      { return priority_; }
      inline bool         getActive()                        { return active_; }
      inline bool         getVisible()                       { return visible_; }
      inline std::shared_ptr<Visualizer> 
                          getVisualizer()                    { return visualizer_; }
      inline std::shared_ptr<GeometricPrimitiveMap> 
                          getGeometricPrimitiveMap()         { return geometric_primitive_map_; }

  private:
    friend         Task;

    std::shared_ptr<GeometricPrimitiveMap>    geometric_primitive_map_;
    std::shared_ptr<Visualizer>               visualizer_;

    std::string                               task_name_;
    unsigned int                              priority_;
    bool                                      visible_;
    bool                                      active_;

    TaskDynamics(const TaskDynamics& other) = delete;
    TaskDynamics(TaskDynamics&& other) = delete;
    TaskDynamics& operator=(const TaskDynamics& other) = delete;
    TaskDynamics& operator=(TaskDynamics&& other) noexcept = delete;
  };

} // namespace hiqp

#endif // include guard