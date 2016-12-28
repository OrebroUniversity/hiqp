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

namespace hiqp
{
namespace tasks
{

  /*! \brief A task definition that sets velocity and position limitations of a specific joint.
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

    std::string              link_frame_name_;
    std::size_t              link_frame_q_nr_;
    double                   jnt_upper_bound_;
    double                   jnt_lower_bound_;

  }; 

} // namespace tasks

} // namespace hiqp

#endif // include guard