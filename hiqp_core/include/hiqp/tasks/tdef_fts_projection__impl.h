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

#ifndef HIQP_TDEF_FORCE_PROJECTION__IMPL_H
#define HIQP_TDEF_FORCE_PROJECTION__IMPL_H

#include <iterator>
#include <sstream>

#include <hiqp/utilities.h>

namespace hiqp {
  namespace tasks {

    template <typename PrimitiveA, typename PrimitiveB>
      TDefFTSProjection<PrimitiveA, PrimitiveB>::TDefFTSProjection(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map, std::shared_ptr<Visualizer> visualizer)
      : TDefGeometricProjection<PrimitiveA, PrimitiveB>(geom_prim_map, visualizer) {}

    template <typename PrimitiveA, typename PrimitiveB>
      int TDefFTSProjection<PrimitiveA, PrimitiveB>::init(const std::vector<std::string>& parameters, RobotStatePtr robot_state) {
      int parameters_size = parameters.size();
      /* if (parameters_size != 4) { */
      /* 	printHiqpWarning( */
      /* 			 "'" + getTaskName() + "': TDefGeomProj takes 4 parameters, got " + */
      /* 			 std::to_string(parameters_size) + "! The task was not added!"); */
      /* 	return -1; */
      /* } */

      //initialize the base class
      int retval=TDefGeometricProjection<PrimitiveA, PrimitiveB>::init(parameters, robot_state);
      if(retval != 0)
	return retval;

      return 0;
    }

    template <typename PrimitiveA, typename PrimitiveB>
      int TDefFTSProjection<PrimitiveA, PrimitiveB>::update(RobotStatePtr robot_state) {
    
      //update the base class
      int retval=TDefGeometricProjection<PrimitiveA, PrimitiveB>::update(robot_state);
      if(retval != 0)
	return retval;
      
      projectForces(this->primitive_a_, this->primitive_b_,robot_state);
      return 0;
    }

    template <typename PrimitiveA, typename PrimitiveB>
      int TDefFTSProjection<PrimitiveA, PrimitiveB>::monitor() {
      return 0;
    }
  }  // namespace tasks

}  // namespace hiqp

#endif  // include guard
