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

#ifndef HIQP_TDEF_FTS_PROJECTION_H
#define HIQP_TDEF_FTS_PROJECTION_H

#include <string>
#include <vector>

#include <hiqp/robot_state.h>
#include <hiqp/tasks/tdef_geometric_projection.h>

/* #include <kdl/treefksolverpos_recursive.hpp> */
/* #include <kdl/treejnttojacsolver.hpp> */

namespace hiqp {
  namespace tasks {

    /*! \brief A task definition that projects task errors, Jacobians, their derivatives as well as given 6D forces into the operational space of the task
     *  \author Robert Krug */
    template <typename PrimitiveA, typename PrimitiveB>
      class TDefFTSProjection : public TDefGeometricProjection<PrimitiveA, PrimitiveB> {
    public:
      TDefFTSProjection(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                          std::shared_ptr<Visualizer> visualizer);
      ~TDefFTSProjection() noexcept = default;

      int init(const std::vector<std::string>& parameters,
      	       RobotStatePtr robot_state);

      int update(RobotStatePtr robot_state);

      int monitor();

    private:

      int projectForces(std::shared_ptr<PrimitiveA> first,
		  std::shared_ptr<PrimitiveB> second,
		  const RobotStatePtr robot_state);

    };

  }  // namespace tasks

}  // namespace hiqp

#include <hiqp/tasks/tdef_fts_projection__impl.h>

#endif  // include guard
