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

#ifndef HIQP_GEOMETRIC_POINT_H
#define HIQP_GEOMETRIC_POINT_H

#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/utilities.h>

#include <kdl/frames.hpp>

#include <Eigen/Dense>

namespace hiqp
{
namespace geometric_primitives
{

  /*! \brief Parameters: [x, y, z]
   *  \author Marcus A Johansson */ 
  class GeometricPoint : public GeometricPrimitive {
  public:
    GeometricPoint(const std::string& name,
                   const std::string& frame_id,
                   bool visible,
                   const std::vector<double>& color)
    : GeometricPrimitive(name, frame_id, visible, color) {}
    
    ~GeometricPoint() noexcept {}

    /*! \brief Parses a set of parameters and initializes the point.
     *  \param parameters : Should be of size 3. Indices 0-2 (required) defines the position of the point.
     * \return 0 on success, -1 if the wrong number of parameters was sent */
    int init(const std::vector<double>& parameters) {
      int size = parameters.size();
      if (size != 3) {
        printHiqpWarning("GeometricPoint requires 3 parameters, got " 
          + std::to_string(size) + "! Initialization failed!");
        return -1;
      }

      kdl_p_(0) = parameters.at(0);
      kdl_p_(1) = parameters.at(1);
      kdl_p_(2) = parameters.at(2);

      eigen_p_ << kdl_p_(0), kdl_p_(1), kdl_p_(2);
      return 0;
    }

    inline const KDL::Vector&       getPointKDL()   { return kdl_p_; }

    inline const Eigen::Vector3d&   getPointEigen() { return eigen_p_; }

    inline double getX() { return kdl_p_(0); }
    inline double getY() { return kdl_p_(1); }
    inline double getZ() { return kdl_p_(2); }

  protected:
    KDL::Vector        kdl_p_;
    Eigen::Vector3d    eigen_p_;

  private:
    GeometricPoint(const GeometricPoint& other) = delete;
    GeometricPoint(GeometricPoint&& other) = delete;
    GeometricPoint& operator=(const GeometricPoint& other) = delete;
    GeometricPoint& operator=(GeometricPoint&& other) noexcept = delete;
  };

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard