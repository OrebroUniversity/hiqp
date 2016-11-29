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

#ifndef HIQP_GEOMETRIC_FRAME_H
#define HIQP_GEOMETRIC_FRAME_H

#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/hiqp_utils.h>

#include <kdl/frames.hpp>

#include <Eigen/Dense>

namespace hiqp
{
namespace geometric_primitives
{

  /*! \brief Parameters: [x, y, z, qw, qx, qy, qz]
   *  \author Marcus A Johansson */ 
  class GeometricFrame : public GeometricPrimitive {
  public:
    GeometricFrame(const std::string& name,
     const std::string& frame_id,
     bool visible,
     const std::vector<double>& color)
    : GeometricPrimitive(name, frame_id, visible, color) {}

    ~GeometricFrame() noexcept {}

    int init(const std::vector<double>& parameters) {
      int size = parameters.size();
      if (size != 7) {
        printHiqpWarning("GeometricFrame requires 7 parameters, got " 
          + std::to_string(size) + "! Initialization failed!");
        return -1;
      }

      kdl_c_(0) = parameters.at(0);
      kdl_c_(1) = parameters.at(1);
      kdl_c_(2) = parameters.at(2);

      eigen_c_ << kdl_c_(0), kdl_c_(1), kdl_c_(2);

      double w = parameters.at(3);
      double x = parameters.at(4);
      double y = parameters.at(5);
      double z = parameters.at(6);

      q_ = Eigen::Quaternion<double>(w, x, y, z);

      return 0;
    }

    inline const KDL::Vector& getCenterKDL() { return kdl_c_; }
    inline const Eigen::Vector3d& getCenterEigen() { return eigen_c_; }
    inline const Eigen::Quaternion<double>& getQuaternionEigen() { return q_; }

    inline double getX() { return kdl_c_(0); }
    inline double getY() { return kdl_c_(1); }
    inline double getZ() { return kdl_c_(2); }

    inline double getQW() { return q_.w(); }
    inline double getQX() { return q_.x(); }
    inline double getQY() { return q_.y(); }
    inline double getQZ() { return q_.z(); }

  protected:
    KDL::Vector        kdl_c_;
    Eigen::Vector3d    eigen_c_;

    Eigen::Quaternion<double> q_;

  private:
    GeometricFrame(const GeometricFrame& other) = delete;
    GeometricFrame(GeometricFrame&& other) = delete;
    GeometricFrame& operator=(const GeometricFrame& other) = delete;
    GeometricFrame& operator=(GeometricFrame&& other) noexcept = delete;

}; // class Geometric Point

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard