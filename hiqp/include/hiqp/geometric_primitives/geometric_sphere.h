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

/*
 * \file   geometric_sphere.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_GEOMETRIC_SPHERE_H
#define HIQP_GEOMETRIC_SPHERE_H

// HiQP Includes
#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/hiqp_utils.h>

// Orocos KDL Includes
#include <kdl/frames.hpp>

// Eigen Includes
#include <Eigen/Dense>





namespace hiqp
{
namespace geometric_primitives
{

/*!
 * \class GeometricSphere
 * \brief Parameters: [x, y, z, radius]
 */ 
class GeometricSphere : public GeometricPrimitive
{
public:

  GeometricSphere
  (
    const std::string& name,
    const std::string& frame_id,
    bool visible,
    const std::vector<double>& color
  )
  : GeometricPrimitive(name, frame_id, visible, color)
  {}

  ~GeometricSphere() noexcept {}

  /*!
   * \brief Parses a set of parameters and initializes the sphere.
   *
   * \param parameters should be of size 4. <br />
   *   Indices 0-2 (required) defines the position of the center of the sphere, <br />
   *   index 3 (required) defines the radius of the sphere.
   *
   * \return 0 on success, -1 if the wrong number of parameters was sent
  */
  int init(const std::vector<double>& parameters)
  {
    int size = parameters.size();
    if (size != 4)
    {
      printHiqpWarning("GeometricSphere requires 4 parameters, got " 
        + std::to_string(size) + "! Initialization failed!");
      return -1;
    }

    kdl_p_(0) = parameters.at(0);
    kdl_p_(1) = parameters.at(1);
    kdl_p_(2) = parameters.at(2);
    radius_ = parameters.at(3);

    eigen_p_ << kdl_p_(0), kdl_p_(1), kdl_p_(2);

    return 0;
  }

  inline const KDL::Vector&     getCenterKDL() { return kdl_p_; }

  inline const Eigen::Vector3d& getCenterEigen() { return eigen_p_; }

  inline double getRadius() { return radius_; }

  inline double getX() { return kdl_p_(0); }

  inline double getY() { return kdl_p_(1); }

  inline double getZ() { return kdl_p_(2); }

protected:

  KDL::Vector      kdl_p_; // the offset of the sphere
  Eigen::Vector3d  eigen_p_;

  double           radius_; // the radius of the sphere



private:

  // No copying of this class is allowed !
  GeometricSphere(const GeometricSphere& other) = delete;
  GeometricSphere(GeometricSphere&& other) = delete;
  GeometricSphere& operator=(const GeometricSphere& other) = delete;
  GeometricSphere& operator=(GeometricSphere&& other) noexcept = delete;

}; // class GeometricSphere

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard