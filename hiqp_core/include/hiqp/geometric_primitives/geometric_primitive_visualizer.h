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

#ifndef HIQP_GEOMETRIC_PRIMITIVE_VISUALIZER_H
#define HIQP_GEOMETRIC_PRIMITIVE_VISUALIZER_H

#include <memory>

#include <hiqp/geometric_primitives/geometric_primitive_visitor.h>
#include <hiqp/visualizer.h>

namespace hiqp {

namespace geometric_primitives {

  /*! \brief A Geometric Primitive Visitor that sends the primitive it visits to a visualizer.
   *  \author Marcus A Johansson */
  class GeometricPrimitiveVisualizer : public GeometricPrimitiveVisitor {
  public:
    GeometricPrimitiveVisualizer(Visualizer* visualizer)
    : visualizer_(visualizer)
    {}

    ~GeometricPrimitiveVisualizer() noexcept {}

    void visit(std::shared_ptr<GeometricPoint> point) const;
    void visit(std::shared_ptr<GeometricLine> line) const;
    void visit(std::shared_ptr<GeometricPlane> plane) const;
    void visit(std::shared_ptr<GeometricBox> box) const;
    void visit(std::shared_ptr<GeometricCylinder> cylinder) const;
    void visit(std::shared_ptr<GeometricSphere> sphere) const;
    void visit(std::shared_ptr<GeometricFrame> frame) const;

  private:
    GeometricPrimitiveVisualizer(const GeometricPrimitiveVisualizer& other) = delete;
    GeometricPrimitiveVisualizer(GeometricPrimitiveVisualizer&& other) = delete;
    GeometricPrimitiveVisualizer& operator=(const GeometricPrimitiveVisualizer& other) = delete;
    GeometricPrimitiveVisualizer& operator=(GeometricPrimitiveVisualizer&& other) noexcept = delete;

    Visualizer* visualizer_;
  };

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard