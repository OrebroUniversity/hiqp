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

#include <hiqp/geometric_primitives/geometric_primitive_visualizer.h>

namespace hiqp
{
namespace geometric_primitives
{

  void GeometricPrimitiveVisualizer::visit
  (
    std::shared_ptr<GeometricPoint> point
  ) const
  {
    int id = point->getVisualId();
    if (id < 0) {
      point->setVisualId(visualizer_->add(point));
    } else {
      visualizer_->update(id, point);
    }
  }

  void GeometricPrimitiveVisualizer::visit
  (
    std::shared_ptr<GeometricLine> line
  ) const
  {
    int id = line->getVisualId();
    if (id < 0) {
      line->setVisualId(visualizer_->add(line));
    } else {
      visualizer_->update(id, line);
    }
  }

  void GeometricPrimitiveVisualizer::visit
  (
    std::shared_ptr<GeometricPlane> plane
  ) const
  {
    int id = plane->getVisualId();
    if (id < 0) {
      plane->setVisualId(visualizer_->add(plane));
    } else {
      visualizer_->update(id, plane);
    }
  }

  void GeometricPrimitiveVisualizer::visit
  (
    std::shared_ptr<GeometricBox> box
  ) const
  {
    int id = box->getVisualId();
    if (id < 0) {
      box->setVisualId(visualizer_->add(box));
    } else {
      visualizer_->update(id, box);
    }
  }

  void GeometricPrimitiveVisualizer::visit
  (
    std::shared_ptr<GeometricCylinder> cylinder
  ) const
  {
    int id = cylinder->getVisualId();
    if (id < 0) {
      cylinder->setVisualId(visualizer_->add(cylinder));
    } else {
      visualizer_->update(id, cylinder);
    }
  }

  void GeometricPrimitiveVisualizer::visit
  (
    std::shared_ptr<GeometricSphere> sphere
  ) const
  {
    int id = sphere->getVisualId();
    if (id < 0) {
      sphere->setVisualId(visualizer_->add(sphere));
    } else {
      visualizer_->update(id, sphere);
    }
  }

  void GeometricPrimitiveVisualizer::visit
  (
    std::shared_ptr<GeometricFrame> frame
  ) const
  {
    int id = frame->getVisualId();
    if (id < 0) {
      frame->setVisualId(visualizer_->add(frame));
    } else {
      visualizer_->update(id, frame);
    }
  }

} // snamespace geometric_primitives

} // namespace hiqp