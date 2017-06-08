// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2017 Jens Lundell
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

#include <hiqp/tasks/tdef_meta_task.h>
#include <iostream>

namespace hiqp {
namespace tasks {

TDefMetaTask::TDefMetaTask(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                           std::shared_ptr<Visualizer> visualizer)
    : TaskDefinition(geom_prim_map, visualizer) {}

int TDefMetaTask::init(const std::vector<std::string>& parameters,
                       RobotStatePtr robot_state) {
  int size = parameters.size();
  unsigned int n_controls = robot_state->getNumControls();
  unsigned int n_joints = robot_state->getNumJoints();
  unsigned int j = 1;
  unsigned int num_tasks = 0;

  e_.resize(1);
  J_.resize(1, n_joints);
  task_types_.clear();

  while (j < size) {
    std::shared_ptr<TaskDefinition> def_;
    std::string type = parameters.at(j);
    std::cout << type << std::endl;
    if (type.compare("TDefGeomProj") == 0) {
      std::string prim_type1 = parameters.at(j + 1);
      std::string prim_type2 = parameters.at(j + 2);
      if (prim_type1.compare("point") == 0 &&
          prim_type2.compare("point") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricPoint, GeometricPoint> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("point") == 0 &&
                 prim_type2.compare("line") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricPoint, GeometricLine> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("point") == 0 &&
                 prim_type2.compare("plane") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricPoint, GeometricPlane> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("point") == 0 &&
                 prim_type2.compare("box") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricPoint, GeometricBox> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("point") == 0 &&
                 prim_type2.compare("cylinder") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricPoint, GeometricCylinder> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("point") == 0 &&
                 prim_type2.compare("sphere") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricPoint, GeometricSphere> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("line") == 0 &&
                 prim_type2.compare("line") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricLine, GeometricLine> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("sphere") == 0 &&
                 prim_type2.compare("plane") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricSphere, GeometricPlane> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("sphere") == 0 &&
                 prim_type2.compare("sphere") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricSphere, GeometricSphere> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("frame") == 0 &&
                 prim_type2.compare("frame") == 0) {
        def_ = std::make_shared<
            TDefGeometricProjection<GeometricFrame, GeometricFrame> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else {
        printHiqpWarning(
            "TDefGeomProj does not support primitive combination of types '" +
            prim_type1 + "' and '" + prim_type2 + "'!");
        return -1;
      }
    } else if (type.compare("TDefGeomAlign") == 0) {
      std::string prim_type1 = parameters.at(j + 1);
      std::string prim_type2 = parameters.at(j + 2);
      if (prim_type1.compare("line") == 0 && prim_type2.compare("line") == 0) {
        def_ = std::make_shared<
            TDefGeometricAlignment<GeometricLine, GeometricLine> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("line") == 0 &&
                 prim_type2.compare("plane") == 0) {
        def_ = std::make_shared<
            TDefGeometricAlignment<GeometricLine, GeometricPlane> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("line") == 0 &&
                 prim_type2.compare("cylinder") == 0) {
        def_ = std::make_shared<
            TDefGeometricAlignment<GeometricLine, GeometricCylinder> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("line") == 0 &&
                 prim_type2.compare("sphere") == 0) {
        def_ = std::make_shared<
            TDefGeometricAlignment<GeometricLine, GeometricSphere> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else if (prim_type1.compare("frame") == 0 &&
                 prim_type2.compare("frame") == 0) {
        def_ = std::make_shared<
            TDefGeometricAlignment<GeometricFrame, GeometricFrame> >(
            this->getGeometricPrimitiveMap(), this->getVisualizer());
      } else {
        printHiqpWarning(
            "TDefGeomAlign does not support primitive combination of types '" +
            prim_type1 + "' and '" + prim_type2 + "'!");
        return -1;
      }
    }
    if (def_->init(std::vector<std::string>(parameters.begin() + j,
                                            parameters.begin() + j + 4),
                   robot_state) != 0) {
      def_.reset();
      return -5;
    }
    if (j > 1) {
      J_.conservativeResize(J_.rows() + 1, J_.cols());
      e_.conservativeResize(e_.size() + 1);
    }

    J_.row(num_tasks) = def_->J_;
    e_(num_tasks) = def_->e_(0);
    task_types_.push_back(def_->task_types_[0]);
    task_defs_.push_back(def_);
    num_tasks++;
    j += 4;
  }

  return 0;
}

int TDefMetaTask::update(RobotStatePtr robot_state) {
  unsigned int i = 0;
  for (auto& task : task_defs_) {
    if (task->update(robot_state) != 0) {
      printHiqpWarning("Could not update the subtask " + task->getTaskName() +
                       " in TdefMetaTask!");
      return -1;
    }
    e_(i) = task->e_(0);
    J_.row(i) = task->J_;
    i++;
  }
  return 0;
}

int TDefMetaTask::monitor() { return 0; }

}  // namespace tasks

}  // namespace hiqp
