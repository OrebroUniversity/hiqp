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

#ifndef HIQP_VISUALIZER_H
#define HIQP_VISUALIZER_H

#include <memory>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>
#include <hiqp/geometric_primitives/geometric_frame.h>

namespace hiqp
{
	using geometric_primitives::GeometricPoint;
	using geometric_primitives::GeometricLine;
	using geometric_primitives::GeometricPlane;
	using geometric_primitives::GeometricBox;
	using geometric_primitives::GeometricCylinder;
	using geometric_primitives::GeometricSphere;
	using geometric_primitives::GeometricFrame;

	/*! \brief An interface for visualizing geometric primitives. Derive from this class to implement your own visualizer and provide it to TaskManager to get visualization of HiQP.
	 *  \author Marcus A Johansson */
	class Visualizer {
	public:
		Visualizer() {}
		~Visualizer() noexcept {}

		virtual int add(std::shared_ptr<GeometricPoint> point) = 0;
		virtual int add(std::shared_ptr<GeometricLine> line) = 0;
		virtual int add(std::shared_ptr<GeometricPlane> plane) = 0;
		virtual int add(std::shared_ptr<GeometricBox> box) = 0;
		virtual int add(std::shared_ptr<GeometricCylinder> cylinder) = 0;
		virtual int add(std::shared_ptr<GeometricSphere> sphere) = 0;
		virtual int add(std::shared_ptr<GeometricFrame> frame) = 0;

		virtual void update(int id, std::shared_ptr<GeometricPoint> point) = 0;
		virtual void update(int id, std::shared_ptr<GeometricLine> line) = 0;
		virtual void update(int id, std::shared_ptr<GeometricPlane> plane) = 0;
		virtual void update(int id, std::shared_ptr<GeometricBox> box) = 0;
		virtual void update(int id, std::shared_ptr<GeometricCylinder> cylinder) = 0;
		virtual void update(int id, std::shared_ptr<GeometricSphere> sphere) = 0;
		virtual void update(int id, std::shared_ptr<GeometricFrame> frame) = 0;

		virtual void remove(int id) = 0;

		virtual void removeMany(const std::vector<int>& ids) = 0;

	private:
		Visualizer(const Visualizer& other) = delete;
		Visualizer(Visualizer&& other) = delete;
		Visualizer& operator=(const Visualizer& other) = delete;
		Visualizer& operator=(Visualizer&& other) noexcept = delete;
	};

} // namespace hiqp

#endif // include guard