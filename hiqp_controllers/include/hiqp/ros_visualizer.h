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




/*!
 * \file   ros_visualizer.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_ROS_VISUALIZER_H
#define HIQP_ROS_VISUALIZER_H


// HiQP Includes
#include <hiqp/visualizer.h>



// ROS Includes
#include <ros/ros.h>






namespace hiqp
{






class ROSVisualizer : public Visualizer
{
public:

	ROSVisualizer();

	~ROSVisualizer() noexcept {}

	int init(ros::NodeHandle* controller_nh);



	int add(GeometricPoint* point);
	int add(GeometricLine* line);
	int add(GeometricPlane* plane);
	int add(GeometricBox* box);
	int add(GeometricCylinder* cylinder);
	int add(GeometricSphere* sphere);

	void update(int id, GeometricPoint* point);
	void update(int id, GeometricLine* line);
	void update(int id, GeometricPlane* plane);
	void update(int id, GeometricBox* box);
	void update(int id, GeometricCylinder* cylinder);
	void update(int id, GeometricSphere* sphere);

	void remove(int id);

private:

	// No copying of this class is allowed !
	ROSVisualizer(const ROSVisualizer& other) = delete;
	ROSVisualizer(ROSVisualizer&& other) = delete;
	ROSVisualizer& operator=(const ROSVisualizer& other) = delete;
	ROSVisualizer& operator=(ROSVisualizer&& other) noexcept = delete;

	int apply(int id, GeometricPoint* point, int action);
	int apply(int id, GeometricLine* line, int action);
	int apply(int id, GeometricPlane* plane, int action);
	int apply(int id, GeometricBox* box, int action);
	int apply(int id, GeometricCylinder* cylinder, int action);
	int apply(int id, GeometricSphere* sphere, int action);

	const std::string 					kNamespace = "/yumi";
	const double 						kInfiniteLength = 3;
	const double 						kPointRadius    = 0.003;
	const double 						kLineRadius     = 0.003;
	const double 						kPlaneThickness = 0.001;

	ros::NodeHandle*                    controller_nh_;

	ros::Publisher 						marker_pub_;

	std::size_t 						next_id_;

};





} // namespace hiqp






#endif // include guard