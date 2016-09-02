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
 * \file   task_geometric_projection.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/tasks/task_geometric_projection.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <hiqp/hiqp_utils.h>

// Orocos KDL Includes
//#include <kdl/treefksolverpos_recursive.hpp>
//#include <kdl/treejnttojacsolver.hpp>

#include <iostream>
#include <string>
#include <sstream>






namespace hiqp
{










//#############################################################################
//
//                                 P O I N T
//
//#############################################################################






template<>
int TaskGeometricProjection<GeometricPoint, GeometricPoint>::project
(
	GeometricPoint* point1, 
	GeometricPoint* point2
)
{
	KDL::Vector p1__ = pose_a_.M * point1->getPointKDL();
	KDL::Vector p1 = pose_a_.p + p1__;

	KDL::Vector p2__ = pose_b_.M * point2->getPointKDL();
	KDL::Vector p2 = pose_b_.p + p2__;

	KDL::Vector d = p2 - p1;
	e_(0) = KDL::dot(d, d);

	// The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jp2p1 = getVelocityJacobianForTwoPoints(p1__, p2__, q_nr);

		J_(0, q_nr) = 2 * dot(d, Jp2p1);
	}
	
	return 0;
}





template<>
int TaskGeometricProjection<GeometricPoint, GeometricLine>::project
(
	GeometricPoint* point, 
	GeometricLine* line
)
{
	KDL::Vector p__ = pose_a_.M * point->getPointKDL();
	KDL::Vector p = pose_a_.p + p__;

	KDL::Vector v = pose_b_.M * line->getDirectionKDL();

	KDL::Vector d__ = pose_b_.M * line->getOffsetKDL();
	KDL::Vector d = pose_b_.p + d__;

	KDL::Vector x = p - d;
	double s = KDL::dot(x, v);

	e_(0) = KDL::dot(x, x) - s*s;

	// The task jacobian is J = 2 (p-d)^T (I-vv^T) (Jp-Jd)

	// As KDL does not provide a KDL::Matrix class, we use KDL::Rotation
	// although K is not an actual rotation matrix in this context !
	// K = (I - v v^T)
	KDL::Rotation K = KDL::Rotation(KDL::Vector(1,0,0) - v*v(0), 
		                            KDL::Vector(0,1,0) - v*v(1), 
		                            KDL::Vector(0,0,1) - v*v(2));

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jpd = - getVelocityJacobianForTwoPoints(p__, d__, q_nr);

		KDL::Vector y = K * Jpd;

		J_(0, q_nr) = 2 * KDL::dot(x, y);
	}
	
	return 0;
}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricPlane>::project
(
	GeometricPoint* point, 
	GeometricPlane* plane
)
{
	KDL::Vector p__ = pose_a_.M * point->getPointKDL();
	KDL::Vector p = pose_a_.p + p__;

	KDL::Vector n = pose_b_.M * plane->getNormalKDL();

	KDL::Vector d__ = n * plane->getOffset();
	KDL::Vector d = d__ + n * KDL::dot(n, pose_b_.p);

	e_(0) = KDL::dot(n, (p-d));

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jpd = - getVelocityJacobianForTwoPoints(p__, d__, q_nr);

		J_(0, q_nr) = KDL::dot(n, Jpd);
	}
	
	return 0;
}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricBox>::project
(
	GeometricPoint* point, 
	GeometricBox* box
)
{

	KDL::Vector p__ = pose_a_.M * point->getPointKDL();
	KDL::Vector p = pose_a_.p + p__; 

	KDL::Rotation S = KDL::Rotation(
		box->getDimX(), 0, 0,
		0, box->getDimY(), 0,
		0, 0, box->getDimZ()
	);

	KDL::Rotation Sinv = KDL::Rotation(
		1/box->getDimX(), 0, 0,
		0, 1/box->getDimY(), 0,
		0, 0, 1/box->getDimZ()
	);

	std::cout << "S = (";
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++) std::cout << S(i,j) << ", ";
		if (i < 2) std::cout << "\n";
	}
	std::cout << ")\n";

	std::cout << "Sinv = (";
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++) std::cout << Sinv(i,j) << ", ";
		if (i < 2) std::cout << "\n";
	}
	std::cout << ")\n";

	double qx, qy, qz, qw;
	box->getQuaternion(qw, qx, qy, qz);

	// rotation from box to root coordinates
	KDL::Rotation Rbox = KDL::Rotation::Quaternion(qx, qy, qz, qw);
	KDL::Rotation R = pose_b_.M * Rbox;

	std::cout << "Rbox = (";
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++) std::cout << Rbox(i,j) << ", ";
		if (i < 2) std::cout << "\n";
	}
	std::cout << ")\n";

	std::cout << "R = (";
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++) std::cout << R(i,j) << ", ";
		if (i < 2) std::cout << "\n";
	}
	std::cout << ")\n";

	KDL::Vector c__= pose_b_.M * box->getCenterKDL();
	KDL::Vector c = pose_b_.p + c__;

	KDL::Vector x = Sinv * R * (p - c);

	std::cout << "p = (" << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
	std::cout << "c = (" << c.x() << ", " << c.y() << ", " << c.z() << ")\n";
	std::cout << "x = (" << x.x() << ", " << x.y() << ", " << x.z() << ")\n";

	double xx = std::abs(x.x());
	double xy = std::abs(x.y());
	double xz = std::abs(x.z());
	double f = 0;
	if (xx > xy)
	{
		if (xx > xz) f = x.x();
		else f = x.z();
	}
	else
	{
		if (xy > xz) f = x.y();
		else f = x.z();
	}

	double lambda = 0.5 * 1 / f;
	KDL::Vector x_prim = lambda * x;
	KDL::Vector x_prim__ = S * R.Inverse() * x_prim;
	KDL::Vector p_prim = x_prim__ + c;

	std::cout << "x_prim = (" << x_prim.x() << ", " << x_prim.y() << ", " << x_prim.z() << ")\n";
	std::cout << "p_prim = (" << p_prim.x() << ", " <<p_prim.y() << ", " << p_prim.z() << ")\n";

	GeometricPoint cpoint( "c", "world", true, {0, 0.0, 1.0, 0.5} );
	std::vector<double> ppos2;
	ppos2.push_back(c.x()); 
	ppos2.push_back(c.y()); 
	ppos2.push_back(c.z());
	cpoint.init( ppos2 );
	getVisualizer()->update(1235, &cpoint);


	GeometricLine bline( "bline", "world", true, {0, 0.0, 1.0, 0.5} );
	std::vector<double> blineparam;
	blineparam.push_back(c.x()-p.x()); 
	blineparam.push_back(c.y()-p.y()); 
	blineparam.push_back(c.z()-p.z());
	blineparam.push_back(c.x()); 
	blineparam.push_back(c.y()); 
	blineparam.push_back(c.z());
	bline.init( blineparam );
	getVisualizer()->update(1236, &bline);




	GeometricPoint projpoint( "p_prim", "world", true, {0, 1.0, 0, 0.5} );
	std::vector<double> ppos;
	ppos.push_back(p_prim.x()); 
	ppos.push_back(p_prim.y()); 
	ppos.push_back(p_prim.z());
	projpoint.init( ppos );
	getVisualizer()->update(1234, &projpoint);

	

	KDL::Vector d = p - p_prim;
	e_(0) = KDL::dot(d, d);

	// The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jp2p1 = getVelocityJacobianForTwoPoints(p__, x_prim__+c__, q_nr);

		J_(0, q_nr) = - 2 * dot(d, Jp2p1);
	}


	return 0;

}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricCylinder>::project
(
	GeometricPoint* point, 
	GeometricCylinder* cylinder
)
{
	KDL::Vector p__ = pose_a_.M * point->getPointKDL();
	KDL::Vector p = pose_a_.p + p__;

	KDL::Vector v = pose_b_.M * cylinder->getDirectionKDL();

	KDL::Vector d__ = pose_b_.M * cylinder->getOffsetKDL();
	KDL::Vector d = pose_b_.p + d__;

	KDL::Vector x = p - d;
	double s = KDL::dot(x, v);

	e_(0) = KDL::dot(x, x) - s*s - cylinder->getRadius()*cylinder->getRadius();

	// The task jacobian is J = 2 (p-d)^T (I-vv^T) (Jp-Jd)

	// As KDL does not provide a KDL::Matrix class, we use KDL::Rotation
	// although K is not an actual rotation matrix in this context !
	// K = (I - v v^T)
	KDL::Rotation K = KDL::Rotation(KDL::Vector(1,0,0) - v*v(0), 
		                            KDL::Vector(0,1,0) - v*v(1), 
		                            KDL::Vector(0,0,1) - v*v(2));

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jpd = - getVelocityJacobianForTwoPoints(p__, d__, q_nr);

		KDL::Vector y = K * Jpd;

		J_(0, q_nr) = 2 * KDL::dot(x, y);
	}
	
	return 0;
}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricSphere>::project
(
	GeometricPoint* point, 
	GeometricSphere* sphere
)
{
	KDL::Vector p1__ = pose_a_.M * point->getPointKDL();
	KDL::Vector p1 = pose_a_.p + p1__;

	KDL::Vector p2__ = pose_b_.M * sphere->getCenterKDL();
	KDL::Vector p2 = pose_b_.p + p2__;

	KDL::Vector d = p2 - p1;
	e_(0) = KDL::dot(d, d) - sphere->getRadius()*sphere->getRadius();

	// The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jp2p1 = getVelocityJacobianForTwoPoints(p1__, p2__, q_nr);

		J_(0, q_nr) = 2 * dot(d, Jp2p1);
	}
	
	return 0;
}









//#############################################################################
//
//                                 S P H E R E
//
//#############################################################################









template<>
int TaskGeometricProjection<GeometricSphere, GeometricPlane>::project
(
	GeometricSphere* sphere, 
	GeometricPlane* plane
)
{
	KDL::Vector c__ = pose_a_.M * sphere->getCenterKDL();
	KDL::Vector c = pose_a_.p + c__;

	KDL::Vector n = pose_b_.M * plane->getNormalKDL();

	KDL::Vector d__ = n * plane->getOffset();
	KDL::Vector d = d__ + n * KDL::dot(n, pose_b_.p);



	e_(0) = (KDL::dot(c, n) - plane->getOffset() - KDL::dot(n, pose_b_.p));

	// The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jpd = - getVelocityJacobianForTwoPoints(c__, d__, q_nr);

		J_(0, q_nr) = KDL::dot(n, Jpd);
	}
	
	return 0;
}







template<>
int TaskGeometricProjection<GeometricSphere, GeometricSphere>::project
(
	GeometricSphere* sphere1, 
	GeometricSphere* sphere2
)
{
	KDL::Vector p1__ = pose_a_.M * sphere1->getCenterKDL();
	KDL::Vector p1 = pose_a_.p + p1__;

	KDL::Vector p2__ = pose_b_.M * sphere2->getCenterKDL();
	KDL::Vector p2 = pose_b_.p + p2__;

	KDL::Vector d = p2 - p1;
	double r1 = sphere1->getRadius();
	double r2 = sphere2->getRadius();
	e_(0) = KDL::dot(d, d) - (r1*r1 + 2*r1*r2 + r2*r2);

	// The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Jp2p1 = getVelocityJacobianForTwoPoints(p1__, p2__, q_nr);

		J_(0, q_nr) = 2 * dot(d, Jp2p1);
	}
	
	return 0;
}







} // namespace hiqp