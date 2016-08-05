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
 * \file   geometric_box.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_BOX_H
#define HIQP_GEOMETRIC_BOX_H




#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/hiqp_utils.h>

#include <kdl/frames.hpp>

#include <Eigen/Dense>



namespace hiqp
{




/*!
 * \class GeometricBox
 * \brief 
 * 
 * The angle is the box's clockwise angle of rotation around the normal vector 
 * of the upper side while looking in the same direction as the normal vector.
 * 
 * box:          [x, y, z, w, d, h]
 * box:          [x, y, z, w, d, h, ex, ey, ez] (using the 2-1-2 Euler angle convention)
 * box:          [x, y, z, w, d, h, qw, qx, qy, qz]
 */  
class GeometricBox : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricBox
	(
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color
	)
	: GeometricPrimitive(name, frame_id, visible, color)
	{}

	/*!
     * \brief Destructor
     */
	~GeometricBox() noexcept {}

	int init(const std::vector<double>& parameters)
	{
		int size = parameters.size();
		if (size != 6 && size != 9 && size != 10)
		{
			printHiqpWarning("GeometricBox requires 6, 9 or 10 parameters, got " 
				+ std::to_string(size) + "! Initialization failed!");
			return -1;
		}

		kdl_c_(0) = parameters.at(0);
		kdl_c_(1) = parameters.at(1);
		kdl_c_(2) = parameters.at(2);

		kdl_dim_(0) = parameters.at(3);
		kdl_dim_(1) = parameters.at(4);
		kdl_dim_(2) = parameters.at(5);

		if (size == 9)
		{
			double angle1 = parameters.at(6);
			double angle2 = parameters.at(7);
			double angle3 = parameters.at(8);

			Eigen::Matrix3d m;
			m =   Eigen::AngleAxisd(angle1, Eigen::Vector3d::UnitX())
			    * Eigen::AngleAxisd(angle2, Eigen::Vector3d::UnitY())
			    * Eigen::AngleAxisd(angle3, Eigen::Vector3d::UnitZ());

			q_ = Eigen::Quaternion<double>(m);
		}
		else if (size == 10)
		{
			double w = parameters.at(6);
			double x = parameters.at(7);
			double y = parameters.at(8);
			double z = parameters.at(9);

			q_ = Eigen::Quaternion<double>(w, x, y, z);
		}

		// Calculate the normal of the left side of the box
		// KDL::Vector left = KDL::Vector(0, 0, 1) * kdl_n_up_;
		// KDL::Vector back = kdl_n_up_ * left;
		// kdl_n_left_ = std::cos(a_) * left + std::sin(a_) * back;

		eigen_c_ << kdl_c_(0), kdl_c_(1), kdl_c_(2);
		eigen_dim_ << kdl_dim_(0), kdl_dim_(1), kdl_dim_(2);
		// eigen_n_up_ << kdl_n_up_(0), kdl_n_up_(1), kdl_n_up_(2);
		// eigen_n_left_ << kdl_n_left_(0), kdl_n_left_(1), kdl_n_left_(2);

		return 0;
	}

	inline const KDL::Vector&     getCentrumKDL() { return kdl_c_; }
	inline const Eigen::Vector3d& getCentrumEigen() { return eigen_c_; }

	inline const KDL::Vector&     getDimensionsKDL() { return kdl_dim_; }
	inline const Eigen::Vector3d& getDimensionsEigen() { return eigen_dim_; }

	inline const Eigen::Quaternion<double>& getQuaternionEigen() { return q_; }
	inline void getQuaternion(double& w, double& x, double& y, double& z)
	{ w = q_.w(); x = q_.x(); y = q_.y(); z = q_.z(); }


	// inline const KDL::Vector&     getNormalUpKDL() { return kdl_n_up_; }
	// inline const Eigen::Vector3d& getNormalUpEigen() { return eigen_n_up_; }

	// inline const KDL::Vector&     getNormalLeftKDL() { return kdl_n_left_; }
	// inline const Eigen::Vector3d& getNormalLeftEigen() { return eigen_n_left_; }


	inline double getCenterX() { return kdl_c_(0); }
	inline double getCenterY() { return kdl_c_(1); }
	inline double getCenterZ() { return kdl_c_(2); }

	inline double getDimX() { return kdl_dim_(0); }
	inline double getDimY() { return kdl_dim_(1); }
	inline double getDimZ() { return kdl_dim_(2); }

	// inline double getNormalUpX() { return kdl_n_up_(0); }
	// inline double getNormalUpY() { return kdl_n_up_(1); }
	// inline double getNormalUpZ() { return kdl_n_up_(2); }

	// inline double getNormalLeftX() { return kdl_n_left_(0); }
	// inline double getNormalLeftY() { return kdl_n_left_(1); }
	// inline double getNormalLeftZ() { return kdl_n_left_(2); }

	



protected:

	KDL::Vector      kdl_c_; // the geometrical cetrum of the box
	Eigen::Vector3d  eigen_c_;

	KDL::Vector      kdl_dim_; // the dimensions of the box
	Eigen::Vector3d  eigen_dim_;

	Eigen::Quaternion<double> q_;


	// KDL::Vector      kdl_n_up_; // the normal vector of the top plane of the box
	// Eigen::Vector3d  eigen_n_up_;

	// KDL::Vector      kdl_n_left_; // the normal vector of the left plane of the box
	// Eigen::Vector3d  eigen_n_left_;




private:

	// No copying of this class is allowed !
	GeometricBox(const GeometricBox& other) = delete;
	GeometricBox(GeometricBox&& other) = delete;
	GeometricBox& operator=(const GeometricBox& other) = delete;
	GeometricBox& operator=(GeometricBox&& other) noexcept = delete;

};











} // namespace hiqp

#endif // include guard