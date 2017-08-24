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

#include <hiqp/tasks/tdef_tracking.h>

#include <hiqp/geometric_primitives/geometric_frame.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/utilities.h>

#include <iostream>
#include <sstream>
#include <string>

namespace hiqp {
  namespace tasks {

    template <>
    int TDefTracking<GeometricPoint, GeometricPoint>::track(std::shared_ptr<GeometricPoint> point1,
									 std::shared_ptr<GeometricPoint> point2,
									 const RobotStatePtr robot_state) {

      KDL::Vector p1__ = pose_a_.M * point1->getPointKDL(); //point 1 from link origin to ee expressed in the world frame
      KDL::Vector p2__ = pose_b_.M * point2->getPointKDL(); //point 2 from link origin to ee expressed in the world frame
      KDL::Vector p1 = pose_a_.p + p1__;  //absolute ee point 1 expressed in the world frame
      KDL::Vector p2 = pose_b_.p + p2__;  //absolute ee point 2 expressed in the world frame

      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_p2, J_p1, J_dot_p2, J_dot_p1;
      J_p2.resize(q_nr);
      J_p1.resize(q_nr);
      J_dot_p2.resize(q_nr);
      J_dot_p1.resize(q_nr);      

      changeJacRefPoint(jacobian_a_, p1__, J_p1);
      changeJacRefPoint(jacobian_b_, p2__, J_p2);

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, p1__, J_dot_p1);
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, p2__, J_dot_p2);

      e_= Eigen::Map<Eigen::Matrix<double,3,1> >(p1.data) - Eigen::Map<Eigen::Matrix<double,3,1> >(p2.data);
      J_=J_p1.data.topRows<3>()-J_p2.data.topRows<3>();
      e_dot_=J_*robot_state->kdl_jnt_array_vel_.qdot.data;
      J_dot_=J_dot_p1.data.topRows<3>()-J_dot_p2.data.topRows<3>();
	
      return 0;
    }

    template <>
    int TDefTracking<GeometricPoint, GeometricFrame>::track(std::shared_ptr<GeometricPoint> point,
									 std::shared_ptr<GeometricFrame> frame,
									 const RobotStatePtr robot_state) {
      KDL::Vector p1__ = pose_a_.M * point->getPointKDL(); //point 1 from link origin to ee expressed in the world frame
      KDL::Vector p2__ = pose_b_.M * frame->getCenterKDL(); //point 2 from link origin to ee expressed in the world frame
      KDL::Vector p1 = pose_a_.p + p1__;  //absolute ee point 1 expressed in the world frame
      KDL::Vector p2 = pose_b_.p + p2__;  //absolute ee point 2 expressed in the world frame
    
      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_p2, J_p1, J_dot_p2, J_dot_p1;
      J_p2.resize(q_nr);
      J_p1.resize(q_nr);
      J_dot_p2.resize(q_nr);
      J_dot_p1.resize(q_nr);      

      changeJacRefPoint(jacobian_a_, p1__, J_p1);
      changeJacRefPoint(jacobian_b_, p2__, J_p2);

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, p1__, J_dot_p1);
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, p2__, J_dot_p2);

      e_= Eigen::Map<Eigen::Matrix<double,3,1> >(p1.data) - Eigen::Map<Eigen::Matrix<double,3,1> >(p2.data);
      J_=J_p1.data.topRows<3>()-J_p2.data.topRows<3>();
      e_dot_=J_*robot_state->kdl_jnt_array_vel_.qdot.data;
      J_dot_=J_dot_p1.data.topRows<3>()-J_dot_p2.data.topRows<3>();
	
      return 0;
    }

    template <>
    int TDefTracking<GeometricFrame, GeometricFrame>::track(std::shared_ptr<GeometricFrame> frame1,
							    std::shared_ptr<GeometricFrame> frame2,
							    const RobotStatePtr robot_state) {
      double q_nr=jacobian_a_.columns();
      Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
	    
      //POSITION TRACKING
      KDL::Vector p1__ = pose_a_.M * frame1->getCenterKDL(); //point 1 from link origin to the frame center expressed in the world frame
      KDL::Vector p1 = pose_a_.p + p1__;  //absolute ee point 1 expressed in the world frame
  
      KDL::Vector p2__ = pose_b_.M * frame2->getCenterKDL(); //point 2 from link origin to the sframe center expressed in the world frame
      KDL::Vector p2 = pose_b_.p + p2__; //absolute ee point 2 expressed in the world frame

      KDL::Jacobian J_p2(q_nr), J_p1(q_nr), J_dot_p2(q_nr), J_dot_p1(q_nr);
      changeJacRefPoint(jacobian_a_, p1__, J_p1);
      changeJacRefPoint(jacobian_b_, p2__, J_p2);

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, p1__, J_dot_p1);
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, p2__, J_dot_p2);

      e_.topRows<3>()= Eigen::Map<Eigen::Matrix<double,3,1> >(p1.data) - Eigen::Map<Eigen::Matrix<double,3,1> >(p2.data);
      J_.topRows<3>()=J_p1.data.topRows<3>()-J_p2.data.topRows<3>();
      e_dot_.topRows<3>()=J_.topRows<3>()*qdot;
      J_dot_.topRows<3>()=J_dot_p1.data.topRows<3>()-J_dot_p2.data.topRows<3>();


      //ORIENTATION TRACKING VARIANT 1: e(i)=a_i^T*b_i-1
      // KDL::Vector ax1 = pose_a_.M * frame1->getAxisXKDL();
      // KDL::Vector ax2 = pose_b_.M * frame2->getAxisXKDL();
      // KDL::Vector ay1 = pose_a_.M * frame1->getAxisYKDL();
      // KDL::Vector ay2 = pose_b_.M * frame2->getAxisYKDL();
      // KDL::Vector az1 = pose_a_.M * frame1->getAxisYKDL();
      // KDL::Vector az2 = pose_b_.M * frame2->getAxisYKDL();      

      // Eigen::VectorXd e, e_dot;
      // Eigen::MatrixXd J, J_dot;
      // alignUnitVectors(ax1, ax2, robot_state, e, e_dot, J, J_dot);
      // e_(3)=e(0);
      // e_dot_(3)=e_dot(0);
      // J_.row(3)=J;
      // J_dot_.row(3)=J_dot;
      // alignUnitVectors(ay1, ay2, robot_state, e, e_dot, J, J_dot);
      // e_(4)=e(0);
      // e_dot_(4)=e_dot(0);
      // J_.row(4)=J;
      // J_dot_.row(4)=J_dot;
      // alignUnitVectors(az1, az2, robot_state, e, e_dot, J, J_dot);      
      // e_(5)=e(0);
      // e_dot_(5)=e_dot(0);
      // J_.row(5)=J;
      // J_dot_.row(5)=J_dot;

      //ORIENTATION TRACKING VARIANT 2: e=sin(alpha)*n
     KDL::Vector a1 = pose_a_.M * frame1->getAxisXKDL();
     KDL::Vector a2 = pose_a_.M * frame1->getAxisYKDL();
     KDL::Vector a3 = pose_a_.M * frame1->getAxisZKDL();
     KDL::Vector b1 = pose_b_.M * frame2->getAxisXKDL();
     KDL::Vector b2 = pose_b_.M * frame2->getAxisYKDL();
     KDL::Vector b3 = pose_b_.M * frame2->getAxisZKDL();     

      KDL::Jacobian J_a1(q_nr), J_b1(q_nr), J_dot_a1(q_nr), J_dot_b1(q_nr);
      KDL::Jacobian J_a2(q_nr), J_b2(q_nr), J_dot_a2(q_nr), J_dot_b2(q_nr);      
      KDL::Jacobian J_a3(q_nr), J_b3(q_nr), J_dot_a3(q_nr), J_dot_b3(q_nr);
      
     changeJacRefPoint(jacobian_a_, a1, J_a1);
     J_a1.data=J_a1.data-jacobian_a_.data;
     changeJacRefPoint(jacobian_a_, a2, J_a2);
     J_a2.data=J_a2.data-jacobian_a_.data;
     changeJacRefPoint(jacobian_a_, a3, J_a3);
     J_a3.data=J_a3.data-jacobian_a_.data;
     changeJacRefPoint(jacobian_b_, b1, J_b1);
     J_b1.data=J_b1.data-jacobian_b_.data;
     changeJacRefPoint(jacobian_b_, b2, J_b2);
     J_b2.data=J_b2.data-jacobian_b_.data;
     changeJacRefPoint(jacobian_b_, b3, J_b3);
     J_b3.data=J_b3.data-jacobian_b_.data;     
     
     changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, a1, J_dot_a1);
     J_dot_a1.data=J_dot_a1.data-jacobian_dot_a_.data;
     changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, a2, J_dot_a2);
     J_dot_a2.data=J_dot_a2.data-jacobian_dot_a_.data;
     changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, a3, J_dot_a3);
     J_dot_a3.data=J_dot_a3.data-jacobian_dot_a_.data;
     changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, b1, J_dot_b1);
     J_dot_b1.data=J_dot_b1.data-jacobian_dot_b_.data;
     changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, b2, J_dot_b2);
     J_dot_b2.data=J_dot_b2.data-jacobian_dot_b_.data;
     changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, b3, J_dot_b3);
     J_dot_b3.data=J_dot_b3.data-jacobian_dot_b_.data;

     Eigen::Matrix3d Sa1=skewSymmetricMatrix(Eigen::Vector3d(a1(0), a1(1), a1(2)));
     Eigen::Matrix3d Sa2=skewSymmetricMatrix(Eigen::Vector3d(a2(0), a2(1), a2(2)));
     Eigen::Matrix3d Sa3=skewSymmetricMatrix(Eigen::Vector3d(a3(0), a3(1), a3(2)));
     Eigen::Matrix3d Sb1=skewSymmetricMatrix(Eigen::Vector3d(b1(0), b1(1), b1(2)));
     Eigen::Matrix3d Sb2=skewSymmetricMatrix(Eigen::Vector3d(b2(0), b2(1), b2(2)));
     Eigen::Matrix3d Sb3=skewSymmetricMatrix(Eigen::Vector3d(b3(0), b3(1), b3(2)));

     Eigen::Matrix3d Sa1_dot=skewSymmetricMatrix((jacobian_dot_a_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(a1.data)));
     Eigen::Matrix3d Sa2_dot=skewSymmetricMatrix((jacobian_dot_a_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(a2.data)));
     Eigen::Matrix3d Sa3_dot=skewSymmetricMatrix((jacobian_dot_a_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(a3.data)));
     Eigen::Matrix3d Sb1_dot=skewSymmetricMatrix((jacobian_dot_b_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(b1.data)));
     Eigen::Matrix3d Sb2_dot=skewSymmetricMatrix((jacobian_dot_b_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(b2.data)));
     Eigen::Matrix3d Sb3_dot=skewSymmetricMatrix((jacobian_dot_b_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(b3.data)));
     
     e_.bottomRows<3>()=0.5*(Sa1*Eigen::Map<Eigen::Matrix<double,3,1> >(b1.data)+Sa2*Eigen::Map<Eigen::Matrix<double,3,1> >(b2.data)+Sa3*Eigen::Map<Eigen::Matrix<double,3,1> >(b3.data));
     J_.bottomRows<3>()=0.5*(Sa1*J_b1.data.topRows<3>()-Sb1*J_a1.data.topRows<3>()+Sa2*J_b2.data.topRows<3>()-Sb2*J_a2.data.topRows<3>()+Sa3*J_b3.data.topRows<3>()-Sb3*J_a3.data.topRows<3>());
     e_dot_.bottomRows<3>()=J_.bottomRows<3>()*robot_state->kdl_jnt_array_vel_.qdot.data;
     J_dot_.bottomRows<3>()=0.5*(Sa1_dot*J_b1.data.topRows<3>()+Sa1*J_dot_b1.data.topRows<3>()-Sb1_dot*J_a1.data.topRows<3>()-Sb1*J_dot_a1.data.topRows<3>()+Sa2_dot*J_b2.data.topRows<3>()+Sa2*J_dot_b2.data.topRows<3>()-Sb2_dot*J_a2.data.topRows<3>()-Sb2*J_dot_a2.data.topRows<3>()+Sa3_dot*J_b3.data.topRows<3>()+Sa3*J_dot_b3.data.topRows<3>()-Sb3_dot*J_a3.data.topRows<3>()-Sb3*J_dot_a3.data.topRows<3>());
     
      return 0;
    }
  }  // namespace tasks

}  // namespace hiqp
