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
//#include <kdl/frames.hpp>
#include <Eigen/Geometry>
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

      KDL::Vector dp=p2-p1;
      //correct point 2's location if d max is exceeded
      if(dp.Norm() > d_max_){
	p2=dp/dp.Norm()*d_max_+p1;
      }
      
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

      KDL::Vector dp=p2-p1;
      //correct point 2's location if d max is exceeded
      if(dp.Norm() > d_max_){
	p2=dp/dp.Norm()*d_max_+p1;
      }
    
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

      KDL::Vector dp=p2-p1;
      //correct point 2's location if d max is exceeded
      if(dp.Norm() > d_max_){
	p2=dp/dp.Norm()*d_max_+p1;
      }
        
      KDL::Jacobian J_p2(q_nr), J_p1(q_nr), J_dot_p2(q_nr), J_dot_p1(q_nr);
      changeJacRefPoint(jacobian_a_, p1__, J_p1);
      changeJacRefPoint(jacobian_b_, p2__, J_p2);

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, p1__, J_dot_p1);
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, p2__, J_dot_p2);

      e_.topRows<3>()= Eigen::Map<Eigen::Matrix<double,3,1> >(p1.data) - Eigen::Map<Eigen::Matrix<double,3,1> >(p2.data);
      J_.topRows<3>()=J_p1.data.topRows<3>()-J_p2.data.topRows<3>();
      e_dot_.topRows<3>()=J_.topRows<3>()*qdot;
      J_dot_.topRows<3>()=J_dot_p1.data.topRows<3>()-J_dot_p2.data.topRows<3>();


      KDL::Vector ax1 = pose_a_.M * frame1->getAxisXKDL();
      KDL::Vector ay1 = pose_a_.M * frame1->getAxisYKDL();
      KDL::Vector az1 = pose_a_.M * frame1->getAxisZKDL();
      KDL::Vector ax2 = pose_b_.M * frame2->getAxisXKDL();
      KDL::Vector ay2 = pose_b_.M * frame2->getAxisYKDL();
      KDL::Vector az2 = pose_b_.M * frame2->getAxisZKDL();

      //Back-project the rotation if the error is too large
      KDL::Rotation A_W_R; A_W_R.UnitX(ax1); A_W_R.UnitY(ay1); A_W_R.UnitZ(az1);
      KDL::Rotation B_W_R; B_W_R.UnitX(ax2); B_W_R.UnitY(ay2); B_W_R.UnitZ(az2);
      double qx, qy,qz, qw;
      A_W_R.GetQuaternion(qx,qy,qz,qw);
      Eigen::Quaterniond qA(qw, qx, qy, qz);
      B_W_R.GetQuaternion(qx,qy,qz,qw);
      Eigen::Quaterniond qB(qw, qx, qy, qz);

      Eigen::Quaterniond qAB=qA.inverse()*qB;
      Eigen::Quaterniond qB_new, qAB_new;
      double phi=2*acos(qAB.w());
      if(phi > phi_max_){
	qB_new = qA.slerp(phi_max_/phi, qB);
      }
      else{
	qB_new = qB;
      }
      qB_new.normalize();
      Eigen::Matrix3d _B_W_R=qB_new.toRotationMatrix();
      ax2(0)=_B_W_R(0,0); ay2(0)=_B_W_R(0,1); az2(0)=_B_W_R(0,2);
      ax2(1)=_B_W_R(1,0); ay2(1)=_B_W_R(1,1); az2(1)=_B_W_R(1,2);
      ax2(2)=_B_W_R(2,0); ay2(2)=_B_W_R(2,1); az2(2)=_B_W_R(2,2);
      
      //DEBUG =======================================================
      //	qAB_new=qA.inverse()*qB_new;
      // double phi_new=2*acos(qAB_new.w());     	
      // 	std::cerr<<"p_A: "<<p1(0)<<" "<<p1(1)<<" "<<p1(2)<<std::endl;
      // 	std::cerr<<"p_B: "<<p2(0)<<" "<<p2(1)<<" "<<p2(2)<<std::endl;      
      // 	std::cerr<<"A_W_R: "<<std::endl<<qA.toRotationMatrix()<<std::endl; 
      // 	std::cerr<<"old B_W_R: "<<std::endl<<qB.toRotationMatrix()<<std::endl;
      // 	std::cerr<<"new B_W_R: "<<std::endl<<_B_W_R<<std::endl;
      // 	std::cerr<<"old phi: "<<phi<<std::endl;
      // 	std::cerr<<"new phi: "<<phi_new<<std::endl;
      // 	std::cerr<<"phi_max_: "<<phi_max_<<std::endl;                  
      // 	std::cerr<<"new ax2: "<<ax2(0)<<" "<<ax2(1)<<" "<<ax2(2)<<std::endl;
      // 	std::cerr<<"new ay2: "<<ay2(0)<<" "<<ay2(1)<<" "<<ay2(2)<<std::endl;
      // 	std::cerr<<"new az2: "<<az2(0)<<" "<<az2(1)<<" "<<az2(2)<<std::endl;      
      // 	std::cerr<<"______________________________________________\n"<<std::endl;
       
      // visualization_msgs::MarkerArray marker_array;
      // double  marker_lifetime=0;
      // std::vector<double> color;
      // color.push_back(0.1);color.push_back(0.5); color.push_back(0.7); color.push_back(1.0);
      // std::shared_ptr<GeometricFrame> frame(new GeometricFrame("new_frame","world",1,color));
      // std::vector<double> params;
      // params.push_back(p2(0));
      // params.push_back(p2(1));
      // params.push_back(p2(2));
      // params.push_back(qB_new.w());
      // params.push_back(qB_new.x());
      // params.push_back(qB_new.y());
      // params.push_back(qB_new.z());
      // // std::cerr<<"params: ";
      // // for(int i=0;i<params.size();i++)
      // // 	std::cerr<<params[i]<<" ";

      // // std::cerr<<std::endl;
      // frame->init(params);
      // {
      // 	visualization_msgs::Marker marker;
      // 	marker.header.frame_id = "/" + frame->getFrameId();
      // 	marker.header.stamp = ros::Time::now();
      // 	marker.ns = "/yumi";
      // 	marker.id = 1;
      // 	marker.type = visualization_msgs::Marker::ARROW;
      // 	marker.action = visualization_msgs::Marker::ADD;
      // 	marker.pose.position.x = frame->getX();
      // 	marker.pose.position.y = frame->getY();
      // 	marker.pose.position.z = frame->getZ();
      // 	marker.pose.orientation.x = frame->getQX();
      // 	marker.pose.orientation.y = frame->getQY();
      // 	marker.pose.orientation.z = frame->getQZ();
      // 	marker.pose.orientation.w = frame->getQW();
      // 	marker.scale.x = 0.04;
      // 	marker.scale.y = 2 * 0.005;
      // 	marker.scale.z = 2 * 0.005;
      // 	marker.color.r = 1.0;  // 0.5*frame->getRedComponent() + 0.5;
      // 	marker.color.g = 0.0;  // 0.5*frame->getGreenComponent();
      // 	marker.color.b = 0.0;  // 0.5*frame->getBlueComponent();
      // 	marker.color.a = frame->getAlphaComponent();
      // 	marker.lifetime = ros::Duration(marker_lifetime);
      // 	marker_array.markers.push_back(marker);
      // }
      // {
      // 	visualization_msgs::Marker marker;
      // 	marker.header.frame_id = "/" + frame->getFrameId();
      // 	marker.header.stamp = ros::Time::now();
      // 	marker.ns = "/yumi";
      // 	marker.id = 2;
      // 	marker.type = visualization_msgs::Marker::ARROW;
      // 	marker.action = visualization_msgs::Marker::ADD;
      // 	marker.pose.position.x = frame->getX();
      // 	marker.pose.position.y = frame->getY();
      // 	marker.pose.position.z = frame->getZ();
      // 	Eigen::Quaternion<double> rotq(0.70710678118, 0, 0, 0.70710678118);
      // 	Eigen::Quaternion<double> resq = frame->getQuaternionEigen() * rotq;
      // 	marker.pose.orientation.x = resq.x();
      // 	marker.pose.orientation.y = resq.y();
      // 	marker.pose.orientation.z = resq.z();
      // 	marker.pose.orientation.w = resq.w();
      // 	marker.scale.x = 0.04;
      // 	marker.scale.y = 2 * 0.005;
      // 	marker.scale.z = 2 * 0.005;
      // 	marker.color.r = 0.0;  // 0.5*frame->getRedComponent() + 0.5;
      // 	marker.color.g = 1.0;  // 0.5*frame->getGreenComponent();
      // 	marker.color.b = 0.0;  // 0.5*frame->getBlueComponent();
      // 	marker.color.a = frame->getAlphaComponent();
      // 	marker.lifetime = ros::Duration(marker_lifetime);
      // 	marker_array.markers.push_back(marker);
      // }
      // {
      // 	visualization_msgs::Marker marker;
      // 	marker.header.frame_id = "/" + frame->getFrameId();
      // 	marker.header.stamp = ros::Time::now();
      // 	marker.id=3;
      // 	marker.ns = "/yumi";
      // 	marker.type = visualization_msgs::Marker::ARROW;
      // 	marker.action = visualization_msgs::Marker::ADD;
      // 	marker.pose.position.x = frame->getX();
      // 	marker.pose.position.y = frame->getY();
      // 	marker.pose.position.z = frame->getZ();
      // 	Eigen::Quaternion<double> rotq(0.70710678118, 0, -0.70710678118, 0);
      // 	Eigen::Quaternion<double> resq = frame->getQuaternionEigen() * rotq;
      // 	marker.pose.orientation.x = resq.x();
      // 	marker.pose.orientation.y = resq.y();
      // 	marker.pose.orientation.z = resq.z();
      // 	marker.pose.orientation.w = resq.w();
      // 	marker.scale.x = 0.04;
      // 	marker.scale.y = 2 * 0.005;
      // 	marker.scale.z = 2 * 0.005;
      // 	marker.color.r = 0.0;  // 0.5*frame->getRedComponent() + 0.5;
      // 	marker.color.g = 0.0;  // 0.5*frame->getGreenComponent();
      // 	marker.color.b = 1.0;  // 0.5*frame->getBlueComponent();
      // 	marker.color.a = frame->getAlphaComponent();
      // 	marker.lifetime = ros::Duration(marker_lifetime);
      // 	marker_array.markers.push_back(marker);
      // }
      // marker_pub_.publish(marker_array);
      //DEBUG END =======================================================

      //ORIENTATION TRACKING VARIANT 1: e(i)=a_i^T*b_i-1      
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

      // ORIENTATION TRACKING VARIANT 2: e=sin(alpha)*n
       KDL::Jacobian J_a1(q_nr), J_b1(q_nr), J_dot_a1(q_nr), J_dot_b1(q_nr);
       KDL::Jacobian J_a2(q_nr), J_b2(q_nr), J_dot_a2(q_nr), J_dot_b2(q_nr);      
       KDL::Jacobian J_a3(q_nr), J_b3(q_nr), J_dot_a3(q_nr), J_dot_b3(q_nr);
      
      changeJacRefPoint(jacobian_a_, ax1, J_a1);
      J_a1.data=J_a1.data-jacobian_a_.data;
      changeJacRefPoint(jacobian_a_, ay1, J_a2);
      J_a2.data=J_a2.data-jacobian_a_.data;
      changeJacRefPoint(jacobian_a_, az1, J_a3);
     J_a3.data=J_a3.data-jacobian_a_.data;
     changeJacRefPoint(jacobian_b_, ax2, J_b1);
     J_b1.data=J_b1.data-jacobian_b_.data;
     changeJacRefPoint(jacobian_b_, ay2, J_b2);
     J_b2.data=J_b2.data-jacobian_b_.data;
     changeJacRefPoint(jacobian_b_, az2, J_b3);
     J_b3.data=J_b3.data-jacobian_b_.data;     
     
     changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, ax1, J_dot_a1);
     J_dot_a1.data=J_dot_a1.data-jacobian_dot_a_.data;
     changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, ay1, J_dot_a2);
     J_dot_a2.data=J_dot_a2.data-jacobian_dot_a_.data;
     changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, az1, J_dot_a3);
     J_dot_a3.data=J_dot_a3.data-jacobian_dot_a_.data;
     changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, ax2, J_dot_b1);
     J_dot_b1.data=J_dot_b1.data-jacobian_dot_b_.data;
     changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, ay2, J_dot_b2);
     J_dot_b2.data=J_dot_b2.data-jacobian_dot_b_.data;
     changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, az2, J_dot_b3);
     J_dot_b3.data=J_dot_b3.data-jacobian_dot_b_.data;

     Eigen::Matrix3d Sa1=skewSymmetricMatrix(Eigen::Vector3d(ax1(0), ax1(1), ax1(2)));
     Eigen::Matrix3d Sa2=skewSymmetricMatrix(Eigen::Vector3d(ay1(0), ay1(1), ay1(2)));
     Eigen::Matrix3d Sa3=skewSymmetricMatrix(Eigen::Vector3d(az1(0), az1(1), az1(2)));
     Eigen::Matrix3d Sb1=skewSymmetricMatrix(Eigen::Vector3d(ax2(0), ax2(1), ax2(2)));
     Eigen::Matrix3d Sb2=skewSymmetricMatrix(Eigen::Vector3d(ay2(0), ay2(1), ay2(2)));
     Eigen::Matrix3d Sb3=skewSymmetricMatrix(Eigen::Vector3d(az2(0), az2(1), az2(2)));

     Eigen::Matrix3d Sa1_dot=skewSymmetricMatrix((jacobian_dot_a_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(ax1.data)));
     Eigen::Matrix3d Sa2_dot=skewSymmetricMatrix((jacobian_dot_a_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(ay1.data)));
     Eigen::Matrix3d Sa3_dot=skewSymmetricMatrix((jacobian_dot_a_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(az1.data)));
     Eigen::Matrix3d Sb1_dot=skewSymmetricMatrix((jacobian_dot_b_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(ax2.data)));
     Eigen::Matrix3d Sb2_dot=skewSymmetricMatrix((jacobian_dot_b_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(ay2.data)));
     Eigen::Matrix3d Sb3_dot=skewSymmetricMatrix((jacobian_dot_b_.data.bottomRows<3>()*qdot).cross(Eigen::Map<Eigen::Matrix<double,3,1> >(az2.data)));
     
     e_.bottomRows<3>()=0.5*(Sa1*Eigen::Map<Eigen::Matrix<double,3,1> >(ax2.data)+Sa2*Eigen::Map<Eigen::Matrix<double,3,1> >(ay2.data)+Sa3*Eigen::Map<Eigen::Matrix<double,3,1> >(az2.data));
     J_.bottomRows<3>()=0.5*(Sa1*J_b1.data.topRows<3>()-Sb1*J_a1.data.topRows<3>()+Sa2*J_b2.data.topRows<3>()-Sb2*J_a2.data.topRows<3>()+Sa3*J_b3.data.topRows<3>()-Sb3*J_a3.data.topRows<3>());
     e_dot_.bottomRows<3>()=J_.bottomRows<3>()*robot_state->kdl_jnt_array_vel_.qdot.data;
     J_dot_.bottomRows<3>()=0.5*(Sa1_dot*J_b1.data.topRows<3>()+Sa1*J_dot_b1.data.topRows<3>()-Sb1_dot*J_a1.data.topRows<3>()-Sb1*J_dot_a1.data.topRows<3>()+Sa2_dot*J_b2.data.topRows<3>()+Sa2*J_dot_b2.data.topRows<3>()-Sb2_dot*J_a2.data.topRows<3>()-Sb2*J_dot_a2.data.topRows<3>()+Sa3_dot*J_b3.data.topRows<3>()+Sa3*J_dot_b3.data.topRows<3>()-Sb3_dot*J_a3.data.topRows<3>()-Sb3*J_dot_a3.data.topRows<3>());
     
      return 0;
    }
  }  // namespace tasks

}  // namespace hiqp
