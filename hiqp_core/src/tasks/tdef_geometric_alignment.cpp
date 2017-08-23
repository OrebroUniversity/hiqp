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

#include <hiqp/tasks/tdef_geometric_alignment.h>

#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_frame.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <hiqp/utilities.h>

#include <cmath>
#include <iostream>

namespace hiqp {
  namespace tasks {

    template <>
    int TDefGeometricAlignment<GeometricLine, GeometricLine>::align(std::shared_ptr<GeometricLine> line1,
								    std::shared_ptr<GeometricLine> line2,
								    const RobotStatePtr robot_state) {
      KDL::Vector v1 = pose_a_.M * line1->getDirectionKDL();
      KDL::Vector v2 = pose_b_.M * line2->getDirectionKDL();

      //DOT PRODUCT =============================================      
      //return alignUnitVectors(v1, v2, robot_state);
      //DOT PRODUCT END =========================================

      //CROSS PRODUCT =============================================      
      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_v1, J_p1, J_v2, J_v1_dot, J_v2_dot;
      J_v1.resize(q_nr);
      J_v2.resize(q_nr);
      J_p1.resize(q_nr);
      J_v1_dot.resize(q_nr);
      J_v2_dot.resize(q_nr);
    
      changeJacRefPoint(jacobian_a_, v1, J_v1);
      J_v1.data=J_v1.data - jacobian_a_.data;

      changeJacRefPoint(jacobian_b_, v2, J_v2);
      J_v2.data=J_v2.data - jacobian_b_.data;

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, v1, J_v1_dot);
      J_v1_dot.data = J_v1_dot.data - jacobian_dot_a_.data;

      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, v2, J_v2_dot);
      J_v2_dot.data = J_v2_dot.data - jacobian_dot_b_.data;
      
      Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
      Eigen::Vector3d v1_dot=J_v1.data.topRows<3>()*qdot;
      Eigen::Vector3d v2_dot=J_v2.data.topRows<3>()*qdot;

      return rotateVectors(Eigen::Vector3d(v1(0),v1(1),v1(2)), Eigen::Vector3d(v2(0),v2(1),v2(2)), v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());
      //CROSS PRODUCT END =========================================      
    }

    template <>
    int TDefGeometricAlignment<GeometricLine, GeometricPlane>::align(std::shared_ptr<GeometricLine> line,
								     std::shared_ptr<GeometricPlane> plane,
								     const RobotStatePtr robot_state) {
      KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();
      KDL::Vector v2 = pose_b_.M * plane->getNormalKDL();

      //DOT PRODUCT =============================================      
      //return alignUnitVectors(v1, v2, robot_state);
      //DOT PRODUCT END =========================================

      //CROSS PRODUCT =============================================      
      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_v1, J_p1, J_v2, J_v1_dot, J_v2_dot;
      J_v1.resize(q_nr);
      J_v2.resize(q_nr);
      J_p1.resize(q_nr);
      J_v1_dot.resize(q_nr);
      J_v2_dot.resize(q_nr);
    
      changeJacRefPoint(jacobian_a_, v1, J_v1);
      J_v1.data=J_v1.data - jacobian_a_.data;

      changeJacRefPoint(jacobian_b_, v2, J_v2);
      J_v2.data=J_v2.data - jacobian_b_.data;

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, v1, J_v1_dot);
      J_v1_dot.data = J_v1_dot.data - jacobian_dot_a_.data;

      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, v2, J_v2_dot);
      J_v2_dot.data = J_v2_dot.data - jacobian_dot_b_.data;
      
      Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
      Eigen::Vector3d v1_dot=J_v1.data.topRows<3>()*qdot;
      Eigen::Vector3d v2_dot=J_v2.data.topRows<3>()*qdot;
      
      return rotateVectors(Eigen::Vector3d(v1(0),v1(1),v1(2)), Eigen::Vector3d(v2(0),v2(1),v2(2)), v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());
      //CROSS PRODUCT END =============================================
    }

    template <>
    int TDefGeometricAlignment<GeometricLine, GeometricCylinder>::align(std::shared_ptr<GeometricLine> line,
									std::shared_ptr<GeometricCylinder> cylinder,
									const RobotStatePtr robot_state) {
  
      KDL::Vector p1__=pose_a_.M * line->getOffsetKDL();
      KDL::Vector p1=p1__+pose_a_.p;
      KDL::Vector k__=pose_b_.M * cylinder->getOffsetKDL();      
      KDL::Vector k=k__+pose_b_.p;
      Eigen::Vector3d v1=Eigen::Map<Eigen::Matrix<double,3,1> >((pose_a_.M * line->getDirectionKDL()).data);
      Eigen::Vector3d vk=Eigen::Map<Eigen::Matrix<double,3,1> >((pose_b_.M * cylinder->getDirectionKDL()).data);

      Eigen::Vector3d dp=Eigen::Map<Eigen::Matrix<double,3,1> >(p1.data)-Eigen::Map<Eigen::Matrix<double,3,1> >(k.data);
      Eigen::Vector3d v2=dp.dot(vk)*vk-Eigen::Map<Eigen::Matrix<double,3,1> >(p1.data)+Eigen::Map<Eigen::Matrix<double,3,1> >(k.data);
      
      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_v1, J_p1, J_dp, J_vk, J_v2, J_v1_dot, J_v2_dot, J_vk_dot, J_p1_dot, J_dp_dot;
      J_v1.resize(q_nr);
      J_vk.resize(q_nr);
      J_p1.resize(q_nr);
      J_dp.resize(q_nr);      
      J_v2.resize(q_nr);
      J_v1_dot.resize(q_nr);
      J_v2_dot.resize(q_nr);      
      J_vk_dot.resize(q_nr);      
      J_p1_dot.resize(q_nr);
      J_dp_dot.resize(q_nr);      
      
      changeJacRefPoint(jacobian_a_, KDL::Vector(v1(0), v1(1), v1(2)), J_v1);
      J_v1.data=J_v1.data - jacobian_a_.data;
      changeJacRefPoint(jacobian_b_, KDL::Vector(vk(0), vk(1), vk(2)), J_vk);
      J_vk.data=J_vk.data - jacobian_b_.data;
      changeJacRefPoint(jacobian_a_, p1__, J_p1);
      changeJacRefPoint(jacobian_b_, k__, J_dp);
      J_dp.data=J_p1.data - J_dp.data;
      J_v2.data.topRows<3>()=vk*(vk.transpose()*J_dp.data.topRows<3>()+dp.transpose()*J_vk.data.topRows<3>())+dp.dot(vk)*J_vk.data.topRows<3>()-J_dp.data.topRows<3>();
   
      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, KDL::Vector(v1(0), v1(1), v1(2)), J_v1_dot);
      J_v1_dot.data = J_v1_dot.data - jacobian_dot_a_.data;
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, KDL::Vector(vk(0), vk(1), vk(2)), J_vk_dot);
      J_vk_dot.data = J_vk_dot.data - jacobian_dot_b_.data;
      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, p1__, J_p1_dot);
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, k__, J_dp_dot);
      J_dp_dot.data = J_p1_dot.data-J_dp_dot.data;

      Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
      Eigen::Vector3d v1_dot=J_v1.data.topRows<3>()*qdot;
      Eigen::Vector3d v2_dot=J_v2.data.topRows<3>()*qdot;
      Eigen::Vector3d vk_dot=J_vk.data.topRows<3>()*qdot;
      Eigen::Vector3d dp_dot=J_dp.data.topRows<3>()*qdot;            

      J_v2_dot.data.topRows<3>() = vk_dot*(vk.transpose()*J_dp.data.topRows<3>()+dp.transpose()*J_vk.data.topRows<3>())+vk*(vk_dot.transpose()*J_dp.data.topRows<3>()+vk.transpose()*J_dp_dot.data.topRows<3>()+dp_dot.transpose()*J_vk.data.topRows<3>()+dp.transpose()*J_vk_dot.data.topRows<3>())-J_dp_dot.data.topRows<3>()+dp.dot(vk)*J_vk_dot.data.topRows<3>()+(dp_dot.dot(vk)+dp.dot(vk_dot))*J_vk.data.topRows<3>();

      //CROSS PRODUCT =============================================      
       return rotateVectors(v1, v2, v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());
      //CROSS PRODUCT END =========================================

      //DOT PRODUCT =============================================      
      //return alignUnitVectorVector(qdot,v1, v2, v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());
      //DOT PRODUCT END =========================================

    }

    template <>
    int TDefGeometricAlignment<GeometricLine, GeometricSphere>::align(std::shared_ptr<GeometricLine> line,
								      std::shared_ptr<GeometricSphere> sphere,
								      const RobotStatePtr robot_state) {
      KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();
      KDL::Vector p1__ =  pose_a_.M * line->getOffsetKDL();
      KDL::Vector p2__ =  pose_b_.M * sphere->getCenterKDL();
      KDL::Vector p1 = pose_a_.p + p1__;
      KDL::Vector p2 = pose_b_.p + p2__;
      KDL::Vector v2 = p2 - p1;

      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_v1, J_p1, J_v2, J_v1_dot, J_v2_dot, J_p1_dot;
      J_v1.resize(q_nr);
      J_v2.resize(q_nr);
      J_p1.resize(q_nr);
      J_p1_dot.resize(q_nr);    
      J_v1_dot.resize(q_nr);
      J_v2_dot.resize(q_nr);
    
      changeJacRefPoint(jacobian_a_, v1, J_v1);
      J_v1.data=J_v1.data - jacobian_a_.data;

      changeJacRefPoint(jacobian_a_,p1__,J_p1);
      changeJacRefPoint(jacobian_b_,p2__,J_v2);
      J_v2.data=J_v2.data-J_p1.data;

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, v1, J_v1_dot);
      J_v1_dot.data = J_v1_dot.data - jacobian_dot_a_.data;

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, p1__, J_p1_dot);
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, p2__, J_v2_dot);
      J_v2_dot.data = J_v2_dot.data - J_p1_dot.data;
      
      
      Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
      Eigen::Vector3d v1_dot=J_v1.data.topRows<3>()*qdot;
      Eigen::Vector3d v2_dot=J_v2.data.topRows<3>()*qdot;

      //CROSS PRODUCT ================================================
       return rotateVectors(Eigen::Vector3d(v1(0),v1(1),v1(2)), Eigen::Vector3d(v2(0),v2(1),v2(2)), v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());
      //CROSS PRODUCT END ============================================

      //DOT PRODUCT ================================================
       // return alignUnitVectorVector(qdot, Eigen::Vector3d(v1(0),v1(1),v1(2)), Eigen::Vector3d(v2(0),v2(1),v2(2)), v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());
      //DOT PRODUCT END ============================================
 
    }

    template <>
    int TDefGeometricAlignment<GeometricFrame, GeometricFrame>::align(std::shared_ptr<GeometricFrame> frame1,
								      std::shared_ptr<GeometricFrame> frame2,
								      const RobotStatePtr robot_state) {

      //CROSS PRODUCT =============================================
      KDL::Vector v1 = pose_a_.M * frame1->getAxisXKDL();
      KDL::Vector v2 = pose_b_.M * frame2->getAxisXKDL();

      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_v1, J_p1, J_v2, J_v1_dot, J_v2_dot;
      J_v1.resize(q_nr);
      J_v2.resize(q_nr);
      J_p1.resize(q_nr);
      J_v1_dot.resize(q_nr);
      J_v2_dot.resize(q_nr);
    
      changeJacRefPoint(jacobian_a_, v1, J_v1);
      J_v1.data=J_v1.data - jacobian_a_.data;

      changeJacRefPoint(jacobian_b_, v2, J_v2);
      J_v2.data=J_v2.data - jacobian_b_.data;

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, v1, J_v1_dot);
      J_v1_dot.data = J_v1_dot.data - jacobian_dot_a_.data;

      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, v2, J_v2_dot);
      J_v2_dot.data = J_v2_dot.data - jacobian_dot_b_.data;
      
      Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
      Eigen::Vector3d v1_dot=J_v1.data.topRows<3>()*qdot;
      Eigen::Vector3d v2_dot=J_v2.data.topRows<3>()*qdot;
      
      //abuse the rotateVectors function to compute the relevant quantities to align both axis
      rotateVectors(Eigen::Vector3d(v1(0),v1(1),v1(2)), Eigen::Vector3d(v2(0),v2(1),v2(2)), v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());

      //temporary save the task errors/jacobians
      Eigen::VectorXd ey=e_;
      Eigen::VectorXd ey_dot=e_dot_;  
      Eigen::MatrixXd Jy=J_;
      Eigen::MatrixXd Jy_dot=J_dot_;

      v1 = pose_a_.M * frame1->getAxisYKDL();
      v2 = frame2->getAxisYKDL();
      
      changeJacRefPoint(jacobian_a_, v1, J_v1);
      J_v1.data=J_v1.data - jacobian_a_.data;

      changeJacRefPoint(jacobian_b_, v2, J_v2);
      J_v2.data=J_v2.data - jacobian_b_.data;

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, v1, J_v1_dot);
      J_v1_dot.data = J_v1_dot.data - jacobian_dot_a_.data;

      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, v2, J_v2_dot);
      J_v2_dot.data = J_v2_dot.data - jacobian_dot_b_.data;
      
      v1_dot=J_v1.data.topRows<3>()*qdot;
      v2_dot=J_v2.data.topRows<3>()*qdot;

      //overwrite the class member errors/jacobians
      rotateVectors(Eigen::Vector3d(v1(0),v1(1),v1(2)), Eigen::Vector3d(v2(0),v2(1),v2(2)), v1_dot, v2_dot, J_v1.data.topRows<3>(), J_v2.data.topRows<3>(), J_v1_dot.data.topRows<3>(), J_v2_dot.data.topRows<3>());

      //append the previously computed quantities
      e_.conservativeResize(2);
      e_(1)=ey(0);
      e_dot_.conservativeResize(2);
      e_dot_(1)=ey_dot(0);
      J_.conservativeResize(2,q_nr);
      J_.bottomRows<1>()=Jy;
      J_dot_.conservativeResize(2,q_nr);
      J_dot_.bottomRows<1>()=Jy_dot;
      //CROSS PRODUCT END ==========================================

      //DOT PRODUCT =============================================      
      // KDL::Vector ax1 = pose_a_.M * frame1->getAxisXKDL();
      // KDL::Vector ax2 = pose_b_.M * frame2->getAxisXKDL();
      // KDL::Vector ay1 = pose_a_.M * frame1->getAxisYKDL();
      // KDL::Vector ay2 = pose_b_.M * frame2->getAxisYKDL();

      // //abuse the alignUnitVectors function to compute the relevant quantities to align both axis
      // alignUnitVectors(ay1, ay2, robot_state);

      // //temporary save the task errors/jacobians
      // Eigen::VectorXd ey=e_;
      // Eigen::VectorXd ey_dot=e_dot_;  
      // Eigen::MatrixXd Jy=J_;
      // Eigen::MatrixXd Jy_dot=J_dot_;  
      // double q_nr=J_.cols();

      // //overwrite the class member errors/jacobians
      // alignUnitVectors(ax1, ax2, robot_state);

      // //append the previously computed quantities
      // e_.conservativeResize(2);
      // e_(1)=ey(0);
      // e_dot_.conservativeResize(2);
      // e_dot_(1)=ey_dot(0);
      // J_.conservativeResize(2,q_nr);
      // J_.bottomRows<1>()=Jy;
      // J_dot_.conservativeResize(2,q_nr);
      // J_dot_.bottomRows<1>()=Jy_dot;
      //DOT PRODUCT END =============================================      
      return 0;
    }

  }  // namespace tasks

}  // namespace hiqp
