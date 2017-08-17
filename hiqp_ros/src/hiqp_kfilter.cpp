#include <hiqp_ros/kalman/hiqp_kfilter.hpp>
#include <ros/console.h>

namespace hiqp_ros {
  // //-----------------------------------------------------
  HiQPKFilter::HiQPKFilter() : dt_(0.0), initialized_(false) {};
  // //-----------------------------------------------------  
  // HiQPKFilter::HiQPKFilter(double dt) : dt_(dt) {};
  //-----------------------------------------------------
  void HiQPKFilter::init( KFVector& _x0,  KFMatrix& _P0, KFMatrix& _R, KFMatrix& _Q, double _dt){
    this->Kalman::KFilter<double,0>::init(_x0,_P0);
    R=_R;
    Q=_Q;
    dt_=_dt;
    initialized_ = true;
  }
  // //-----------------------------------------------------
  bool HiQPKFilter::isInitialized(){
    return initialized_;
  }
  // //-----------------------------------------------------  
  // double HiQPKFilter::getSamplingTime(){
  //   return dt_;
  // }
  //-----------------------------------------------------
  void HiQPKFilter::makeA(){
    A(0,0) = 1.0;
    A(0,1) = dt_;
    A(1,0) = 0.0;
    A(1,1) = 1.0;
  };
  //-----------------------------------------------------  
  void HiQPKFilter::makeH(){
     H(0,0) = 1.0;
    H(0,1) = 0.0;
    H(1,0) = 0.0;
    H(1,1) = 1.0;
  };
  //-----------------------------------------------------
  
  void HiQPKFilter::makeV(){
     V(0,0) = 1.0;
    V(0,1) = 0.0;
    V(1,0) = 0.0;
    V(1,1) = 1.0;
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeW(){
     W(0,0) = 1.0;
    W(0,1) = 0.0;
    W(1,0) = 0.0;
    W(1,1) = 1.0;
  };
    //-----------------------------------------------------
  void HiQPKFilter::makeR(){
   ROS_ERROR("R SCHAS");
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeQ(){
   ROS_ERROR("Q SCHAS");
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeBaseB(){
    std::cerr<<"calling makeBaseb."<<std::endl;
    ROS_ERROR("SCHAS");
    //    NoModification();
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeB(){
    std::cerr<<"calling makeBase. "<<std::endl;
    ROS_ERROR("VOISCHAS");
    B(0,0) = 0.0;
    B(1,0) = dt_;
  };
  //-----------------------------------------------------
  
}
