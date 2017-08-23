#include <hiqp_ros/kalman/hiqp_kfilter.hpp>
#include <ros/console.h>
#include <Eigen/Dense>

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
  //-----------------------------------------------------  
  double HiQPKFilter::getSamplingTime(){
    return dt_;
  }
  //-----------------------------------------------------  
  void HiQPKFilter::setSamplingTime(double dt){
    dt_=dt;
  }
  //-----------------------------------------------------
  void HiQPKFilter::makeA(){
    Eigen::MatrixXd A_b(2,2);
    A_b(0,0) = 1.0;
    A_b(0,1) = dt_;
    A_b(1,0) = 0.0;
    A_b(1,1) = 1.0;

    Eigen::MatrixXd _A = Eigen::MatrixXd::Zero(n,n);
    for(unsigned int i=0; i<nu;i++)
      _A.block<2,2>(2*i, 2*i)=A_b.transpose();

    A.assign(n,n,_A.data());
  };
  //-----------------------------------------------------  
  void HiQPKFilter::makeH(){
    Eigen::MatrixXd _H = Eigen::MatrixXd::Identity(n,n);
    H.assign(n,n,_H.data());
  };
  //-----------------------------------------------------
  
  void HiQPKFilter::makeV(){
    Eigen::MatrixXd _V = Eigen::MatrixXd::Identity(n,n);
    V.assign(n,n,_V.data());
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeW(){
    Eigen::MatrixXd _W = Eigen::MatrixXd::Identity(n,n);
    W.assign(n,n,_W.data());
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeR(){
    //            std::cerr<<"R: "<<std::endl;
    // for(unsigned int i=0;i<n;i++){
    //   for(unsigned int j=0;j<n;j++)
    // 	std::cerr<<R(i,j)<<" ";

    //   std::cerr<<std::endl;
    // }
    // std::cerr<<std::endl;
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeQ(){
    //            std::cerr<<"Q: "<<std::endl;
    // for(unsigned int i=0;i<n;i++){
    //   for(unsigned int j=0;j<n;j++)
    // 	std::cerr<<Q(i,j)<<" ";

    //   std::cerr<<std::endl;
    // }
    // std::cerr<<std::endl;
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeBaseB(){
    Eigen::MatrixXd _B=Eigen::MatrixXd::Zero(n,nu);
    for(unsigned int i=0; i<nu; i++)
      B(2*i+1,i)=dt_;
  };
  //-----------------------------------------------------
  void HiQPKFilter::makeB(){
    Eigen::MatrixXd _B=Eigen::MatrixXd::Zero(n,nu);
    for(unsigned int i=0; i<nu; i++)
      B(2*i+1,i)=dt_;

  };
  //-----------------------------------------------------
  
}
