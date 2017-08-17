#ifndef HIQP_KFILTER_HPP
#define HIQP_KFILTER_HPP

#include <hiqp_ros/kalman/kfilter.hpp>

namespace hiqp_ros {
     typedef Kalman::KVector< double, 0, true > KFVector;
  typedef Kalman::KMatrix< double, 0, true > KFMatrix;
  
  class HiQPKFilter : public Kalman::KFilter<double,0> {

  public:
    HiQPKFilter();
    //HiQPKFilter(double dt);

    // void setSamplingTime(double dt);
    // double getSamplingTime();
    bool isInitialized();
    void init( KFVector& x0,  KFMatrix& P0, KFMatrix& R, KFMatrix& Q, double dt);
  protected:

    void makeA();
    void makeH();
    void makeV();
    void makeR();
    void makeW();
    void makeQ();
    void makeBaseB();
    void makeB();  

  private:
    double dt_;
    bool initialized_;

  };
  

  // typedef HiQPKFilter::Matrix Matrix;
}
#endif
