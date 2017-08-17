// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           École Polytechnique de Montréal
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef KFILTER_HPP
#define KFILTER_HPP

//! \file
//! \brief Contains the interface of the \c KFilter base template class.

#include <hiqp_ros/kalman/ekfilter.hpp>

namespace Kalman {

  // TODO : il faut que E(v) == 0 && E(w) == 0 !!!

  //! Generic Linear Kalman Filter template base class.

  //! \par Usage
  //! This class is derived from \c EKFilter. You should really read all the
  //! documentation of \c EKFilter before reading this. \n
  //! This class implements a Variable-Dimension Linear Kalman Filter
  //! based on the \c EKFilter template class.
  //!
  //! \par Notation
  //! Assume a state vector
  //! \f$ x \f$ (to estimate) and a linear process 
  //! function \f$ f \f$ (to model) that describes the
  //! evolution of this state through time, that is :
  //! \f[ x_k = f \left( x_{k-1}, u_{k-1}, w_{k-1} \right) = 
  //!     A x_{k-1} + B u_{k-1} + W w_{k-1} \f]
  //! where \f$ u \f$ is the (known) input vector fed to the process and 
  //! \f$ w \f$ is the (unknown) process noise vector due to uncertainty
  //! and process modeling errors. Further suppose that the (known) process
  //! noise covariance matrix is : \f[ Q = E \left( w w^T \right) \f]
  //! Now, let's assume a (known) measurement vector \f$ z \f$, which depends 
  //! on the current state \f$ x \f$ in the form of a linear function
  //! \f$ h \f$ (to model) : \f[ z_k = h \left( x_k, v_k \right) =
  //! H x_k + V v_k \f]
  //! where \f$ v \f$ is the (unknown) measurement noise vector with
  //! a (known) covariance matrix : \f[ R = E \left( v v^T \right) \f]
  //! Suppose that we have an estimate of the previous state 
  //! \f$ \hat{x}_{k-1} \f$, called a corrected state or an
  //! <em>a posteriori</em> state estimate. We can build a predicted state
  //! (also called an <em>a priori</em> state estimate) by using \f$ f \f$ :
  //! \f[ \tilde{x}_k = f \left( \hat{x}_{k-1}, u_{k-1}, 0 \right) =
  //!     A \hat{x}_{k-1} + B u_{k-1}  \f]
  //! since the input is known and the process noise, unknown. With this
  //! predicted state, we can get a predicted measurement vector by
  //! using \f$ h \f$ : \f[ \tilde{z}_k = h \left( \tilde{x}_k, 0 \right) =
  //! H \tilde{x}_k \f]
  //! since the measurement noise is unknown.
  //!
  //! \note In this class, \c makeProcess() and \c makeMeasure() have already
  //! been overridden, and cannot be ovverridden again. However, there is a
  //! new matrix to create : \a B. This means there are two new virtual
  //! functions that can be overridden : \c makeBaseB() and \c makeB().
  //! \see \c EKFilter
  template<typename T, K_UINT_32 BEG, bool OQ = false, 
           bool OVR = false, bool DBG = true>
  class KFilter : public EKFilter<T, BEG, OQ, OVR, DBG> {
    
    using typename EKFilter<T, BEG, OQ, OVR, DBG>::Matrix;
    using typename EKFilter<T, BEG, OQ, OVR, DBG>::Vector;    
    
  public:

    //! Virtual destructor.
    virtual ~KFilter() = 0;
    
  protected:

    //! Virtual pre-creator of \a B.
    virtual void makeBaseB();

    //! Virtual creator of \a B.
    virtual void makeB();
    
    //! Input matrix.
    Matrix B;

  private:

    //! Process function overridden to be linear.
    virtual void makeProcess();

    //! Measurement function overridden to be linear.
    virtual void makeMeasure();

    //! Matrix and vector resizing function, overridden to take B into account.
    virtual void sizeUpdate();

    //! Temporary vector.
    Vector x__;
  };

}

#include <hiqp_ros/kalman/kfilter_impl.hpp>
#endif
