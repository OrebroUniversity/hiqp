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

#ifndef KFILTER_IMPL_HPP
#define KFILTER_IMPL_HPP

//! \file
//! \brief Contains the implementation of the \c KFilter base template class.

namespace Kalman {
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  KFilter<T, BEG, OQ, OVR, DBG>::~KFilter() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeBaseB() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeB() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeProcess() {


    
    // x = Ax + Bu + Ww    n.1 = n.n * n.1 + n.nu * nu.1
    makeB();

    K_UINT_32 i, j;
    x__.resize(this->n);

    for (i = BEG; i < this->n + BEG; ++i) {

      x__(i) = T(0.0);

      for (j = BEG; j < this->n + BEG; ++j)
        x__(i) += this->A(i,j) * this->x(j);

      for (j = BEG; j < this->nu + BEG; ++j)
        x__(i) += B(i,j) * this->u(j);

    }

    this->x.swap(x__);

  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::makeMeasure() {
    
    // z = Hx + Vv
    K_UINT_32 i, j;

    this->z.resize(this->m);
    for (i = BEG; i < this->m + BEG; ++i) {

      this->z(i) = T(0.0);

      for (j = BEG; j < this->n + BEG; ++j)
        this->z(i) += this->H(i,j) * this->x(j);

    }

  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void KFilter<T, BEG, OQ, OVR, DBG>::sizeUpdate() {

    if (this->flags & ( KALMAN_N_MODIFIED | KALMAN_NU_MODIFIED ) ) {
      B.resize(this->n, this->nu);
      makeBaseB();
    }   
    
    EKFilter<T, BEG, OQ, OVR, DBG>::sizeUpdate();
  }

}

#endif
