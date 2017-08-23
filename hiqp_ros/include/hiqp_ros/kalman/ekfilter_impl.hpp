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

#ifndef EKFILTER_IMPL_HPP
#define EKFILTER_IMPL_HPP

//! \file
//! \brief Contains the implementation of the \c EKFilter base template class.

//! \internal 
//! Flag : \a n has changed
#define KALMAN_N_MODIFIED    1

//! \internal
//! Flag : \a nu has changed
#define KALMAN_NU_MODIFIED  (1<<1)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NW_MODIFIED  (1<<2)

//! \internal
//! Flag : \a m has changed
#define KALMAN_M_MODIFIED   (1<<3)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NV_MODIFIED  (1<<4)

//! \internal
//! Flag : \a P has changed
#define KALMAN_P_MODIFIED   (1<<5)

//! \internal
//! Mask : used to reset dimension flags
#define KALMAN_LOWMASK      ((1<<8) - 1)

//! \internal
//! Flag : \a A has changed
#define KALMAN_A_MODIFIED   (1<<8)

//! \internal
//! Flag : \a W has changed
#define KALMAN_W_MODIFIED   (1<<9)

//! \internal
//! Flag : \a Q has changed
#define KALMAN_Q_MODIFIED   (1<<10)

//! \internal
//! Mask : used to reset time update matrix flags
#define KALMAN_MIDMASK      ( ((1<<4) - 1) << 8 )

//! \internal
//! Flag : \a H has changed
#define KALMAN_H_MODIFIED   (1<<12)

//! \internal
//! Flag : \a V has changed
#define KALMAN_V_MODIFIED   (1<<13)

//! \internal
//! Flag : \a R has changed
#define KALMAN_R_MODIFIED   (1<<14)

//! \internal
//! Mask : used to reset measure update matrix flags
#define KALMAN_HIGHMASK     ( ((1<<4) - 1) << 12 )

namespace Kalman {

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  EKFilter<T, BEG, OQ, OVR, DBG>::EKFilter()
    : flags(0) {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  EKFilter<T, BEG, OQ, OVR, DBG>::EKFilter(K_UINT_32 n_, K_UINT_32 nu_, 
                                           K_UINT_32 nw_, K_UINT_32 m_, 
                                           K_UINT_32 nv_)
    : flags(0) {
    setDim(n_, nu_, nw_, m_, nv_);
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  EKFilter<T, BEG, OQ, OVR, DBG>::~EKFilter() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::setDim(K_UINT_32 n_, K_UINT_32 nu_, 
                                              K_UINT_32 nw_, K_UINT_32 m_, 
                                              K_UINT_32 nv_) {
    setSizeX(n_);
    setSizeU(nu_);
    setSizeW(nw_);
    setSizeZ(m_);
    setSizeV(nv_);
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  K_UINT_32 EKFilter<T, BEG, OQ, OVR, DBG>::getSizeX() const {
    return n;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  K_UINT_32 EKFilter<T, BEG, OQ, OVR, DBG>::getSizeU() const {
    return nu;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  K_UINT_32 EKFilter<T, BEG, OQ, OVR, DBG>::getSizeW() const {
    return nw;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  K_UINT_32 EKFilter<T, BEG, OQ, OVR, DBG>::getSizeZ() const {
    return m;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  K_UINT_32 EKFilter<T, BEG, OQ, OVR, DBG>::getSizeV() const {
    return nv;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::setSizeX(K_UINT_32 n_) {

    // verify : n_ > 0

    if (n_ != n) {
      flags |= KALMAN_N_MODIFIED;
      n = n_;
    }
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::setSizeU(K_UINT_32 nu_) {
    if (nu_ != nu) {
      flags |= KALMAN_NU_MODIFIED;
      nu = nu_;
    }
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::setSizeW(K_UINT_32 nw_) {
    if (nw_ != nw) {
      flags |= KALMAN_NW_MODIFIED;
      nw = nw_;
    }
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::setSizeZ(K_UINT_32 m_) {
    if (m_ != m) {
      flags |= KALMAN_M_MODIFIED;
      m = m_;
    }
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::setSizeV(K_UINT_32 nv_) {
    if (nv_ != nv) {
      flags |= KALMAN_NV_MODIFIED;
      nv = nv_;
    }
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::init(Vector& x_, Matrix& P_) {

    // verify : (x_.size() == n && P_.nrow() == n && P_.ncol() == n)

    x.swap(x_);
    _P.swap(P_);
    flags |= KALMAN_P_MODIFIED;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::step(Vector& u_, const Vector& z_) {
    timeUpdateStep(u_);
    measureUpdateStep(z_);
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::timeUpdateStep(Vector& u_) {

    // verif : u_.size() == nu
    K_UINT_32 i, j, k;

    sizeUpdate();
    u.swap(u_);
    
    makeCommonProcess();
    makeAImpl();
    makeWImpl();
    makeQImpl();
    makeProcess();

    if (!OQ) {

      if (flags & KALMAN_Q_MODIFIED) {

        Q_ = Q;
        factor(Q_);
        upperInvert(Q_);

      }

      Q.swap(Q_);
      
      // W_ = W*U   n.nw = n.nw * nw.nw

      if (flags & ( KALMAN_W_MODIFIED | KALMAN_Q_MODIFIED ) ) {

        for (i = BEG; i < n + BEG; ++i) {
    
          for (j = BEG; j < nw + BEG; ++j) {
      
            W_(i,j) = W(i,j);
            for (k = BEG; k < j; ++k)
              W_(i,j) += W(i,k)*Q(j,k);
      
          }
    
        }
  
      }
      
      W.swap(W_);
  
    }

    timeUpdate();

    if (!OQ) {
      Q.swap(Q_);
      W.swap(W_);
    }

    u.swap(u_);
    flags &= ~KALMAN_MIDMASK;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::measureUpdateStep(const Vector& z_) {

    // verif : z_.size() == m
    K_UINT_32 i, j, k;

    sizeUpdate();

    if (m == 0) {
      return;
    }
    
    makeCommonMeasure();
    makeHImpl();
    makeVImpl();
    makeRImpl();    
    makeMeasure();
    
    // verif : nv != 0

    for (i = BEG; i < m + BEG; ++i)
      dz(i) = z_(i) - z(i);

    makeDZ();

    if (OVR) {

      // verif : m == nv

      if (flags & ( KALMAN_V_MODIFIED | KALMAN_R_MODIFIED ) ) {

        for (i = BEG; i < m + BEG; ++i)
          R_(i,i) = V(i,i)*V(i,i)*R(i,i);

      }

    } else {


      if (flags & ( KALMAN_V_MODIFIED | KALMAN_R_MODIFIED ) ) { // calculate R_

        _x.resize(nv);
      
        // R_ = V*R*V'
        for (i = BEG; i < m + BEG; ++i) {

          // _x = row i of V*R = (V*R)(i,:)
          for (j = BEG; j < nv + BEG; ++j) {

            _x(j) = T(0.0);
            for (k = BEG; k < nv + BEG; ++k)
              _x(j) += V(i,k)*R(k,j);

          }

          // R_(i,:) = (V*R*V')(i,:) = (V*R)(i,:) * V'
          for (j = BEG; j < m + BEG; ++j) {

            R_(i,j) = T(0.0);
            for (k = BEG; k < nv + BEG; ++k)
              R_(i,j) += _x(k)*V(j,k);

          }

        }

        // R_ = U*D*U'
        // diag(R_) = D, upper(R_) = U, lower(R_) = junk
        factor(R_);

        // lower(R_) = (inv(U))'
        upperInvert(R_);

      }

      if (flags & ( KALMAN_H_MODIFIED | KALMAN_V_MODIFIED | KALMAN_R_MODIFIED ) ) { // calculate H_

        // H_ = inv(U)*H    m.n = m.m * m.n
        for (i = BEG; i < m + BEG; ++i) {

          for (j = BEG; j < n + BEG; ++j) {

            H_(i,j) = H(i,j);
            for (k = i + 1; k < m + BEG; ++k)
              H_(i,j) += R_(k,i)*H(k,j);

          }

        }

      }

      H.swap(H_);

      // _x = inv(U)*dz    m.1 = m.m * m.1
      _x.resize(m);

      for (i = BEG; i < m + BEG; ++i) {

        _x(i) = dz(i);
        for (k = i + 1; k < m + BEG; ++k)
          _x(i) += R_(k,i)*dz(k); 

      }

      dz.swap(_x);

    }
    
    _x.resize(n); // dx : innovation
    _x = T(0.0);
    for (i = BEG; i < m + BEG; ++i) {

      for (j = BEG; j < n + BEG; ++j)
        a(j) = H(i,j);

      measureUpdate(dz(i), R_(i,i));

    }
    for (i = BEG; i < n + BEG; ++i)
      x(i) += _x(i);

    if (!OVR) {
      H.swap(H_);
    }

    flags &= ~KALMAN_HIGHMASK;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  const KTYPENAME EKFilter<T, BEG, OQ, OVR, DBG>::Vector& EKFilter<T, BEG, OQ, OVR, DBG>::predict(Vector& u_) {
    
    // verif : u_.size() == nu

    sizeUpdate();
    u.swap(u_);   
    _x = x;
    
    makeCommonProcess();
    makeProcess();
    
    x.swap(_x);
    u.swap(u_);
    return _x;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  const KTYPENAME EKFilter<T, BEG, OQ, OVR, DBG>::Vector& EKFilter<T, BEG, OQ, OVR, DBG>::simulate() {
    
    sizeUpdate();
    _x = z;
    
    makeCommonMeasure();
    makeMeasure();
    
    z.swap(_x);
    return _x;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  const KTYPENAME EKFilter<T, BEG, OQ, OVR, DBG>::Vector& EKFilter<T, BEG, OQ, OVR, DBG>::getX() const {
    return x;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  const KTYPENAME EKFilter<T, BEG, OQ, OVR, DBG>::Matrix& EKFilter<T, BEG, OQ, OVR, DBG>::calculateP() const {

    if (!(flags & KALMAN_P_MODIFIED)) {

      _P.resize(n, n);         // keep this resize
    
      for (K_UINT_32 i = BEG; i < n + BEG; ++i) {

        _P(i,i) = U(i,i);

        for (K_UINT_32 j = i + 1; j < n + BEG; ++j) {

          _P(i,j)  = U(i,j)*U(j,j);
          _P(i,i) += U(i,j)*_P(i,j);

          for (K_UINT_32 k = j + 1; k < n + BEG; ++k) {
            _P(i,j) += U(i,k)*U(j,k)*U(k,k);
          }

          _P(j,i) = _P(i,j);

        }

      }

    }

    return _P;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::NoModification() {
    modified_ = false;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseA() {
    NoModification();
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseW() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseQ() {
    NoModification();
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseH() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseV() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseR() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeCommonProcess() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeCommonMeasure() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeA() {
    NoModification();
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeW() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeQ() {
    NoModification();
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeH() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeV() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeR() {
    NoModification();
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeDZ() {}

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::sizeUpdate() {
    
    if (!flags) {
      return;
    }

    if (flags & KALMAN_N_MODIFIED) {
      A.resize(n, n);
      makeBaseAImpl();
    }

    if (flags & (KALMAN_N_MODIFIED | KALMAN_NW_MODIFIED) ) {
      nn = n + nw;
      a.resize(nn);
      v.resize(nn);
      d.resize(nn);
      if (!OQ)
        W_.resize(n, nw);
      W.resize(n, nw);
      makeBaseWImpl();
    }

    // KALMAN_N_MODIFIED imply KALMAN_P_MODIFIED
    // => KALMAN_N_MODIFIED must not be set OR KALMAN_P_MODIFIED must be set
    // => NOT  KALMAN_N_MODIFIED  OR  KALMAN_P_MODIFIED  must be set
    // verif : (flags ^ KALMAN_N_MODIFIED) & 
    //              (KALMAN_N_MODIFIED | KALMAN_P_MODIFIED)

    if (flags & KALMAN_P_MODIFIED) { 
      // this covers the case of KALMAN_N_MODIFIED = true also

      // We have a new matrix P : let's factorize it and store it in U
      // First, resize U and copy P in its left part
      U.resize(n, nn);
      for (K_UINT_32 i = BEG; i < n + BEG; ++i)
        for (K_UINT_32 j = BEG; j < n + BEG; ++j)
          U(i,j) = _P(i,j);
      
      // Factorize
      factor(U);

    } else if (flags & KALMAN_NW_MODIFIED) {
      // KALMAN_N_MODIFIED is necessarily false, else KALMAN_P_MODIFIED
      // would have been true

      // Let's just copy U in temporary matrix _P of the right size,
      // then swap the matrices
      _P.resize(n, nn);
      for (K_UINT_32 i = BEG; i < n + BEG; ++i)
        for (K_UINT_32 j = i; j < n + BEG; ++j)
          _P(i,j) = U(i,j);
      U.swap(_P);
    }

    if (flags & KALMAN_NW_MODIFIED) {
      if (!OQ)
        Q_.resize(nw, nw);
      Q.resize(nw, nw);
      makeBaseQImpl();
    }

    if (m != 0) {

      if (flags & (KALMAN_N_MODIFIED | KALMAN_M_MODIFIED) ) {
        if (!OVR)
          H_.resize(m, n);
        H.resize(m, n);
        makeBaseHImpl();
      }

      if (flags & (KALMAN_M_MODIFIED | KALMAN_NV_MODIFIED) ) {
        V.resize(m, nv);
        makeBaseVImpl();
      }

      if (flags & KALMAN_NV_MODIFIED) {
        R.resize(nv, nv);
        makeBaseRImpl();
      }

      if (flags & KALMAN_M_MODIFIED) {
        R_.resize(m, m);
        z.resize(m);
        dz.resize(m);
      }

    }
    
    flags &= ~KALMAN_LOWMASK;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::factor(Matrix& P_) {

    // ne pas vérifier que P_.ncol() == P_.nrow(), comme ça, même si
    // nrow() < ncol(), on peut factoriser la sous-matrice carrée de P
    // Utile pour factoriser U

    T alpha, beta;
    K_UINT_32 i, j, k, N = P_.nrow();
    for(j = N - 1 + BEG; j > BEG; --j) {
      alpha = T(1.0)/P_(j,j);
      for(k = BEG; k < j; ++k) {
        beta = P_(k,j);
        P_(k,j) = alpha*beta;
        for(i = BEG; i <= k; ++i)
          P_(i,k) -= beta*P_(i,j);
      }
    }
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::upperInvert(Matrix& P_) {

    T val;
    K_UINT_32 i, j, k, N = P_.nrow();
    for (i = N - 2 + BEG; i != (K_UINT_32)(BEG-1); --i) { // intended overflow if BEG==0
      for (k = i + 1; k < N + BEG; ++k) {

        val = P_(i,k);
        for (j = i + 1; j <= k - 1; ++j)
          val += P_(i,j)*P_(k,j);
        P_(k,i) = -val;

      }
    }

  }

  // U    u     U-D covariance matrix (n,nn)
  // A    phi   transition matrix (F) (n,n)
  // W    g     process noise matrix (G) (n,nw)
  // Q    q     process noise variance vector (nw) Q = diag(q)
  // a, v, d temporary vectors
  // U is updated
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::timeUpdate() {

    K_UINT_32 i, j, k;
    T sigma, dinv;
  
    // U = phi * U
    // d = diag(U)
    // 
    // This algo could be faster
    // if phi is known to be diagonal
    // It could be almost zapped if phi=I
    for(j = n - 1 + BEG; j > BEG; --j) {
      for(i = BEG; i <= j; ++i)
        d(i) = U(i,j);
      for(i = BEG; i < n + BEG; ++i) {
        U(i,j) = A(i,j);
        for(k = BEG; k < j; ++k)
          U(i,j) += A(i,k)*d(k);
      }
    }

    d(BEG) = U(BEG,BEG);
    for(j = BEG; j < n + BEG; ++j)
      U(j,BEG) = A(j,BEG);

    // d(n+1:nn) = q 
    // U(:,n+1:nn) = G 
    for(i = BEG; i < nw + BEG; ++i) {
      d(i+n) = Q(i,i);
      for(j = BEG; j < n + BEG; ++j)
        U(j,i+n) = W(j,i);
    }

    // Gram-Schmidt
    // Too hard to simplify
    for(j = n - 1 + BEG; j != (K_UINT_32)(BEG-1); --j) { // intended overflow if BEG==0
      sigma = T(0.0);
      for(k = BEG; k < nn + BEG; ++k) {
        v(k) = U(j,k);
        a(k) = d(k)*v(k);
        sigma += v(k)*a(k);
      }
      U(j,j) = sigma;
      if(j == BEG || sigma == T(0.0)) continue;
      dinv = T(1.0)/sigma;
      for(k = BEG; k < j; ++k) {
        sigma = T(0.0);
        for(i = BEG; i < nn + BEG; ++i) 
          sigma += U(k,i)*a(i);
        sigma *= dinv;
        for(i = BEG; i < nn + BEG; ++i) 
          U(k,i) -= sigma*v(i);
        U(j,k) = sigma;
      }
    }

    // U = transpose(U)
    for(j = BEG + 1; j < n + BEG; ++j)
      for(i = BEG; i < j; ++i)
        U(i,j) = U(j,i);
  }

  // x     a priori estimate vector (n)
  // U     a priori U-D covariance matrix (n,nn)
  // dz    measurement diff (z - ax) (scalar)
  // a     measurement coefficients vector (n) (a row of A, which is H)
  //          a is destroyed
  // r     measurement variance
  // d is a temporary vector
  // x and U are updated
  // a is destroyed
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::measureUpdate(T dz, T r) {

    K_UINT_32 i, j, k;
    T alpha, gamma, beta, lambda;

    // dz = dz - Hdx
    for (j = BEG; j < n + BEG; ++j)
      dz -= a(j)*_x(j);
    
    // d = D * transpose(U) * a
    // a =     transpose(U) * a
    //
    // This algo could be faster
    // if A is known to be diagonal or I
    for(j = n - 1 + BEG; j > BEG; --j) {
      for(k = BEG; k < j; ++k)
        a(j) += U(k,j)*a(k);
      d(j) = U(j,j)*a(j);
    }
    d(BEG) = U(BEG,BEG)*a(BEG);

    // UDU
    // Too hard to simplify
    alpha = r+d(BEG)*a(BEG);
    gamma = T(1.0)/alpha;
    U(BEG,BEG) = r*gamma*U(BEG,BEG);
    for(j = BEG + 1; j < n + BEG; ++j) {
      beta = alpha;
      alpha += d(j)*a(j);
      lambda = -a(j)*gamma;
      gamma = T(1.0)/alpha;
      U(j,j) *= beta*gamma;
      for(i = BEG; i < j; ++i) {
        beta = U(i,j);
        U(i,j) = beta+d(i)*lambda;
        d(i) += d(j)*beta;
      }
    }
  
    // dx = dx + K(dz - Hdx)
    dz *= gamma;
    for(j = BEG; j < n + BEG; ++j)
      _x(j) += d(j)*dz;
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseAImpl() {
    modified_ = true;
    makeBaseA();
    if (modified_)
      flags |= KALMAN_A_MODIFIED;
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseWImpl() {
    modified_ = true;
    makeBaseW();
    if (modified_)
      flags |= KALMAN_W_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseQImpl() {
    modified_ = true;
    makeBaseQ();
    if (modified_)
      flags |= KALMAN_Q_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseHImpl() {
    modified_ = true;
    makeBaseH();
    if (modified_)
      flags |= KALMAN_H_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseVImpl() {
    modified_ = true;
    makeBaseV();
    if (modified_)
      flags |= KALMAN_V_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeBaseRImpl() {
    modified_ = true;
    makeBaseR();
    if (modified_)
      flags |= KALMAN_R_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeAImpl() {
    modified_ = true;
    makeA();
    if (modified_)
      flags |= KALMAN_A_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeWImpl() {
    modified_ = true;
    makeW();
    if (modified_)
      flags |= KALMAN_W_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeQImpl() {
    modified_ = true;
    makeQ();
    if (modified_)
      flags |= KALMAN_Q_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeHImpl() {
    modified_ = true;
    makeH();
    if (modified_)
      flags |= KALMAN_H_MODIFIED;    
  }
  
  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeVImpl() {
    modified_ = true;
    makeV();
    if (modified_)
      flags |= KALMAN_V_MODIFIED;    
  }

  template<typename T, K_UINT_32 BEG, bool OQ, bool OVR, bool DBG>
  void EKFilter<T, BEG, OQ, OVR, DBG>::makeRImpl() {
    modified_ = true;
    makeR();
    if (modified_)
      flags |= KALMAN_R_MODIFIED;    
  }

}

#endif
