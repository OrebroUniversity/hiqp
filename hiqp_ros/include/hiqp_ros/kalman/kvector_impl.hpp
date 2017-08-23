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

#ifndef KVECTOR_IMPL_HPP
#define KVECTOR_IMPL_HPP

//! \file
//! \brief Contains the implementation of the \c KVector template class.

#include KSSTREAM_HEADER

namespace Kalman {

  //! Contains necessary informations to print a formatted \c KVector.

  //! Instances of this class defines the behaviour of input/output operations
  //! of \c KVector instances from/to streams. That is to say, 
  //! <tt>get()</tt> and <tt>put()</tt> as well as corresponding
  //! <tt>operator>>()</tt> and <tt>operator<<()</tt>
  //! depend of the \c KVectorContextImpl that has been selected by calling
  //! <tt>selectKVectorContext()</tt>.
  class KVectorContextImpl {
  public:

    //! Constructor.

    //! \param elemDelim Delimiter string between vector elements.
    //! \param startDelim Starting string before first vector element.
    //! \param endDelim Ending string after last vector element.
    //! \param prec Number of significant digits to output.
    //! \warning This constructor should never be called directly. Use
    //! <tt>createKVectorContext()</tt> function instead.
    explicit KVectorContextImpl(std::string elemDelim = " ",
                                std::string startDelim = std::string(),
                                std::string endDelim = std::string(),
                                unsigned prec = 4) 
      : elemDelim_(elemDelim), startDelim_(startDelim), 
        endDelim_(endDelim), precision_(prec), width_(8+prec) {
      
      std::string ws(" \t\n");
      skipElemDelim_  = 
        ( elemDelim_.find_first_not_of(ws) != std::string::npos );
      skipStartDelim_ = 
        (startDelim_.find_first_not_of(ws) != std::string::npos );
      skipEndDelim_   = 
        (  endDelim_.find_first_not_of(ws) != std::string::npos );
    }

    std::string elemDelim_;  //!< Delimiter string between vector elements.
    std::string startDelim_; //!< Starting string before first vector element.
    std::string endDelim_;   //!< Ending string after last vector element.
    unsigned precision_;     //!< Number of significant digits to output.
    unsigned width_;         //!< Width of output field for nice alignment.
    bool skipElemDelim_;     //!< Must we skip a word between elements ?
    bool skipStartDelim_;    //!< Must we skip a word at start of vector ?
    bool skipEndDelim_;      //!< Must we skip a word at end of vector ?
};

  //! Refers to the currently selected vector printing context.

  //! \warning Never modify this value directly. Use 
  //! <tt>selectKVectorContext()</tt> instead.
  extern KVectorContextImpl* currentVectorContext;

  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector()
    : v_(0), n_(0) {}

  //! \param n Number of elements in vector. Can be 0.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(K_UINT_32 n)
    : vimpl_(n), v_( (n != 0) ? &vimpl_[0] - BEG : 0 ), n_(n) {}

  //! \param n Number of elements in vector. Can be 0.
  //! \param a Value to copy multiple times in the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(K_UINT_32 n, const T& a)
    : vimpl_(n, a), v_( (n != 0) ? &vimpl_[0] - BEG : 0 ), n_(n) {}

  //! This function allows to transform a C-style array of \c T objects in
  //! a <tt>KVector<T, BEG, DBG></tt> equivalent array. Note that objects from 
  //! the C-style array are copied into the vector, which may slow down
  //! application if used extensively.
  //! \param n Size of the \c v array. Can be 0.
  //! \param v Pointer to an \c n-size array of \c T objects.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(K_UINT_32 n, const T* v)
    : vimpl_(v, v + n), v_( (n != 0) ? &vimpl_[0] - BEG : 0 ), n_(n) {}

  //! \param v Vector to copy. Can be an empty vector.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(const KVector& v) 
    : vimpl_(v.vimpl_), v_( (v.size() != 0) ? &vimpl_[0] - BEG : 0 ), 
      n_(v.n_) {}

  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::~KVector() {}

  //! \param i Index of element to retrieve from vector.
  //! \return A reference to the \c i'th element. 
  //! \exception OutOfBoundError Thrown if \c i is out of vector bounds and 
  //!   <tt>DBG == true</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline T& KVector<T, BEG, DBG>::operator()(K_UINT_32 i) {
    if (DBG) {
      if (i < BEG || i >= n_ + BEG) {
        KOSTRINGSTREAM oss;
        oss << "Trying to access element " << i << " not included in ["
            << BEG << ", " << n_ + BEG - 1 << "]." KEND_OF_STREAM;
        throw OutOfBoundError(oss.str());
      }
    }
    return v_[i];
  }

  //! \param i Index of element to retrieve from vector.
  //! \return A \c const reference to the \c i'th element. 
  //! \exception OutOfBoundError Thrown if i is out of vector bounds and 
  //!   <tt>DBG == true</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline const T& KVector<T, BEG, DBG>::operator()(K_UINT_32 i) const {
    if (DBG) {
      if (i < BEG || i >= n_ + BEG) {
        KOSTRINGSTREAM oss;
        oss << "Trying to access element " << i << " not included in ["
            << BEG << ", " << n_ + BEG - 1 << "]." KEND_OF_STREAM;
        throw OutOfBoundError(oss.str());
      }
    }
    return v_[i];
  }

  //! \return The number of elements in the vector.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline K_UINT_32 KVector<T, BEG, DBG>::size() const {
    return n_;
  }

  //! \param n The new size of the vector. Can be 0.
  //! \warning This function may invalidates pointers inside the vector.
  //! \note Resizing to a smaller size does not free any memory. To do so,
  //! one can swap the vector to shrink with a temporary copy of itself :
  //! <tt>KVector<T, BEG, DBG>(v).swap(v)</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::resize(K_UINT_32 n) {
    
    if (n == n_) {
      return;
    }
    
    vimpl_.resize(n);
    v_ = (n != 0) ? &vimpl_[0] - BEG : 0;
    n_ = n;
  }

  //! \param a Instance to copy to each element of vector.
  //! \return A reference to the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>& KVector<T, BEG, DBG>::operator=(const T& a) {
    if (n_ == 0) {
      return *this;
    }

    T* ptr = &vimpl_[0];
    const T* end = ptr + n_;

    while (ptr != end) {
      *ptr++ = a;
    }
    return *this;
  }

  //! \param v Vector to copy.
  //! \return A reference to the assigned vector.
  //! \warning This function invalidates pointers inside the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>& 
  KVector<T, BEG, DBG>::operator=(const KVector& v) {
    KVector temp(v);
    swap(temp);
    return *this;
  }

  //! \param n Size of C-style array.
  //! \param v Pointer to first element to copy from C-style array.
  //! \warning This function invalidates pointers inside the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::assign(K_UINT_32 n, const T* v) {
    KVector temp(n, v);
    swap(temp);
  }

  //! This function is fast, since it exchanges pointers to underlying
  //! implementation without copying any element.
  //! \param v Vector to swap.
  //! \note No pointer is invalidated by this function. They still point to
  //!   the same element, but in the other vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::swap(KVector& v) {
    vimpl_.swap(v.vimpl_);
    Util::swap(v_, v.v_);
    Util::swap(n_, v.n_);
  }

  //! This function will extract <tt>size()</tt> elements from a stream to 
  //! fill the vector, while considering the formatting constraints of the 
  //! current vector printing context.
  //! \param is A reference to the input stream.
  //! \see <tt>createKVectorContext()</tt>
  //! \see <tt>selectKVectorContext()</tt>
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::get(std::istream& is) {
    if (n_ == 0) {
      return;
    }
    
    T* ptr = &vimpl_[0];
    std::string tmp;
    K_UINT_32 i;
    
    if (currentVectorContext->skipStartDelim_) {
      is >> tmp;
    }

    if (currentVectorContext->skipElemDelim_) {
      for (i = 0; i < n_-1; ++i) {
        is >> ptr[i] >> tmp;
      }
      is >> ptr[i];
    } else {
      for (i = 0; i < n_; ++i) {
        is >> ptr[i];
      }
    }

    if (currentVectorContext->skipEndDelim_) {
      is >> tmp;
    }
  }

  //! This function will send all the vector elements to a stream, while 
  //! considering the formatting constraints of the current vector printing 
  //! context.
  //! \param os A reference to the output stream.
  //! \see <tt>createKVectorContext()</tt>
  //! \see <tt>selectKVectorContext()</tt>
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::put(std::ostream& os) const {
    if (n_ == 0) {
      return;
    }

    const T* ptr = &vimpl_[0];
    K_UINT_32 i;

    std::ios::fmtflags f = os.setf(std::ios::scientific, 
                                   std::ios::floatfield);
    os.setf(std::ios::showpoint);
    std::streamsize p = os.precision(currentVectorContext->precision_);

    os << currentVectorContext->startDelim_;
    for (i = 0; i < n_ - 1; ++i) {
      os.width(currentVectorContext->width_);
      os << ptr[i] << currentVectorContext->elemDelim_;
    }
    os.width(currentVectorContext->width_);
    os << ptr[i] << currentVectorContext->endDelim_;

    os.precision(p);
    os.flags(f);
  }

  //! \param is A reference to the input stream.
  //! \param v  A reference to the vector to read.
  //! \return A reference to the input stream.
  //! \note This function is equivalent to <tt>v.get(is)</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::istream& operator>>(std::istream& is, 
                                  KVector<T, BEG, DBG>& v) {
    v.get(is);
    return is;
  }

  //! \param os A reference to the output stream.
  //! \param v  A reference to the vector to print.
  //! \return A reference to the output stream.
  //! \note This function is equivalent to <tt>v.put(os)</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::ostream& operator<<(std::ostream& os, 
                                  const KVector<T, BEG, DBG>& v) {
    v.put(os);
    return os;
  }

}

#endif
