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

#ifndef KMATRIX_IMPL_HPP
#define KMATRIX_IMPL_HPP

//! \file
//! \brief Contains the implementation of the \c KMatrix template class.

#include KSSTREAM_HEADER

namespace Kalman {

  //! Contains necessary informations to print a formatted \c KMatrix.

  //! Instances of this class defines the behaviour of input/output operations
  //! of \c KMatrix instances from/to streams. That is to say, 
  //! <tt>get()</tt> and <tt>put()</tt> as well as corresponding
  //! <tt>operator>>()</tt> and <tt>operator<<()</tt>
  //! depend of the \c KMatrixContextImpl that has been selected by calling
  //! <tt>selectKMatrixContext()</tt>.
  class KMatrixContextImpl {
  public:

    //! Constructor.

    //! \param elemDelim Delimiter string between matrix elements.
    //! \param rowDelim Delimiter string at the end of each row.
    //! \param startDelim Starting string before first matrix element.
    //! \param endDelim Ending string after last matrix element.
    //! \param prec Number of significant digits to output
    //! \warning This constructor should never be called directly. Use
    //! <tt>createKMatrixContext()</tt> function instead.
    KMatrixContextImpl(std::string elemDelim = " ",
                       std::string rowDelim = "\n",
                       std::string startDelim = std::string(),
                       std::string endDelim = std::string(),
                       unsigned prec = 4) 
      : elemDelim_(elemDelim), rowDelim_(rowDelim), startDelim_(startDelim), 
        endDelim_(endDelim), precision_(prec), width_(8+prec) {
      
      std::string ws(" \t\n");
      skipElemDelim_  = 
        ( elemDelim_.find_first_not_of(ws) != std::string::npos);
      skipRowDelim_   = 
        (  rowDelim_.find_first_not_of(ws) != std::string::npos);
      skipStartDelim_ = 
        (startDelim_.find_first_not_of(ws) != std::string::npos);
      skipEndDelim_   = 
        (  endDelim_.find_first_not_of(ws) != std::string::npos);
    }

    std::string elemDelim_;  //!< Delimiter string between matrix elements.
    std::string rowDelim_;   //!< Delimiter string at the end of each row.
    std::string startDelim_; //!< Starting string before first matrix element.
    std::string endDelim_;   //!< Ending string after last matrix element.
    unsigned precision_;     //!< Number of significant digits to output.
    unsigned width_;         //!< Width of output field for nice alignment.
    bool skipElemDelim_;     //!< Must we skip a word between elements ?
    bool skipRowDelim_;      //!< Must we skip a word at the end of the row ?
    bool skipStartDelim_;    //!< Must we skip a word at start of vector ?
    bool skipEndDelim_;      //!< Must we skip a word at end of vector ?
  };

  //! Refers to the currently selected matrix printing context.

  //! \warning Never modify this value directly. Use 
  //! <tt>selectKMatrixContext()</tt> instead.
  extern KMatrixContextImpl* currentMatrixContext;

  //! This function is called by all the constructors and also by the 
  //! \c resize() function. If either \c m or \c n is 0, then both
  //! \a m_ and \a n_ will be set to 0.
  //! \pre \a Mimpl_ has been resized to <tt>m*n</tt> elements.
  //! \post \a vimpl_ has been resized to <\c m elements.
  //! \post All pointers of \a vimpl_ have been assigned to point in \a Mimpl_.
  //! \post \a M_, \a m_ and \a n_ have been initialized correctly.
  //! \param m Number of rows. Can be 0.
  //! \param n Number of cols. Can be 0.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KMatrix<T, BEG, DBG>::init(K_UINT_32 m, K_UINT_32 n) {

    if (m != 0 && n != 0) {

      // non-empty matrix
      vimpl_.resize(m);
      T* ptr = &Mimpl_[0] - BEG;
      T** M = &vimpl_[0];
      T** end = M + m;
      
      while (M != end) {
        *M++ = ptr;
        ptr += n;
      }

      M_ = &vimpl_[0] - BEG;
      m_ = m;
      n_ = n;

    } else {

      // empty matrix
      M_ = 0;
      m_ = 0;
      n_ = 0;

    }

  }

  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>::KMatrix() {
    init(0, 0);
  }

  //! \param m Number of rows in matrix. Can be 0.
  //! \param n Number of columns in matrix. Can be 0.
  //! \note If either \c m or \c n is 0, then both the number of rows and the
  //!   number of columns will be set to 0.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>::KMatrix(K_UINT_32 m, K_UINT_32 n)
    : Mimpl_(m*n) { 
    init(m, n);
  }

  //! \param m Number of rows in matrix. Can be 0.
  //! \param n Number of columns in matrix. Can be 0.
  //! \param a Value to copy multiple times in the matrix.
  //! \note If either \c m or \c n is 0, then both the number of rows and the
  //!   number of columns will be set to 0.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>::KMatrix(K_UINT_32 m, K_UINT_32 n, const T& a)
    : Mimpl_(m*n, a) {
    init(m, n);
  }

  //! This function allows to transform a C-style array of \c T objects in
  //! a <tt>KMatrix<T, BEG, DBG></tt> equivalent array. Note that objects from 
  //! the C-style array are copied into the matrix, which may slow down
  //! application if used extensively. The elements are copied row-wise,
  //! that is to say, the elements of the C-style array first fill the first
  //! row of the matrix, then the second, etc.
  //! \param m Number of rows in matrix. Can be 0.
  //! \param n Number of columns in matrix. Can be 0.
  //! \param v Pointer to an <tt>m*n</tt> array of \c T objects.
  //! \note If either \c m or \c n is 0, then both the number of rows and the
  //!   number of columns will be set to 0.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>::KMatrix(K_UINT_32 m, K_UINT_32 n, const T* v)
    : Mimpl_(v, v + m*n) {
    init(m, n);
  }

  //! \param M Matrix to copy. Can be an empty matrix.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>::KMatrix(const KMatrix& M) 
    : Mimpl_(M.Mimpl_) {
    init(M.m_, M.n_);
  }

  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>::~KMatrix() {}

  //! \param i Row index of matrix element.
  //! \param j Column index of matrix element.
  //! \return A reference to the element <tt>(i,j)</tt>. 
  //! \exception OutOfBoundError Thrown if \c i or \c j is out of matrix 
  //!   bounds and <tt>DBG == true</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline T& KMatrix<T, BEG, DBG>::operator()(K_UINT_32 i, 
                                             K_UINT_32 j) {
    if (DBG) {
      if (i < BEG || i >= m_ + BEG || j < BEG || j >= n_ + BEG) {
        KOSTRINGSTREAM oss;
        oss << "Trying to access element (" << i << ", " << j 
            << ") not included in [" << BEG << ", " << m_ + BEG - 1 << "][" 
            << BEG << ", " << n_ + BEG - 1 << "]." KEND_OF_STREAM;
        throw OutOfBoundError(oss.str());
      }
    }
    return M_[i][j];
  }

  //! \param i Row index of matrix element.
  //! \param j Column index of matrix element.
  //! \return A \c const reference to the element <tt>(i,j)</tt>. 
  //! \exception OutOfBoundError Thrown if \c i or \c j is out of matrix 
  //!   bounds and <tt>DBG == true</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline const T& KMatrix<T, BEG, DBG>::operator()(K_UINT_32 i, 
                                                   K_UINT_32 j) const {
    if (DBG) {
      if (i < BEG || i >= m_ + BEG || j < BEG || j >= n_ + BEG) {
        KOSTRINGSTREAM oss;
        oss << "Trying to access element (" << i << ", " << j 
            << ") not included in [" << BEG << ", " << m_ + BEG - 1 << "][" 
            << BEG << ", " << n_ + BEG - 1 << "]." KEND_OF_STREAM;
        throw OutOfBoundError(oss.str());
      }
    }
    return M_[i][j];
  }

  //! \return The number of rows in the matrix.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline K_UINT_32 KMatrix<T, BEG, DBG>::nrow() const {
    return m_;
  }

  //! \return The number of columns in the matrix.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline K_UINT_32 KMatrix<T, BEG, DBG>::ncol() const {
    return n_;
  }

  //! \param m Number of rows in matrix. Can be 0.
  //! \param n Number of columns in matrix. Can be 0.
  //! \warning This function may invalidates pointers inside the matrix.
  //! \note Resizing to a smaller size does not free any memory. To do so,
  //! one can swap the matrix to shrink with a temporary copy of itself :
  //! <tt>KMatrix<T, BEG, DBG>(M).swap(M)</tt>.
  //! \note If either \c m or \c n is 0, then both the number of rows and the
  //!   number of columns will be set to 0.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KMatrix<T, BEG, DBG>::resize(K_UINT_32 m, K_UINT_32 n) {

    if (m == m_ && n == n_) {
      return;
    }

    Mimpl_.resize(m*n);
    init(m, n);
  }

  //! \param a Instance to copy to each element of matrix.
  //! \return A reference to the matrix.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>& KMatrix<T, BEG, DBG>::operator=(const T& a) {
    
    T* ptr = &Mimpl_[0];
    const T* end = ptr + Mimpl_.size();

    while (ptr != end) {
      *ptr++ = a;
    }
    return *this;
  }

  //! \param M Matrix to copy.
  //! \return A reference to the assigned matrix.
  //! \warning This function invalidates pointers inside the matrix.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KMatrix<T, BEG, DBG>& 
  KMatrix<T, BEG, DBG>::operator=(const KMatrix& M) {
    KMatrix temp(M);
    swap(temp);    
    return *this;
  }

  //! The elements are copied row-wise,
  //! that is to say, the elements of the C-style array first fill the first
  //! row of the matrix, then the second, etc.
  //! \param m Number of rows in matrix. Can be 0.
  //! \param n Number of columns in matrix. Can be 0.
  //! \param v Pointer to first element to copy from C-style array.
  //! \warning This function invalidates pointers inside the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KMatrix<T, BEG, DBG>::assign(K_UINT_32 m, K_UINT_32 n, 
                                           const T* v) {
    KMatrix temp(m, n, v);
    swap(temp);
  }

  //! This function is fast, since it exchanges pointers to underlying
  //! implementation without copying any element.
  //! \param M Matrix to swap.
  //! \note No pointer is invalidated by this function. They still point to
  //!   the same element, but in the other matrix.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KMatrix<T, BEG, DBG>::swap(KMatrix& M) {
    vimpl_.swap(M.vimpl_);
    Mimpl_.swap(M.Mimpl_);
    Util::swap(M_, M.M_);
    Util::swap(m_, M.m_);
    Util::swap(n_, M.n_);
  }

  //! This function will extract <tt>nrow()*ncol()</tt> elements from a 
  //! stream to fill the matrix, while considering the formatting constraints 
  //! of the current matrix printing context.
  //! \param is A reference to the input stream.
  //! \see <tt>createKMatrixContext()</tt>
  //! \see <tt>selectKMatrixContext()</tt>
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KMatrix<T, BEG, DBG>::get(std::istream& is) {

    T* ptr = &Mimpl_[0];
    std::string tmp;
    K_UINT_32 i, j;

    if (currentMatrixContext->skipStartDelim_) {
      is >> tmp;
    }

    if (currentMatrixContext->skipRowDelim_) {

      if (currentMatrixContext->skipElemDelim_) {

        for (i = 0; i < m_-1; ++i) {
          for (j = 0; j < n_; ++j) {
            is >> *ptr++ >> tmp;
          }
        }

        for (j = 0; j < n_-1; ++j) {
          is >> *ptr++ >> tmp;
        }

        is >> *ptr;

      } else {

        for (i = 0; i < m_-1; ++i) {
          for (j = 0; j < n_; ++j) {
            is >> *ptr++;
          }
          is >> tmp;
        }

        for (j = 0; j < n_; ++j) {
          is >> *ptr++;
        }

      }

    } else {

      if (currentMatrixContext->skipElemDelim_) {

        for (i = 0; i < m_; ++i) {
          for (j = 0; j < n_-1; ++j) {
            is >> *ptr++ >> tmp;
          }
          is >> *ptr++;
        }

      } else {

        for (i = 0; i < m_; ++i) {
          for (j = 0; j < n_; ++j) {
            is >> *ptr++;
          }
        }

      }

    }

    if (currentMatrixContext->skipEndDelim_) {
      is >> tmp;
    }
  }

  //! This function will send all the matrix elements to a stream, while 
  //! considering the formatting constraints of the current matrix printing 
  //! context.
  //! \param os A reference to the output stream.
  //! \see <tt>createKMatrixContext()</tt>
  //! \see <tt>selectKMatrixContext()</tt>
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KMatrix<T, BEG, DBG>::put(std::ostream& os) const {
    if (m_ == 0 || n_ == 0) {
      return;
    }

    const T* ptr = &Mimpl_[0];
    K_UINT_32 i, j;

    std::ios::fmtflags f = os.setf(std::ios::scientific, std::ios::floatfield);
    os.setf(std::ios::showpoint);
    std::streamsize p = os.precision(currentMatrixContext->precision_);

    os << currentMatrixContext->startDelim_;

    for (i = 0; i < m_-1; ++i) {
      for (j = 0; j < n_-1; ++j) {
        os.width(currentMatrixContext->width_);
        os << *ptr++ << currentMatrixContext->elemDelim_;
      }
      os.width(currentMatrixContext->width_);
      os << *ptr++ << currentMatrixContext->rowDelim_;
    }

    for (j = 0; j < n_-1; ++j) {
      os.width(currentMatrixContext->width_);
      os << *ptr++ << currentMatrixContext->elemDelim_;
    }

    os.width(currentMatrixContext->width_);
    os << *ptr++ << currentMatrixContext->endDelim_;

    os.precision(p);
    os.flags(f);
  }

  //! \param is A reference to the input stream.
  //! \param M  A reference to the matrix to read.
  //! \return A reference to the input stream.
  //! \note This function is equivalent to <tt>M.get(is)</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::istream& operator>>(std::istream& is, 
                                  KMatrix<T, BEG, DBG>& M) {
    M.get(is);
    return is;
  }

  //! \param os A reference to the output stream.
  //! \param M  A reference to the matrix to print.
  //! \return A reference to the output stream.
  //! \note This function is equivalent to <tt>M.put(os)</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::ostream& operator<<(std::ostream& os, 
                                  const KMatrix<T, BEG, DBG>& M) {
    M.put(os);
    return os;
  }

}

#endif
