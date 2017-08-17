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

#ifndef KMATRIX_HPP
#define KMATRIX_HPP

//! \file
//! \brief Contains the interface of the \c KMatrix template class.

#include <vector>
#include <string>
#include <iostream>

#include <hiqp_ros/kalman/ktypes.hpp>

namespace Kalman {

  //! Minimalist matrix template class.

  //! This matrix class does not define any fancy linear algebra functions, nor
  //! even any operator between matrices. Its sole purpose is to well
  //! encapsulate an extensible array representing a matrix, and also to allow
  //! starting index to be 0 or 1.
  //!
  //! \par Template parameters
  //! - \c T : Type of elements contained in the matrix. Usually \c float or
  //!          \c double.
  //! - \c BEG : Starting index of matrix. Can be either 0 or 1.
  //! - \c DGB : Debug flag. If \c true, then bound-checking will be performed,
  //!            and \c OutOfBoundError exceptions can be thrown.
  //!
  //! \par Type requirements for T
  //! - \c T must be <b>default constructible</b>.
  //! - \c T must be \b assignable.
  //! - \c T must be \b serializable.
  //! .
  //! This means that, if \c t1, \c t2 are instances of \c T, \c is is of type
  //! \c istream and \c os is of type \c ostream, the following
  //! expressions must be valid :
  //! - \code T(); T t1; \endcode Default constructor
  //! - \code T t1 = t2; T t1(t2); T(t1); \endcode Copy constructor
  //! - \code t1 = t2; \endcode Assignment operator
  //! - \code is >> t1; \endcode \c operator>>()
  //! - \code os << t1; \endcode \c operator<<()
  //! .
  //! Finally, note that \c operator>>() and \c operator<<() must be
  //! compatible. Also, \c operator&() must not have been overloaded.
  template<typename T, K_UINT_32 BEG, bool DBG>
  class KMatrix {
  public:

    typedef T type;          //!< Type of objects contained in the matrix.

    enum { beg = BEG         //!< Starting index of matrix, either 0 or 1.
    };

    //! \name Constructors and destructor.
    //@{

    //! Default constructor. Creates an empty matrix.
    inline KMatrix();

    //! Creates an \c m by \c n matrix of default instances of \c T.
    inline KMatrix(K_UINT_32 m, K_UINT_32 n);

    //! Creates an \c m by \c n matrix of copies of \c a.
    inline KMatrix(K_UINT_32 m, K_UINT_32 n, const T& a);

    //! Creates an \c m by \c n matrix from an array of instances of \c T.
    inline KMatrix(K_UINT_32 m, K_UINT_32 n, const T* v);

    //! Copy constructor. Performs a deep copy.
    inline KMatrix(const KMatrix& M);

    //! Destructor.
    inline ~KMatrix();

    //@}

    //! \name Member access functions.
    //@{

    //! Returns the element <tt>(i,j)</tt>.
    inline T& operator()(K_UINT_32 i, K_UINT_32 j);

    //! Returns the element <tt>(i,j)</tt>, \c const version.
    inline const T& operator()(K_UINT_32 i, K_UINT_32 j) const;

    //! Returns \a m_, the number of rows of the matrix.
    inline K_UINT_32 nrow() const;

    //! Returns \a n_, the number of columns of the matrix.
    inline K_UINT_32 ncol() const;

    //@}

    //! Resizes the matrix. Resulting matrix contents are undefined.
    inline void resize(K_UINT_32 m, K_UINT_32 n);

    //! Assigns a copy of \c a to all elements of the matrix.
    inline KMatrix& operator=(const T& a);

    //! Copy assignment operator. Performs a deep copy.
    inline KMatrix& operator=(const KMatrix& M);

    //! Copies a C-style array of instances of \c T in an \c m by \c n matrix.
    inline void assign(K_UINT_32 m, K_UINT_32 n, const T* v);

    //! Constant-time swap function between two matrices.
    inline void swap(KMatrix& M);

    //! \name Streaming functions
    //@{

    //! Reads a matrix from a stream.
    inline void get(std::istream& is);

    //! Writes a matrix to a stream.
    inline void put(std::ostream& os) const;

    //@}

  private:
    //! Array of pointers to rows of \a Mimpl_.

    //! In fact, \a vimpl_ is such that 
    //! <tt>&vimpl_[i][beg] == &Mimpl_[i*n_]</tt>.
    std::vector<T*> vimpl_;
    std::vector<T> Mimpl_;    //!< Underlying vector implementation.
    
    //! Pointer to the start of \a vimpl_.

    //! In fact, \a M_ is such that <tt>&M_[beg] == &vimpl_[0]</tt>.
    //! This means also that <tt>&M_[beg][beg] == &Mimpl_[0][0]</tt>.
    //! \warning <tt>M_[0]</tt> is not defined for
    //! <tt>beg != 0</tt>.
    T** M_;
    K_UINT_32 m_;             //!< Number of rows of matrix.
    K_UINT_32 n_;             //!< Number of columns of matrix.

    //! Helper function to initialize matrix.
    inline void init(K_UINT_32 m, K_UINT_32 n);
  };

  //! Reads a matrix from a stream.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::istream& operator>>(std::istream& is, 
                                  KMatrix<T, BEG, DBG>& M);

  //! Writes a matrix to a stream.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::ostream& operator<<(std::ostream& os, 
                                  const KMatrix<T, BEG, DBG>& M);

  //! Handle type to a matrix printing context.
  typedef unsigned short KMatrixContext;

  //! Default matrix printing context object.
  extern KMatrixContext DEFAULT_MATRIX_CONTEXT;

  //! Creates a matrix printing context.
  KMatrixContext createKMatrixContext(std::string elemDelim = " ", 
                                      std::string rowDelim = "\n",
                                      std::string startDelim = "", 
                                      std::string endDelim = "", 
                                      unsigned prec = 4);

  //! Selects a matrix printing context as the current context.
  KMatrixContext selectKMatrixContext(KMatrixContext c);

}

#include <hiqp_ros/kalman/kmatrix_impl.hpp>
#endif
