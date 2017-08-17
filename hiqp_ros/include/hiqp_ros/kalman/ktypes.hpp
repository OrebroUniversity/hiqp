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

#ifndef KTYPES_HPP
#define KTYPES_HPP

//! \file
//! \brief Contains type definitions specific to each platform.
//!
//! This file can be modified to support different architectures. It also
//! contains some exception classes and helper function definitions.

#include <stdexcept>
#include <string>

// Patch for MSVC++ 6.0 lack of support for templates sstream
#if (defined(_MSC_VER) && _MSC_VER <= 1200)

#define KTYPENAME
#define KSSTREAM_HEADER <strstream>
#define KOSTRINGSTREAM  std::ostrstream
#define KEND_OF_STREAM  << '\0'

#else

#define KTYPENAME       typename
#define KSSTREAM_HEADER <sstream>
#define KOSTRINGSTREAM  std::ostringstream
#define KEND_OF_STREAM

#endif

//! Contains all classes and functions related to Kalman filtering.

//! The Kalman filtering template classes make use of traditional vector and
//! matrix template classes. Since Kalman filtering is mainly used in numeric
//! applications, the presence of another library of vectors and matrices is
//! probable. To avoid name clashes, the \c Kalman namespace contains
//! everything.
namespace Kalman {
  
  typedef short int K_INT_16;             //!< Signed 16-bits integral type
  typedef unsigned short int K_UINT_16;   //!< Unsigned 16-bits integral type
  typedef long int K_INT_32;              //!< Signed 32-bits integral type
  typedef unsigned long int K_UINT_32;    //!< Unsigned 32-bits integral type
  typedef float K_REAL_32;                //!< 32-bits floating point type
  typedef double K_REAL_64;               //!< 64-bits floating point type
  
  //! Base class for all exceptions thrown in the \c Kalman namespace.
  struct KalmanError : public std::logic_error {

    //! Constructor taking an error message.

    //! Since \c KalmanError is derived from <tt>std::logic_error</tt>,
    //! the error message can be displayed by the \c what() function.
    //! \param message Error message.
    explicit KalmanError(const std::string& message) 
      : logic_error(message) {}
  };

  //! Exception class for access to out-of-bound elements.
  struct OutOfBoundError : public KalmanError {

    //! Constructor taking an error message.
    explicit OutOfBoundError(const std::string& message) 
      : KalmanError(message) {}
  };

  //! Nested namespace in \c Kalman to avoid name clash with \c std::swap
  namespace Util {
    
    //! Swaps objects \c a and \c b.
    
    //! \par Type requirements
    //! \c T must be \b assignable.
    //! If \c t1, \c t2 are instances of \c T, the following expressions 
    //! must be valid :
    //! - \code T t1 = t2; \endcode Copy constructor
    //! - \code t1 = t2; \endcode Assignment operator
    template <typename T>
    inline void swap(T& a, T& b) {
      T tmp = a;
      a = b;
      b = tmp;
    }
    
  }

}

#endif
