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

#ifndef KVECTOR_HPP
#define KVECTOR_HPP

//! \file
//! \brief Contains the interface of the \c KVector template class.

#include <vector>
#include <string>
#include <iostream>

#include <hiqp_ros/kalman/ktypes.hpp>


namespace Kalman {

  //! Minimalist vector template class.

  //! This vector class does not define any fancy linear algebra functions, nor
  //! even any operator between vectors. Its sole purpose is to well
  //! encapsulate an extensible array representing a vector, and also to allow
  //! starting index to be 0 or 1.
  //!
  //! \par Template parameters
  //! - \c T : Type of elements contained in the vector. Usually \c float or
  //!          \c double.
  //! - \c BEG : Starting index of vector. Can be either 0 or 1.
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
  class KVector {
  public:

    typedef T type;             //!< Type of objects contained in the vector.

    enum { beg = BEG            //!< Starting index of vector, either 0 or 1.
    };

    //! \name Constructors and destructor.
    //@{

    //! Default constructor. Creates an empty vector.
    inline KVector();

    //! Creates a vector of \c n default instances of \c T.
    inline explicit KVector(K_UINT_32 n);

    //! Creates a vector of \c n copies of \c a.
    inline KVector(K_UINT_32 n, const T& a);

    //! Creates a vector from an array of \c n instances of \c T.
    inline KVector(K_UINT_32 n, const T* v);

    //! Copy constructor. Performs a deep copy.
    inline KVector(const KVector& v);

    //! Destructor.
    inline ~KVector();

    //@}

    //! \name Member access functions.
    //@{

    //! Returns the \c i'th element.
    inline T& operator()(K_UINT_32 i);

    //! Returns the \c i'th element, \c const version.
    inline const T& operator()(K_UINT_32 i) const;

    //! Returns the vector size.
    inline K_UINT_32 size() const;

    //@}

    //! Resizes the vector. Resulting vector contents are undefined.
    inline void resize(K_UINT_32 n);

    //! Assigns a copy of \c a to all elements of the vector.
    inline KVector& operator=(const T& a);

    //! Copy assignment operator. Performs a deep copy.
    inline KVector& operator=(const KVector& v);

    //! Copies an array of \c n instances of \c T.
    inline void assign(K_UINT_32 n, const T* v);

    //! Constant-time swap function between two vectors.
    inline void swap(KVector& v);

    //! \name Streaming functions
    //@{

    //! Reads a vector from a stream.
    inline void get(std::istream& is);

    //! Writes a vector to a stream.
    inline void put(std::ostream& os) const;

    //@}

  private:
    std::vector<T> vimpl_;      //!< Underlying vector implementation.
    
    //! Pointer to start of \a vimpl_ array.

    //! In fact, \a v_ is such that <tt>&v_[beg] == &vimpl_[0]</tt>.
    //! \warning This means that <tt>v_[0]</tt> is not defined for
    //! <tt>beg != 0</tt>.
    T* v_;
    K_UINT_32 n_;               //!< Number of elements in \a vimpl_.
  };

  //! Reads a vector from a stream.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::istream& operator>>(std::istream& is, 
                                  KVector<T, BEG, DBG>& v);

  //! Writes a vector to a stream.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline std::ostream& operator<<(std::ostream& os, 
                                  const KVector<T, BEG, DBG>& v);

  //! Handle type to a vector printing context.
  typedef unsigned short KVectorContext;

  //! Default vector printing context object.
  extern KVectorContext DEFAULT_VECTOR_CONTEXT;

  //! Creates a vector printing context.
  KVectorContext createKVectorContext(std::string elemDelim = " ", 
                                      std::string startDelim = "", 
                                      std::string endDelim = "", 
                                      unsigned prec = 4);

  //! Selects a vector printing context as the current context.
  KVectorContext selectKVectorContext(KVectorContext c);
}

#include <hiqp_ros/kalman/kvector_impl.hpp>
#endif
