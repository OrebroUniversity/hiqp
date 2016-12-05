// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HIQP_TIME_POINT_H
#define HIQP_TIME_POINT_H

namespace hiqp {

  /*! \brief Represents a time duration in seconds and nanoseconds.
   *  \author Marcus A Johansson */  
  class HiQPTimePoint
  {
  public:
    HiQPTimePoint();
    HiQPTimePoint(unsigned int sec, unsigned int nsec);
    HiQPTimePoint(const HiQPTimePoint& other);
    ~HiQPTimePoint();

    double toSec() const;

    inline void setTimePoint(unsigned int sec, unsigned int nsec)
    { this->sec_ = sec; this->nsec_ = nsec; }

    inline unsigned int getSec() const { return sec_; }
    inline unsigned int getNSec() const { return nsec_; }

    HiQPTimePoint& operator=(const HiQPTimePoint& other);
    HiQPTimePoint operator+(const HiQPTimePoint& other) const;
    HiQPTimePoint operator-(const HiQPTimePoint& other) const;
    HiQPTimePoint& operator+=(const HiQPTimePoint& other);
    HiQPTimePoint& operator-=(const HiQPTimePoint& other);

  private:
    unsigned int sec_;
    unsigned int nsec_;
  };

} // namespace hiqp

#endif // include guard