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

#include <hiqp/hiqp_time_point.h>

namespace hiqp {

  HiQPTimePoint::HiQPTimePoint() : sec_(0), nsec_(0) {}

  HiQPTimePoint::HiQPTimePoint(unsigned int sec, unsigned int nsec) 
   : sec_(sec), nsec_(nsec) {}

  HiQPTimePoint::HiQPTimePoint(const HiQPTimePoint& other)
   : sec_(other.getSec()), nsec_(other.getNSec()) {}

  HiQPTimePoint::~HiQPTimePoint() {}

  double HiQPTimePoint::toSec() const {
      return static_cast<double>(sec_) + static_cast<double>(nsec_) * 1e-9;
  }

  HiQPTimePoint& HiQPTimePoint::operator=(const HiQPTimePoint& other) {
    this->sec_ = other.getSec();
    this->nsec_ = other.getNSec();
    return *this;
  }

  HiQPTimePoint HiQPTimePoint::operator+(const HiQPTimePoint& other) const {
    unsigned int sec = sec_ + other.getSec();
    unsigned int nsec = nsec_ + other.getNSec();
    unsigned int overlap = nsec * 1e-9;
    return HiQPTimePoint( sec + overlap, nsec - overlap * 1e9 );
  }

  HiQPTimePoint HiQPTimePoint::operator-(const HiQPTimePoint& other) const {
    unsigned int sec = sec_ - other.getSec();
    unsigned int nsec = 0;
    unsigned int underlap = 0;

    if (nsec_ >= other.getNSec()) {
      nsec = nsec_ - other.getNSec();
    } else {
      nsec = 1e9 + nsec_ - other.getNSec();
      sec -= 1;
    }

    return HiQPTimePoint(sec, nsec);
  }

  HiQPTimePoint& HiQPTimePoint::operator+=(const HiQPTimePoint& other) {
    unsigned int sec = sec_ + other.getSec();
    unsigned int nsec = nsec_ + other.getNSec();
    unsigned int overlap = nsec * 1e-9;

    this->sec_ = sec + overlap;
    this->nsec_ = nsec - overlap * 1e9;

    return *this;
  }

  HiQPTimePoint& HiQPTimePoint::operator-=(const HiQPTimePoint& other) {
    unsigned int sec = sec_ - other.getSec();
    unsigned int nsec = 0;
    unsigned int underlap = 0;

    if (nsec_ >= other.getNSec()) {
      nsec = nsec_ - other.getNSec();
    } else {
      nsec = 1e9 + nsec_ - other.getNSec();
      sec -= 1;
    }

    this->sec_ = sec;
    this->nsec_ = nsec;

    return *this;
  }

} // namespace hiqp