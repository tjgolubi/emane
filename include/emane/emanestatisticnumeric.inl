/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of DRS CenGen, LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The copyright holder hereby grants to the U.S. Government a copyright
 * license to use this computer software/computer software documentation
 * that is of the same scope as the rights set forth in the definition of
 * "unlimited rights" found in DFARS 252.227-7014(a)(15)(June 1995).
 */

#include <sstream>
#include <iomanip>
#include <ace/Guard_T.h>

template<typename T>
EMANE::StatisticNumeric<T>::StatisticNumeric(const T & value):
  value_(value){}

template<typename T>
EMANE::StatisticNumeric<T>::StatisticNumeric(const EMANE::StatisticNumeric<T> & stat):
  value_(stat.value_){}

template<typename T>
EMANE::StatisticNumeric<T> & EMANE::StatisticNumeric<T>::operator++()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  
  ++value_;
  
  return *this;
}

template<typename T>
const EMANE::StatisticNumeric<T> EMANE::StatisticNumeric<T>::operator++(int)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  
  StatisticNumeric<T> tmp(*this);
      
  ++value_;

  return tmp;
}

template<typename T>
EMANE::StatisticNumeric<T> & EMANE::StatisticNumeric<T>::operator+=(const EMANE::StatisticNumeric<T> & rhs)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  
  value_ += rhs.value_;
  
  return *this;
}

template<typename T>
EMANE::StatisticNumeric<T> EMANE::StatisticNumeric<T>::operator+(const EMANE::StatisticNumeric<T> & rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  EMANE::StatisticNumeric<T> tmp(*this);
      
  tmp += rhs;

  return tmp;
}

template<typename T>
EMANE::StatisticNumeric<T> & EMANE::StatisticNumeric<T>::operator--()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  --value_;

  return *this;
}

template<typename T>
const EMANE::StatisticNumeric<T> EMANE::StatisticNumeric<T>::operator--(int)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  EMANE::StatisticNumeric<T> tmp(this->value_);

  --value_;

  return tmp;
}

template<typename T>
EMANE::StatisticNumeric<T> EMANE::StatisticNumeric<T>::operator-(const EMANE::StatisticNumeric<T> & rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  EMANE::StatisticNumeric<T> tmp(*this);
      
  tmp -= rhs;

  return tmp;
}

template<typename T>
EMANE::StatisticNumeric<T> & EMANE::StatisticNumeric<T>::operator-=(const EMANE::StatisticNumeric<T> & rhs)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  value_ -= rhs.value_;

  return *this;
}

template<typename T>
EMANE::StatisticNumeric<T> EMANE::StatisticNumeric<T>::operator*(const EMANE::StatisticNumeric<T> & rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  EMANE::StatisticNumeric<T> tmp(*this);

  tmp *= rhs;

  return tmp;
}

template<typename T>
EMANE::StatisticNumeric<T> & EMANE::StatisticNumeric<T>::operator*=(const EMANE::StatisticNumeric<T> & rhs)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
      
  value_ *= rhs.value_;
      
  return *this;
}

template<typename T>
EMANE::StatisticNumeric<T> EMANE::StatisticNumeric<T>::operator/(const EMANE::StatisticNumeric<T> & rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  EMANE::StatisticNumeric<T> tmp(*this);
      
  tmp /= rhs;
      
  return tmp;
}

template<typename T>
EMANE::StatisticNumeric<T> & EMANE::StatisticNumeric<T>::operator/=(const EMANE::StatisticNumeric<T> & rhs)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
      
  value_ /= rhs.value_;
      
  return *this;
}

template<typename T>
EMANE::StatisticNumeric<T> & EMANE::StatisticNumeric<T>::operator=(const EMANE::StatisticNumeric<T> & rhs)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
            
  value_ = rhs.value_;

  return *this;
}

template<typename T>
bool EMANE::StatisticNumeric<T>::operator==(const EMANE::StatisticNumeric<T> &rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  return value_ == rhs.value_;
}

template<typename T>
bool EMANE::StatisticNumeric<T>::operator!=(const EMANE::StatisticNumeric<T> &rhs) const
{
  return !operator==(rhs);
}

template<typename T>
bool EMANE::StatisticNumeric<T>::operator<(const EMANE::StatisticNumeric<T> &rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
      
  return value_ < rhs.value_;
}

template<typename T>
bool EMANE::StatisticNumeric<T>::operator<=(const EMANE::StatisticNumeric<T> &rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
      
  return value_ <= rhs.value_;
}

template<typename T>
bool EMANE::StatisticNumeric<T>::operator>(const EMANE::StatisticNumeric<T> &rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
      
  return value_ > rhs.value_;
}

template<typename T>
bool EMANE::StatisticNumeric<T>::operator>=(const EMANE::StatisticNumeric<T> &rhs) const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
      
  return value_ >= rhs.value_;
}

template<typename T>
T EMANE::StatisticNumeric<T>::getValue() const
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  return value_;
}


template<typename T>
std::string EMANE::StatisticNumeric<T>::toString() const
{
  std::stringstream ss;
  ss << getValue();
  return ss.str();
}

// use partial template specialization to specialize the 
// floating point string representation
namespace EMANE
{
  template <>
  inline std::string StatisticNumeric<float>::toString() const
  {
    std::stringstream ss;
    ss<<std::fixed<<std::setprecision(6)<<value_;
    return ss.str();
  }
}

template<typename T>
void EMANE::StatisticNumeric<T>::clear()
{
  value_ = 0;
}

template <typename T>
bool operator<=(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return EMANE::StatisticNumeric<T>(val)<=stat;
}

template <typename T>
bool operator>=(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return EMANE::StatisticNumeric<T>(val)>=stat;
}

template <typename T>
bool operator>(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return EMANE::StatisticNumeric<T>(val)>stat;
}

template <typename T>
bool operator<(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val > stat.getValue();
}

template <typename T>
bool operator==(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val == stat.getValue();
}

template <typename T>
bool operator!=(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val != stat.getValue();
}

template <typename T>
T operator+(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val + stat.getValue();
}
    
template <typename T>
T operator-(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val - stat.getValue();
}

template <typename T>
T operator*(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val * stat.getValue();
}

template <typename T>
T operator/(const T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val / stat.getValue();
}

template <typename T>
T & operator+=(T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val += stat.getValue();
}

template <typename T>
T & operator-=(T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val -= stat.getValue();
}

template <typename T>
T & operator*=(T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val *= stat.getValue();
}

template <typename T>
T & operator/=(T & val, const EMANE::StatisticNumeric<T> & stat)
{
  return val /= stat.getValue();
}

