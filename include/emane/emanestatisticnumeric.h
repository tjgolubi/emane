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

#ifndef EMANESTATISTICNUMERIC_T_HEADER_
#define EMANESTATISTICNUMERIC_T_HEADER_

#include "emane/emanestatistic.h"

#include <ace/Thread_Mutex.h>

namespace EMANE
{
/**
 *
 * @class  StatisticNumeric 
 *
 * @brief Defines a numeric statistic and its operations
 *
 */
  template<typename T>
  class StatisticNumeric : public Statistic
  {
  public:
    StatisticNumeric(const T & value = 0);
    
    StatisticNumeric(const StatisticNumeric<T> & stat);
    
    virtual ~StatisticNumeric<T>(){}
   
    StatisticNumeric<T> & operator++();

    const StatisticNumeric<T> operator++(int);

    StatisticNumeric<T> & operator+=(const StatisticNumeric<T> & rhs);

    StatisticNumeric<T> operator+(const StatisticNumeric<T> & rhs) const;

    StatisticNumeric<T> & operator--();

    const StatisticNumeric<T> operator--(int);

    StatisticNumeric<T> operator-(const StatisticNumeric<T> & rhs) const;

    StatisticNumeric<T> & operator-=(const StatisticNumeric<T> & rhs);

    StatisticNumeric<T> operator*(const StatisticNumeric<T> & rhs) const;

    StatisticNumeric<T> & operator*=(const StatisticNumeric<T> & rhs);

    StatisticNumeric<T> operator/(const StatisticNumeric<T> & rhs) const;

    StatisticNumeric<T> & operator/=(const StatisticNumeric<T> & rhs);

    StatisticNumeric<T> & operator=(const StatisticNumeric<T> & rhs);

    bool operator==(const StatisticNumeric<T> &rhs) const;

    bool operator!=(const StatisticNumeric<T> &rhs) const;

    bool operator<(const StatisticNumeric<T> &rhs) const;

    bool operator<=(const StatisticNumeric<T> &rhs) const;

    bool operator>(const StatisticNumeric<T> &rhs) const;

    bool operator>=(const StatisticNumeric<T> &rhs) const;

    T getValue() const;

    std::string toString() const;
    
    void clear();

  private:
    T value_;    
    mutable ACE_Thread_Mutex mutex_;
  };
} 

template <typename T>
bool operator<=(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator>=(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator>(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator<(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator==(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator!=(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T operator+(const T & val, const EMANE::StatisticNumeric<T> & stat);
    
template <typename T>
T operator-(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T operator*(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T operator/(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator+=(T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator-=(T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator*=(T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator/=(T & val, const EMANE::StatisticNumeric<T> & stat);
      
#include "emanestatisticnumeric.inl"

#endif // EMANESTATISTICNUMERIC_T_HEADER_
