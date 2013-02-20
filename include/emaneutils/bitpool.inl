/*
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "bitpool.h"

#include <cmath>

#include <ace/OS_NS_sys_time.h>


const ACE_UINT64 USEC_PER_SEC = 1000000;


/**
  *
  * @class BitPool
  *
  * @brief Implementation of a rate limiting bit pool
  *
  * @param pPlatformService reference to log service
  *
  */
inline
BitPool::BitPool(EMANE::PlatformServiceProvider * pPlatformService) :
 pPlatformService_(pPlatformService),
 u64PoolMax_(0), 
 u64PoolCurrent_(0), 
 u64FillIntervalUsec_(USEC_PER_SEC),
 tvLastRequestTime_(ACE_Time_Value::zero),
 tvFillInterval_(0, USEC_PER_SEC),
 fFillAccumulator_(0.0)
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "BitPool::%s", __func__);
}


inline
BitPool::~BitPool()
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "BitPool::%s", __func__);
}



/**
 * Set pool size
 *
 * @param fSetRequest pool size request in bits
 *
 */
inline
void BitPool::size(ACE_UINT64 u64Request)
{
  // lock mutex
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  ACE_UINT64 u64SetRequest;

  // set to 0
  if(u64Request == 0)
    {
      u64SetRequest = 0;
    }
  // less than max u64
  else if(u64Request < ACE_UINT64_MAX)
    {
      u64SetRequest = u64Request;
    }
  // cap at max u64
  else
    {
      u64SetRequest = ACE_UINT64_MAX;
    }

  // size changed
  if(u64SetRequest != u64PoolMax_) 
    {
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "BitPool::%s "
                 " current pool size "
                 ACE_UINT64_FORMAT_SPECIFIER_ASCII
                 " new pool size " 
                 ACE_UINT64_FORMAT_SPECIFIER_ASCII,
                 __func__, u64PoolMax_, u64SetRequest);

      // new size is less than current size
      if(u64PoolCurrent_ > u64SetRequest) 
        {
          // reduce current size
          u64PoolCurrent_ = u64SetRequest;
        }

      // set new max size
      u64PoolMax_ = u64SetRequest;
    }
}



/**
 * Get from pool
 *
 * @param u64Request      request size
 * @param bFullFill        continue until entire request is fulfilled
 *
 * @return  number outstanding or 0 if disabled
 *
 */
inline
ACE_UINT64 BitPool::get(ACE_UINT64 u64Request, bool bFullFill)
{
  // lock mutex
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  // disabled
  if(u64PoolMax_ == 0) {

    // nothing outstanding
    return 0;
  }

  // total acquired
  ACE_UINT64 u64Acquired = 0;

  // try to fulfill request
  while(1) {
     // get current time
     ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();

     // time to wait interval
     ACE_Time_Value tvWaitInterval;

     // make request
     u64Acquired += doDrainPool(u64Request - u64Acquired, tvCurrentTime, tvWaitInterval);
 
#ifdef VERBOSE_LOGGING
     pPlatformService_->log (EMANE::DEBUG_LEVEL, "BitPool::%s request " 
                 ACE_UINT64_FORMAT_SPECIFIER_ASCII
                 " acquired "
                 ACE_UINT64_FORMAT_SPECIFIER_ASCII
                 " remaining %4.2f%%", 
                __func__, u64Request, u64Acquired,  
                100.0 * (1.0 - (static_cast<float> (u64PoolMax_ - u64PoolCurrent_) / static_cast<float> (u64PoolMax_))));
#endif

     // request fulfilled 
     if(!bFullFill || u64Acquired == u64Request) {

       // done
       break;
     } 

     // absolute wait time 
     ACE_Time_Value tvWaitAbsoluteTime = tvCurrentTime + tvWaitInterval;

     // local mutex 
     ACE_Thread_Mutex mutex;

     // condition variable
     ACE_Condition<ACE_Thread_Mutex> cond(mutex);

     // wait here
     if(cond.wait(&tvWaitAbsoluteTime) != -1) {

       // did not time out, spurious wake up
       break;
     }
  }

  // return the result
  return u64Request - u64Acquired;
}


/**
 * Check current pool size
 *
 * @return the current pool size
 *
 */
inline
ACE_UINT64 BitPool::size()
{
  // lock mutex
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  return u64PoolCurrent_;
}


/**
 * drain the pool
 *
 * @param u64Request       request size
 * @param rTvRequestTime   request time
 * @param rTvWaitInterval  time to wait until request would be fulfilled
 *
 * @return  number available in the pool
 *
 */
inline
ACE_UINT64 BitPool::doDrainPool(ACE_UINT64 u64Request, const ACE_Time_Value & rTvRequestTime, ACE_Time_Value & rTvWaitInterval)
{
  // the result
  ACE_UINT64 u64Acquired;

  // bring the pool up to date since last request
  doFillPool(rTvRequestTime);

  // drain the pool
  if(u64Request > u64PoolCurrent_) {

    // size outstanding
    ACE_UINT64 u64Outstanding = u64Request - u64PoolCurrent_;

    // result is the available pool
    u64Acquired = u64PoolCurrent_;

    // empty the pool
    u64PoolCurrent_ = 0;

    // fill time in seconds since the fill interval in over 1 second
    const float fFillTime = static_cast<float>(u64Outstanding) / static_cast<float>(u64PoolMax_);

    // set wait interval
    rTvWaitInterval.set(fFillTime);

    // cap wait interval to fill interval
    if(rTvWaitInterval > tvFillInterval_) {
      rTvWaitInterval = tvFillInterval_;
    }
  }
  else {
    // result is the request
    u64Acquired = u64Request;

    // drain the pool by the request size
    u64PoolCurrent_ -= u64Request;

    // no wait time
    rTvWaitInterval = ACE_Time_Value::zero;
  }

  // update last request time
  tvLastRequestTime_ = rTvRequestTime;

  // return result
  return u64Acquired;
}


/**
 * add to pool
 *
 * @param rTvRequestTime   request time
 *
 */
inline
void BitPool::doFillPool(const ACE_Time_Value & rTvRequestTime)
{
  // fill the pool since last request, skip on first time around
  if(tvLastRequestTime_ != ACE_Time_Value::zero) {

    // time since last request
    const ACE_Time_Value tvRequestInterval = rTvRequestTime - tvLastRequestTime_;
    
    // fill the poll depending on elapsed time
    if(tvRequestInterval >= tvFillInterval_) {

      // fill to full capacity
      u64PoolCurrent_ = u64PoolMax_;
    }
    else {
      // find the amount of time into the fill interval
      ACE_Time_Value tvIntervalDiff = tvFillInterval_ - (tvFillInterval_ - tvRequestInterval);

      // interval difference in usec
      ACE_UINT64 u64DiffUsec = 0;

      // get time diff to usec
      tvIntervalDiff.to_usec(u64DiffUsec);

      // accumulate fill amount for this interval
      fFillAccumulator_ += (static_cast<float>(u64DiffUsec) / static_cast<float>(u64FillIntervalUsec_)) * u64PoolMax_;

      // have at least a whole unit
      if(fFillAccumulator_ >= 1.0) {
        ACE_UINT64 u64Fill = static_cast<ACE_UINT64> (fFillAccumulator_);

        // fill pool
        u64PoolCurrent_ += u64Fill;
   
        // cap at max pool size 
        if(u64PoolCurrent_ > u64PoolMax_) {
          u64PoolCurrent_ = u64PoolMax_;
        }
   
        // keep fraction remainder
        fFillAccumulator_ -= u64Fill;
      } 
    }
  }
}
