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

#ifndef BITPOOL_HEADER_
#define BITPOOL_HEADER_

#include <ace/Time_Value.h>
#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>
#include <ace/Guard_T.h>

#include "emane/emaneplatformserviceprovider.h"

  /**
   * @class BitPool
   *
   * @brief Bit pool implementation
   *
   * @note Uses a thread to allow blocking dequeues
   */

class BitPool
{
  public:
   BitPool(EMANE::PlatformServiceProvider * pPlatformService);

   ~BitPool();

   ACE_UINT64 get(ACE_UINT64 u64Request, bool bFullFill = true);

   ACE_UINT64 size();

   void size(ACE_UINT64 u64Request);

  private:
   EMANE::PlatformServiceProvider * pPlatformService_;
   ACE_UINT64            u64PoolMax_;
   ACE_UINT64            u64PoolCurrent_;
   const ACE_UINT64      u64FillIntervalUsec_;
   ACE_Time_Value        tvLastRequestTime_;
   const ACE_Time_Value  tvFillInterval_;
   float                 fFillAccumulator_;
   ACE_Thread_Mutex      mutex_;

   void doFillPool(const ACE_Time_Value & rTvRequestTime);

   ACE_UINT64 doDrainPool(ACE_UINT64 u64Request, const ACE_Time_Value & rTvRequestTime, ACE_Time_Value & rTvWaitInterval);
};

#include "bitpool.inl"

#endif // BITPOOL_HEADER_
