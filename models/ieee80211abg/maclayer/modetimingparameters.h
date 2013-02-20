/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef MODETIMINGPARAMTERS_HEADER_
#define MODETIMINGPARAMTERS_HEADER_

#include "macconfig.h"

#include <ace/Basic_Types.h>
#include <ace/Time_Value.h>

#include <map>

namespace IEEE80211ABG
{

/* Forward declaration */
  class IEEE80211abgMACLayer;


/**
*
* @brief structure used to define frame timing parameters
*
*/


/**
*
* @brief class used to define timing parameters
*
*/
  class ModeTimingParameters
  {
  private:
    const MACConfig & macConfig_;

  /**
  *
  * @brief structure used to define timing parameters for each mode
  *
  */
    struct TimingParams
    {
      ACE_UINT16 u16RtsBitLength_;      // rts bit length
      ACE_UINT16 u16CtsBitLength_;      // cts bit length
      ACE_UINT16 u16AckBitLength_;      // ack bit length
      ACE_UINT32 u32SlotTimeUsec_;      // slot interval
      ACE_UINT32 u32SifsTimeUsec_;      // sifs interval
      ACE_UINT32 u32PreambleTimeUsec_;  // preamble interval

        TimingParams ():u16RtsBitLength_ (0),
        u16CtsBitLength_ (0),
        u16AckBitLength_ (0), u32SlotTimeUsec_ (0),
        u32SifsTimeUsec_ (0), u32PreambleTimeUsec_ (0)
      {
      };
    };

    TimingParams timingParams_[MODULATION_TYPE_INDEX_MAX + 1];


    ACE_UINT16 getRtsBitLength (MODULATION_TYPE) const;

    ACE_UINT16 getCtsBitLength (MODULATION_TYPE) const;

    ACE_UINT16 getAckBitLength (MODULATION_TYPE) const;


    ACE_UINT32 getSifsTimeUsec (MODULATION_TYPE) const;

    ACE_UINT32 getPreambleTimeUsec (MODULATION_TYPE) const;

    ACE_UINT32 getPropagationTimeUsec (ACE_UINT32) const;


    ACE_Time_Value getBroadcastMessageDuration (size_t) const;

    ACE_Time_Value getUnicastMessageDuration (size_t) const;

    ACE_Time_Value getCtsMessageDuration () const;

    ACE_Time_Value getRtsMessageDuration () const;


  public:
    ModeTimingParameters (const MACConfig & macConfig);

    ~ModeTimingParameters ();

    bool packetTimedOut (const ACE_Time_Value &, const ACE_Time_Value &) const;

    int getContentionWindow (ACE_UINT8, ACE_UINT8) const;

    ACE_UINT32 getSlotTimeUsec () const;

    ACE_Time_Value getOverheadInterval (ACE_UINT8 idx) const;

    ACE_Time_Value getMessageDuration (ACE_UINT8, size_t) const;

    ACE_Time_Value getDeferInterval (ACE_UINT8) const;

    ACE_Time_Value getSotTime () const;
  };
}

#endif // MODETIMINGPARAMTERS_HEADER_
