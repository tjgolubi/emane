/*
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef IEEE80211ABGMACHEADER_HEADER_
#define IEEE80211ABGMACHEADER_HEADER_

#include "emaneutils/netutils.h"
#include "msgtypes.h"

#include <ace/Time_Value.h>
#include <ace/Basic_Types.h>

#include <string>


namespace IEEE80211ABG {


 /**
  *
  * @brief high fidelity mac header.
  *
  */
  class MacHeader
  {

    public:
   /**
    *
    * @brief high fidelity mac header initializer.
    *
    */
      MacHeader ():
      u8MsgType_ (0),
      u8Retries_ (0),
      u16DataRateIndex_ (0), 
      u16Seq_ (0),
      u16SrcNEM_(0),
      u16DstNEM_(0),
      u32DurationSec_(0),
      u32DurationUsec_(0)
    { }
   /**
    *
    * @brief high fidelity mac header initializer.
    *
    * @param type     msg type data, rtc, cts, ack
    * @param retries  retry count
    * @param rate     data rate index
    * @param seq      sequence number
    * @param src      source NEM
    * @param dst      destination NEM
    * @param duration message duration
    *
    */ MacHeader (ACE_UINT8 type,
                  ACE_UINT8 retries,
                  ACE_UINT16 rate,
                  ACE_UINT16 seq,
                  ACE_UINT16 src,
                  ACE_UINT16 dst,
                  const ACE_Time_Value & duration):
      u8MsgType_ (type),
      u8Retries_ (retries),
      u16DataRateIndex_ (rate),
      u16Seq_ (seq),
      u16SrcNEM_(src),
      u16DstNEM_(dst), 
      u32DurationSec_(duration.sec()),
      u32DurationUsec_(duration.usec())
     { } 

    ACE_UINT8 getType () const
    {
      return u8MsgType_;
    }

    ACE_UINT8 getRetries () const
    {
      return u8Retries_;
    }

    ACE_UINT16 getDataRateIndex () const
    {
      return u16DataRateIndex_;
    }

    ACE_UINT16 getSequence () const
    {
      return u16Seq_;
    }

    ACE_UINT16 getSrcNEM () const
    {
      return u16SrcNEM_;
    }

    ACE_UINT16 getDstNEM () const
    {
      return u16DstNEM_;
    }

    ACE_Time_Value getDuration() const
    {
      return ACE_Time_Value(u32DurationSec_, u32DurationUsec_);
    }

    std::string format () const
    {
        char fmtBuff[256];

        snprintf (fmtBuff, sizeof (fmtBuff),
                  "ieeemachdr: type %s, src %hu, dst %hu, duration %ld:%06ld retries %02hhu, datarate index %hu, seq %02hu",
                  typeToString(getType ()).c_str(),
                  getSrcNEM(),
                  getDstNEM(),
                  getDuration().sec(),
                  getDuration().usec(),
                  getRetries (), 
                  getDataRateIndex (), 
                  getSequence ());

        return fmtBuff;
    }


   /**
    *
    * @brief convert mac header from host to network byte order.
    *
    */ void toNetworkByteOrder ()
    {
      // short
      u16Seq_           = ACE_HTONS (u16Seq_);
      u16DataRateIndex_ = ACE_HTONS (u16DataRateIndex_);
      u16SrcNEM_        = ACE_HTONS (u16SrcNEM_);
      u16DstNEM_        = ACE_HTONS (u16DstNEM_);

      // long
      u32DurationSec_   = ACE_HTONL(u32DurationSec_);
      u32DurationUsec_  = ACE_HTONL(u32DurationUsec_);
    }


   /**
    *
    * @brief convert mac header from network to host byte order.
    *
    */
    void toHostByteOrder ()
    {
      // short
      u16Seq_           = ACE_NTOHS (u16Seq_);
      u16DataRateIndex_ = ACE_NTOHS (u16DataRateIndex_);
      u16SrcNEM_        = ACE_NTOHS (u16SrcNEM_);
      u16DstNEM_        = ACE_NTOHS (u16DstNEM_);

      // long
      u32DurationSec_   = ACE_NTOHL(u32DurationSec_);
      u32DurationUsec_  = ACE_NTOHL(u32DurationUsec_);
    }

   private:
    ACE_UINT8  u8MsgType_;          // msg type
    ACE_UINT8  u8Retries_;          // retry count

    ACE_UINT16 u16DataRateIndex_;   // data rate index
    ACE_UINT16 u16Seq_;             // sequence number
    ACE_UINT16 u16SrcNEM_;          // src NEM
    ACE_UINT16 u16DstNEM_;          // dst NEM

    ACE_UINT32 u32DurationSec_;     // duration in sec
    ACE_UINT32 u32DurationUsec_;    // duration in usec

    std::string typeToString(ACE_UINT8 type) const
     {
       switch(type)
        {
          case MSG_TYPE_BROADCAST_DATA:
            return "BROADCAST_DATA";

          case MSG_TYPE_UNICAST_DATA:
            return "UNICAST_DATA";

          case MSG_TYPE_UNICAST_RTS_CTS_DATA:
            return "UNICAST_RTS_CTS_DATA";

          case MSG_TYPE_UNICAST_CTS_CTRL:
            return "UNICAST_CTS_CTRL";

          case MSG_TYPE_NONE:
            return "NONE";
        }
       
        return "UNKNOWN";
     }
  } __attribute__ ((packed));
}

#endif //IEEE80211ABGMACHEADER_HEADER_
