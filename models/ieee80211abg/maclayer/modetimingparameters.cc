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

#include "modetimingparameters.h"
#include "emane/emanepacketinfo.h"
#include "macconfig.h"
#include "macstatistics.h"
#include "ieee80211abgmacheader.h"
#include "ieee80211abgmaclayer.h"

#include <ace/OS_NS_time.h>

#include "utils.h"

namespace {
 const ACE_UINT8 RTS_BIT_LENGTH_DEFAULT = 160;
 const ACE_UINT8 RTS_BIT_LENGTH_80211A = 160;
 const ACE_UINT8 RTS_BIT_LENGTH_80211B = 160;
 const ACE_UINT8 RTS_BIT_LENGTH_80211BG = 160;

 const ACE_UINT8 CTS_BIT_LENGTH_DEFAULT = 112;
 const ACE_UINT8 CTS_BIT_LENGTH_80211A = 112;
 const ACE_UINT8 CTS_BIT_LENGTH_80211B = 112;
 const ACE_UINT8 CTS_BIT_LENGTH_80211BG = 112;

 const ACE_UINT8 ACK_BIT_LENGTH_DEFAULT = 112;
 const ACE_UINT8 ACK_BIT_LENGTH_80211A = 112;
 const ACE_UINT8 ACK_BIT_LENGTH_80211B = 112;
 const ACE_UINT8 ACK_BIT_LENGTH_80211BG = 112;

 const ACE_UINT16 SLOT_TIME_USEC_DEFAULT = 9;
 const ACE_UINT16 SLOT_TIME_USEC_80211A = 9;
 const ACE_UINT16 SLOT_TIME_USEC_80211B = 20;
 const ACE_UINT16 SLOT_TIME_USEC_80211BG = 20;

 const ACE_UINT16 SIFS_TIME_USEC_DEFAULT = 10;
 const ACE_UINT16 SIFS_TIME_USEC_80211A = 16;
 const ACE_UINT16 SIFS_TIME_USEC_80211B = 10;
 const ACE_UINT16 SIFS_TIME_USEC_80211BG = 16;

 const ACE_UINT16 PREAMBLE_TIME_USEC_DEFAULT = 192;
 const ACE_UINT16 PREAMBLE_TIME_USEC_80211A = 20;
 const ACE_UINT16 PREAMBLE_TIME_USEC_80211B = 192;
 const ACE_UINT16 PREAMBLE_TIME_USEC_80211BG = 192;

 const float SLOT_TIME_DISTANCE_DIVISOR = 300.0;

 const ACE_UINT16 IEEE_80211MAC_DATAHEADER_BITLEN = 272;
 const ACE_UINT16 IEEE_80211MAC_CTRLHEADER_BITLEN = 160;
}

/**
*
* @brief constructor
*
*/
IEEE80211ABG::ModeTimingParameters::ModeTimingParameters (const MACConfig & macConfig):
 macConfig_ (macConfig)
{
  // default
  timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u16RtsBitLength_ = RTS_BIT_LENGTH_DEFAULT;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u16CtsBitLength_ = CTS_BIT_LENGTH_DEFAULT;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u16AckBitLength_ = ACK_BIT_LENGTH_DEFAULT;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u32SlotTimeUsec_ = SLOT_TIME_USEC_DEFAULT;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u32SifsTimeUsec_ = SIFS_TIME_USEC_DEFAULT;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u32PreambleTimeUsec_ = PREAMBLE_TIME_USEC_DEFAULT;

  // 802.11a (OFDM) 
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u16RtsBitLength_ = RTS_BIT_LENGTH_80211A;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u16CtsBitLength_ = CTS_BIT_LENGTH_80211A;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u16AckBitLength_ = ACK_BIT_LENGTH_80211A;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u32SlotTimeUsec_ = SLOT_TIME_USEC_80211A;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u32SifsTimeUsec_ = SIFS_TIME_USEC_80211A;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u32PreambleTimeUsec_ = PREAMBLE_TIME_USEC_80211A;

  // 802.11b (DSS) 
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u16RtsBitLength_ = RTS_BIT_LENGTH_80211B;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u16CtsBitLength_ = CTS_BIT_LENGTH_80211B;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u16AckBitLength_ = ACK_BIT_LENGTH_80211B;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u32SlotTimeUsec_ = SLOT_TIME_USEC_80211B;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u32SifsTimeUsec_ = SIFS_TIME_USEC_80211B;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u32PreambleTimeUsec_ = PREAMBLE_TIME_USEC_80211B;

  // 802.11b/g (MIXED) 
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u16RtsBitLength_ = RTS_BIT_LENGTH_80211BG;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u16CtsBitLength_ = CTS_BIT_LENGTH_80211BG;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u16AckBitLength_ = ACK_BIT_LENGTH_80211BG;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u32SlotTimeUsec_ = SLOT_TIME_USEC_80211BG;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u32SifsTimeUsec_ = SIFS_TIME_USEC_80211BG;
  timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u32PreambleTimeUsec_ = PREAMBLE_TIME_USEC_80211BG;
}


/**
*
* @brief destructor
*
*/
IEEE80211ABG::ModeTimingParameters::~ModeTimingParameters ()
{ }


/**
*
* @brief get the rts length for a given mode
*
* @param mode modulation type
*
* @retval rts length in bits
*
*/
ACE_UINT16 IEEE80211ABG::ModeTimingParameters::getRtsBitLength (MODULATION_TYPE mode) const
{
  switch (mode)
    {
      case IEEE80211ABG::MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u16RtsBitLength_;

      case IEEE80211ABG::MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u16RtsBitLength_;

      case IEEE80211ABG::MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u16RtsBitLength_; 

      default:             // default
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u16RtsBitLength_;
    }
}


/**
*
* @brief get the cts length for a given mode
*
* @param mode modulation type
*
* @retval cts length in bits
*
*/
ACE_UINT16
IEEE80211ABG::ModeTimingParameters::getCtsBitLength (MODULATION_TYPE mode) const 
{
  switch (mode)
    {
      case IEEE80211ABG::MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u16CtsBitLength_;

      case IEEE80211ABG::MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u16CtsBitLength_;

      case IEEE80211ABG::MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u16CtsBitLength_;

      default:             // default
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u16CtsBitLength_;
    }
}


/**
*
* @brief get the ack length for a given mode
*
* @param mode modulation type
*
* @retval ack length in bits
*
*/
ACE_UINT16
IEEE80211ABG::ModeTimingParameters::getAckBitLength (MODULATION_TYPE mode) const 
{
  switch (mode)
    {
      case IEEE80211ABG::MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u16AckBitLength_;

      case IEEE80211ABG::MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u16AckBitLength_;

      case IEEE80211ABG::MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u16AckBitLength_;

      default:             // default
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u16AckBitLength_;
    }
}


/**
*
* @brief get the slot time for a given mode and distance
*
* @retval slot time length in usec
*
*/
ACE_UINT32
IEEE80211ABG::ModeTimingParameters::getSlotTimeUsec () const 
{
  const MODULATION_TYPE mode = macConfig_.getModulationType ();

  const ACE_UINT32 dist = macConfig_.getMaxP2pDistance ();

  switch (mode)
    {
      case IEEE80211ABG::MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u32SlotTimeUsec_ + getPropagationTimeUsec (dist);

      case IEEE80211ABG::MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u32SlotTimeUsec_ + getPropagationTimeUsec (dist);

      case IEEE80211ABG::MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u32SlotTimeUsec_ + getPropagationTimeUsec (dist);

      default:             // default
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u32SlotTimeUsec_ + getPropagationTimeUsec (dist);
    }
}


/**
*
* @brief get the sifs time for a given mode
*
* @param mode modulation type
*
* @retval sifs length in usec
*
*/ 
ACE_UINT32
IEEE80211ABG::ModeTimingParameters::getSifsTimeUsec (MODULATION_TYPE mode) const
{
  switch (mode)
    {
      case IEEE80211ABG::MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u32SifsTimeUsec_;

      case IEEE80211ABG::MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u32SifsTimeUsec_;

      case IEEE80211ABG::MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u32SifsTimeUsec_;
  
      default:             // default
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u32SifsTimeUsec_;
    }
}


/**
*
* @brief get the preamble time for a given mode
*
* @param mode modulation type
*
* @retval sifs length in usec
*
*/ 
ACE_UINT32
IEEE80211ABG::ModeTimingParameters::getPreambleTimeUsec (MODULATION_TYPE mode) const
{
  switch (mode)
    {
      case IEEE80211ABG::MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211A].u32PreambleTimeUsec_;
 
      case IEEE80211ABG::MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211B].u32PreambleTimeUsec_;

      case IEEE80211ABG::MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_80211BG].u32PreambleTimeUsec_;

      default:             // default
        return timingParams_[IEEE80211ABG::MODULATION_TYPE_DEFAULT].u32PreambleTimeUsec_;
    }
}


/**
*
* @brief get the propagation time for a given distance
*
* @param dist distance in meters
*
* @retval propagation time in usec
*
*/ 
ACE_UINT32
IEEE80211ABG::ModeTimingParameters::getPropagationTimeUsec (ACE_UINT32 dist) const
{
  return dist / SLOT_TIME_DISTANCE_DIVISOR;
}


/**
*
* @brief get the defer time
*
* @param idx queue index
*
* @retval tv the defer interval in sec and usec
*
*/ 
ACE_Time_Value
IEEE80211ABG::ModeTimingParameters::getDeferInterval (ACE_UINT8 idx) const
{
  const MODULATION_TYPE mode = macConfig_.getModulationType ();

  const double dUseconds = macConfig_.getAifsUsec (idx) * getSlotTimeUsec () + getSifsTimeUsec (mode);

  return (IEEE80211ABG::USEC_TO_TV (dUseconds));
}

/**
*
* @brief get the overhead time
*
* @param idx queue index
*
* @retval tv the overhead interval in sec and usec
*
*/ 
ACE_Time_Value
IEEE80211ABG::ModeTimingParameters::getOverheadInterval (ACE_UINT8 idx) const
{
  const MODULATION_TYPE mode = macConfig_.getModulationType ();

  const double dUseconds = getSlotTimeUsec () * 
                             (macConfig_.getAifsUsec (idx) + (macConfig_.getCwMin (idx) / 2.0)) + 
                               getSifsTimeUsec (mode);

  return (IEEE80211ABG::USEC_TO_TV (dUseconds));
}





/**
*
* @brief check if a packet has timed out
*
* @param tvTxOpTime packet txop time
* @param tvRxTime   packet rx time
*
* @retval return true if packet has timed out, else return false.
*
*/
bool
IEEE80211ABG::ModeTimingParameters::packetTimedOut (const ACE_Time_Value & tvTxOpTime, const ACE_Time_Value & tvRxTime) const
{
  // txop is enabled when non zero
  if (tvTxOpTime != ACE_Time_Value::zero)
    {
      // timed out if sent time plus txop is in the past
      return (tvRxTime + tvTxOpTime) < ACE_OS::gettimeofday ();
    }

  return false;
}




/**
*
* @brief get the message duration
*
* @param type packet type (rts, cts, ack, data)
* @param numBytes packet size in bytes
*
* @retval tv the transmission duration in sec and usec
*
*/
ACE_Time_Value
IEEE80211ABG::ModeTimingParameters::getMessageDuration (ACE_UINT8 type, size_t numBytes) const
{
  switch (type)
    {
      // cts
      case IEEE80211ABG::MSG_TYPE_UNICAST_CTS_CTRL:
         return getCtsMessageDuration();

      // unicast data
      case IEEE80211ABG::MSG_TYPE_UNICAST_DATA:
         return getUnicastMessageDuration (numBytes);

      // unicast data rst-cts
      case IEEE80211ABG::MSG_TYPE_UNICAST_RTS_CTS_DATA:
           return getUnicastMessageDuration (numBytes) + getRtsMessageDuration() + getCtsMessageDuration();

      // broadcast data
      case IEEE80211ABG::MSG_TYPE_BROADCAST_DATA:
         return getBroadcastMessageDuration (numBytes);

      default:
        return ACE_Time_Value::zero;
    }
}


ACE_Time_Value IEEE80211ABG::ModeTimingParameters::getBroadcastMessageDuration (size_t numBytes) const
{
  const MODULATION_TYPE mode = macConfig_.getModulationType ();

  const double dUseconds = getPreambleTimeUsec (mode) + 
                           ((IEEE_80211MAC_DATAHEADER_BITLEN + (numBytes * 8.0)) / (macConfig_.getMulticastDataRateKbps () / 1000.0));

  return (IEEE80211ABG::USEC_TO_TV (dUseconds));
}



ACE_Time_Value IEEE80211ABG::ModeTimingParameters::getUnicastMessageDuration (size_t numBytes) const
{
  const MODULATION_TYPE mode = macConfig_.getModulationType ();

  double dUseconds = getSifsTimeUsec (mode) + (2.0 * getPreambleTimeUsec (mode)) +
                     ((IEEE_80211MAC_DATAHEADER_BITLEN + getAckBitLength (mode) + (numBytes * 8.0)) / (macConfig_.getUnicastDataRateKbps () / 1000.0));

  return (IEEE80211ABG::USEC_TO_TV (dUseconds));
}



ACE_Time_Value IEEE80211ABG::ModeTimingParameters::getCtsMessageDuration () const
{
  const MODULATION_TYPE mode = macConfig_.getModulationType ();

  const double dUseconds = getPreambleTimeUsec (mode) + (IEEE_80211MAC_CTRLHEADER_BITLEN / (macConfig_.getUnicastDataRateKbps () / 1000.0));

  return (IEEE80211ABG::USEC_TO_TV (dUseconds));
}



ACE_Time_Value IEEE80211ABG::ModeTimingParameters::getRtsMessageDuration () const
{
  const MODULATION_TYPE mode = macConfig_.getModulationType ();

  const double dUseconds = getPreambleTimeUsec (mode) + (IEEE_80211MAC_CTRLHEADER_BITLEN / (macConfig_.getUnicastDataRateKbps () / 1000.0));

  return (IEEE80211ABG::USEC_TO_TV (dUseconds));
}




/**
*
* @brief get the contention window
*
* @param idx      queue index
* @param tries    number of retires
*
* @retval cw the contention interval
*
*/ 
int
IEEE80211ABG::ModeTimingParameters::getContentionWindow (ACE_UINT8 idx, ACE_UINT8 tries) const
{
  const int min = macConfig_.getCwMin (idx);
  const int max = macConfig_.getCwMax (idx);

  int cw = min * pow (2.0, tries);

  if (cw > max)
    {
      cw = max;
    }
  else if (cw < min)
    {
      cw = min;
    }

  return cw;
}
