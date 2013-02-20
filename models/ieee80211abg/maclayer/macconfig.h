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

#ifndef MACCONFIG_HEADER_
#define MACCONFIG_HEADER_


#include "emane/emanepacketinfo.h"

#include <ace/Basic_Types.h>
#include <ace/Time_Value.h>
#include <ace/Thread_Mutex.h>
#include <ace/Singleton.h>
#include <ace/Assert.h>

#include <string>
#include <map>
#include <vector>

namespace IEEE80211ABG
{

// currently supporting modes 802.11a, 802.11b, 802.11b/g
  enum MODULATION_TYPE
  { MODULATION_TYPE_DEFAULT = 0x0,
    MODULATION_TYPE_80211A  = 0x1,
    MODULATION_TYPE_80211B  = 0x2,
    MODULATION_TYPE_80211BG = 0x3
  };


// modulation type
  const ACE_UINT8 MODULATION_TYPE_INDEX_MIN     = 0;
  const ACE_UINT8 MODULATION_TYPE_INDEX_MAX     = 3;
  const ACE_UINT8 MODULATION_TYPE_INDEX_DEFAULT = 0;


// promiscous mode
  const bool PROMISCOUS_ENABLE_DEFAULT = false;

// max allowable pkt size
  const ACE_UINT16 MAX_PKT_SIZE_DEFAULT = 0xFFFF;

// max distance for ptp link
  const ACE_UINT32 MAX_DISTANCE_MIN     = 0;
  const ACE_UINT32 MAX_DISTANCE_MAX     = 0xFFFFFFFF;
  const ACE_UINT32 MAX_DISTANCE_DEFAULT = 1000;

// multicast data rate index
  const ACE_UINT8 MULTICAST_DATARATE_INDEX_MIN     = 1;
  const ACE_UINT8 MULTICAST_DATARATE_INDEX_MAX     = 12;
  const ACE_UINT8 MULTICAST_DATARATE_INDEX_DEFAULT = 1;

// unicast data rate index
  const ACE_UINT8 UNICAST_DATARATE_INDEX_MIN     = 1;
  const ACE_UINT8 UNICAST_DATARATE_INDEX_MAX     = 12;
  const ACE_UINT8 UNICAST_DATARATE_INDEX_DEFAULT = 4;

// wireless multi media
  const bool WMM_ENABLE_DEFAULT = false;

// flow control default
  const bool FLOW_CONTROL_ENABLE_DEFAULT = false;

// flow control tokens
  const ACE_UINT16 FLOW_CONTROL_TOKENS_MIN     = 1;
  const ACE_UINT16 FLOW_CONTROL_TOKENS_MAX     = 0xFFFF;
  const ACE_UINT16 FLOW_CONTROL_TOKENS_DEFAULT = 10;

// number of queues
  const ACE_UINT8 NUM_ACCESS_CATEGORIES_MIN     = 1;
  const ACE_UINT8 NUM_ACCESS_CATEGORIES_MAX     = 4;
  const ACE_UINT8 NUM_ACCESS_CATEGORIES_DEFAULT = 4;

// queue length (depth)
  const ACE_UINT16 QUEUE_SIZE_MIN     = 0;
  const ACE_UINT16 QUEUE_SIZE_MAX     = 255;
  const ACE_UINT16 QUEUE_SIZE_DEFAULT = 255;

// contention window min
  const ACE_UINT16 CWMIN_MIN     = 1;
  const ACE_UINT16 CWMIN_MAX     = 0xFFFF;
  const ACE_UINT16 CWMIN_DEFAULT = 31;

// contention window max
  const ACE_UINT16 CWMAX_MIN     = 1;
  const ACE_UINT16 CWMAX_MAX     = 0xFFFF;
  const ACE_UINT16 CWMAX_DEFAULT = 1023;

// arbitration inter frame spacing time
  const ACE_UINT32 AIFS_USEC_MIN     = 0;
  const ACE_UINT32 AIFS_USEC_MAX     = 255;
  const ACE_UINT32 AIFS_USEC_DEFAULT = 2;

// tx oppurtunity time 
  const ACE_UINT32 TXOP_TIME_USEC_MIN     = 0;
  const ACE_UINT32 TXOP_TIME_USEC_MAX     = 1000000;
  const ACE_UINT32 TXOP_TIME_USEC_DEFAULT = 0;

// number of retries
  const ACE_UINT8 RETRY_LIMIT_MIN     = 0;
  const ACE_UINT8 RETRY_LIMIT_MAX     = 255;
  const ACE_UINT8 RETRY_LIMIT_DEFAULT = 3;

// rts threshold
  const ACE_UINT16 RTS_THRESHOLD_MIN     = 0;
  const ACE_UINT16 RTS_THRESHOLD_MAX     = 0xFFFF;
  const ACE_UINT16 RTS_THRESHOLD_DEFAULT = 256;

// neighbor timeout
  const ACE_UINT16 NEIGHBOR_TIMEOUT_SEC_MIN     = 0;
  const ACE_UINT16 NEIGHBOR_TIMEOUT_SEC_MAX     = 3600;
  const ACE_UINT16 NEIGHBOR_TIMEOUT_SEC_DEFAULT = 30;

// channel activity timer
  const ACE_UINT16 CHANNEL_ACTIVITY_TIMER_MSEC_MIN     = 1;
  const ACE_UINT16 CHANNEL_ACTIVITY_TIMER_MSEC_MAX     = 1000;
  const ACE_UINT16 CHANNEL_ACTIVITY_TIMER_MSEC_DEFAULT = 100;

// sot wait intwerval
  const ACE_Time_Value SOT_WAIT_TIME_DEFAULT = ACE_Time_Value (0, 1000);

/**
  *
  * @brief ieee80211abg mac queue configuration items.
  *
  */
  struct QueueConfigItems
  {
    ACE_UINT16     u16QueueSize_;    // queue size
    ACE_UINT16     u16CwMin_;        // contention window min
    ACE_UINT16     u16CwMax_;        // contention window max;
    ACE_UINT32     u32AifsUsec_;     // aifs
    ACE_Time_Value tvTxOpTime_;      // tx opportunity
    ACE_UINT8      u8RetryLimit_;    // retry limit

      QueueConfigItems ():
       u16QueueSize_ (QUEUE_SIZE_DEFAULT),
       u16CwMin_ (CWMIN_DEFAULT),
       u16CwMax_ (CWMAX_DEFAULT),
       u32AifsUsec_ (AIFS_USEC_DEFAULT),
       tvTxOpTime_ (ACE_Time_Value (0, TXOP_TIME_USEC_DEFAULT)),
       u8RetryLimit_ (RETRY_LIMIT_DEFAULT)
    { }
  };


  typedef std::vector < QueueConfigItems > QueueConfogItemsVector;

/**
  *
  * @brief mac configuration items.
  *
  */
  struct ConfigItems
  {
    ACE_UINT8       u8ModeIndex_;                     // mode index (modulation type)
    bool            bPromiscousEnable_;               // enable promiscous mode
    ACE_UINT32      u32MaxP2pDistance_;               // max p2p distance
    ACE_UINT16      u16UnicastDataRateIndex_;         // unicast data rate index
    ACE_UINT16      u16MulticastDataRateIndex_;       // multicast data rate index
    ACE_UINT16      u16RtsThreshold_;                 // rtc cts enable threshold
    bool            bWmmEnable_;                      // enable wmm
    ACE_UINT8       u8NumAccessCategories_;           // number of access categories
    bool            bFlowControlEnable_;              // flow control enable
    ACE_UINT16      u16FlowControlTokens_;            // flow control tokens
    std::string     sPcrUri_;                         // pcr uri
    ACE_Time_Value  tvNeighborTimeout_;               // neighbor timeout
    ACE_Time_Value  tvChannelActivityTimerInterval_;  // channel activity interval

    QueueConfogItemsVector queueConfigVec_;

    ConfigItems ();
  };


/**
*
* @brief class used to define the mac layer configuration items
*
*/
  class MACConfig
  {
  /**
  *
  * @brief structue used to define an index/value queue configuration pair
  *
  */
    struct IndexValuePair
    {
      ACE_UINT8 index_;         // index
      float value_;            // value

      IndexValuePair ()
      {
        index_ = 0;
        value_ = 0;
      }
    };

    typedef std::vector < struct IndexValuePair >IndexValueVector;

  private:
    ConfigItems configItems_;

  public:
    MACConfig ();

    ~MACConfig ();

    bool getPromiscuosEnable () const;

    bool getWmmEnable () const;

    MODULATION_TYPE getModulationType () const;

    ACE_UINT16 getUnicastDataRateIndex () const;

    ACE_UINT16 getMulticastDataRateIndex () const;

    ACE_UINT32 getUnicastDataRateKbps () const;

    ACE_UINT32 getMulticastDataRateKbps () const;

    ACE_UINT32 getMaxP2pDistance () const;

    ACE_UINT8 getNumAccessCategories () const;

    ACE_UINT16 getRtsThreshold () const;

    ACE_UINT16 getQueueSize (ACE_UINT8) const;

    ACE_UINT16 getCwMin (ACE_UINT8) const;

    ACE_UINT16 getCwMax (ACE_UINT8) const;

    ACE_UINT32 getAifsUsec (ACE_UINT8) const;

    ACE_Time_Value getTxOpTime (ACE_UINT8) const;

    ACE_UINT8 getRetryLimit (ACE_UINT8) const;

    ACE_UINT16 getFlowControlTokens () const;

    bool getFlowControlEnable () const;

    std::string getPcrUri () const;

    ACE_Time_Value getNeighborTimeout () const;

    ACE_Time_Value getChannelActivityTimerInterval () const;

    bool set (const std::string, const std::string);

    bool verify (std::string &) const;

    bool getValues (const std::string &, IndexValueVector &, float min, float max) const;
  };
}

#endif //MACCONFIG_HEADER_
