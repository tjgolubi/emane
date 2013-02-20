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

#ifndef DOWNSTREAMQUEUE_HEADER_
#define DOWNSTREAMQUEUE_HEADER_

#include "emane/emanemaclayerimpl.h"
#include "emane/emaneplatformserviceprovider.h"
#include "emane/emanestatisticunsignedinteger32.h"
#include "macconfig.h"

#include <queue>
#include <sstream>

#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>
#include <ace/Task.h>


namespace IEEE80211ABG
{

/**
*
* @brief structure defines the mac downstream packet queue entry
*
*/
  struct DownstreamQueueEntry
  {
    EMANE::DownstreamPacket pkt_;                    // payload
    ACE_Time_Value          tvAcquireTime_;          // acquire time from network layer absolute
    ACE_Time_Value          tvDurationInterval_;     // duration interval
    ACE_Time_Value          tvPreTxDelayTime_;       // pre tx delay time absolute
    ACE_Time_Value          tvPostTxDelayInterval_;  // post tx delay time absolute
    ACE_Time_Value          tvTxTime_;               // tx time absolute
    ACE_UINT16              u16Seq_;                 // sequence number
    ACE_UINT8               qidx_;                   // queue index
    size_t                  numRetries_;             // number of retries count
    size_t                  maxRetries_;             // max retry count
    bool                    bRtsCtsEnable_;          // rts cts enable
    bool                    bCollisionOccured_;      // flag that a tx collision occured

      DownstreamQueueEntry ():
      pkt_ (EMANE::PacketInfo (), 0, 0),
      tvAcquireTime_ (ACE_Time_Value::zero),
      tvDurationInterval_ (ACE_Time_Value::zero),
      tvPreTxDelayTime_ (ACE_Time_Value::zero),
      tvPostTxDelayInterval_ (ACE_Time_Value::zero),
      tvTxTime_ (ACE_Time_Value::zero), 
      u16Seq_ (0), 
      qidx_ (0),
      numRetries_ (0), 
      maxRetries_ (0),
      bRtsCtsEnable_(false),
      bCollisionOccured_(false)
    { } 

     DownstreamQueueEntry (EMANE::DownstreamPacket pkt, ACE_Time_Value tv, ACE_UINT8 qidx):
      pkt_ (pkt),
      tvAcquireTime_ (tv), 
      tvDurationInterval_ (ACE_Time_Value::zero),
      tvPreTxDelayTime_ (ACE_Time_Value::zero),
      tvPostTxDelayInterval_ (ACE_Time_Value::zero),
      tvTxTime_ (ACE_Time_Value::zero), 
      u16Seq_ (0), 
      qidx_ (qidx),
      numRetries_ (0), 
      maxRetries_ (0),
      bRtsCtsEnable_(false),
      bCollisionOccured_(false)
    { }
  };


  typedef std::queue < DownstreamQueueEntry > IEEE80211abgDownstreamPacketQueue;

/**
 * @brief Defines an access category container
 *
 *
 */
  struct AccessCategory
  {
    ACE_UINT8 category_;

    size_t maxQueueSize_;

    IEEE80211abgDownstreamPacketQueue queue_;

    /*
     * Stat counters
     */
    // unicast
      EMANE::StatisticUnsignedInteger32 numUcastPacketsEnqueued_;
      EMANE::StatisticUnsignedInteger32 numUcastBytesEnqueued_;

      EMANE::StatisticUnsignedInteger32 numUcastPacketsDequeued_;
      EMANE::StatisticUnsignedInteger32 numUcastBytesDequeued_;

      EMANE::StatisticUnsignedInteger32 numUcastPacketsOverFlow_;
      EMANE::StatisticUnsignedInteger32 numUcastBytesOverFlow_;

      EMANE::StatisticUnsignedInteger32 numUcastPacketsTooLarge_;
      EMANE::StatisticUnsignedInteger32 numUcastBytesTooLarge_;

    // multicast
      EMANE::StatisticUnsignedInteger32 numMcastPacketsEnqueued_;
      EMANE::StatisticUnsignedInteger32 numMcastBytesEnqueued_;

      EMANE::StatisticUnsignedInteger32 numMcastPacketsDequeued_;
      EMANE::StatisticUnsignedInteger32 numMcastBytesDequeued_;

      EMANE::StatisticUnsignedInteger32 numMcastPacketsOverFlow_;
      EMANE::StatisticUnsignedInteger32 numMcastBytesOverFlow_;

      EMANE::StatisticUnsignedInteger32 numMcastPacketsTooLarge_;
      EMANE::StatisticUnsignedInteger32 numMcastBytesTooLarge_;

    // queue size/depth
      EMANE::StatisticUnsignedInteger32 numHighWaterMark_;
      EMANE::StatisticUnsignedInteger32 numHighWaterMax_;
      EMANE::StatisticUnsignedInteger32 numEntrySizeMax_;

    /*
     * Constructor
     */
      AccessCategory ()
    { } 
    void registerStatistics(int groupId, EMANE::StatisticServiceProvider* pOwner)
    {
       std::stringstream ss;
       ss << groupId;

       std::string name;

       name = "numUcastPacketsEnqueued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastPacketsEnqueued_);
       name.clear();

       name = "numUcastBytesEnqueued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastBytesEnqueued_);
       name.clear();

       name = "numUcastPacketsDequeued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastPacketsDequeued_);
       name.clear();

       name = "numUcastBytesDequeued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastBytesDequeued_);
       name.clear();

       name = "numUcastPacketsOverflow" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastPacketsOverFlow_);
       name.clear();

       name = "numUcastBytesOverflow" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastBytesOverFlow_);
       name.clear();

       name = "numUcastPacketsTooLarge" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastPacketsTooLarge_);
       name.clear();

       name = "numUcastBytesTooLarge" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numUcastBytesTooLarge_);
       name.clear();



       name = "numMcastPacketsEnqueued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastPacketsEnqueued_);
       name.clear();

       name = "numMcastBytesEnqueued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastBytesEnqueued_);
       name.clear();

       name = "numMcastPacketsDequeued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastPacketsDequeued_);
       name.clear();

       name = "numMcastBytesDequeued" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastBytesDequeued_);
       name.clear();

       name = "numMcastPacketsOverflow" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastPacketsOverFlow_);
       name.clear();

       name = "numMcastBytesOverflow" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastBytesOverFlow_);
       name.clear();

       name = "numMcastPacketsTooLarge" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastPacketsTooLarge_);
       name.clear();

       name = "numMcastBytesTooLarge" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numMcastBytesTooLarge_);
       name.clear();

       name = "numHighWaterMark" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numHighWaterMark_);
       name.clear();

       name = "numHighWaterMax" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numHighWaterMax_);
       name.clear();

       name = "numEntrySizeMax" + ss.str();
       pOwner->registerStatistic(name.c_str(),&numEntrySizeMax_);
       name.clear();

    }

  };

 /**
 *
 * @brief class used to define the mac downstream packet queue
 *
 */
  class DownstreamQueue
  {
  private:
    EMANE::NEMId id_;

    AccessCategory categories_[NUM_ACCESS_CATEGORIES_MAX];

    ACE_UINT8 numCategories_;

    ACE_Thread_Mutex mutex_;

    ACE_Condition < ACE_Thread_Mutex > cond_;

    volatile bool canceled_;

    EMANE::StatisticServiceProvider * pOwner_;

    bool empty (ACE_UINT8 &);

  public:

    DownstreamQueue (EMANE::NEMId id, EMANE::StatisticServiceProvider *pOwner);

    ~DownstreamQueue ();

    void maxsize (size_t, ACE_UINT8);

    size_t total ();

    size_t size (ACE_UINT8);

    void categories (ACE_UINT8);

    DownstreamQueueEntry dequeue ();

    bool enqueue (DownstreamQueueEntry & entry);

    void cancel ();

  };
}
#endif //DOWNSTREAMQUEUE_HEADER_
