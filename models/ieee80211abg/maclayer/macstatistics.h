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

#ifndef MACSTATISTICS_HEADER_
#define MACSTATISTICS_HEADER_

#include "macconfig.h"
#include "emane/emaneplatformserviceprovider.h"
#include "emane/emanestatisticunsignedinteger32.h"

#include <map>
#include <vector>

#include <ace/Basic_Types.h>
#include <ace/Thread_Mutex.h>
#include <ace/Time_Value.h>

namespace IEEE80211ABG
{

/**
*
* @brief class used to define the mac layer statistic items
*
*/
  class MACStatistics
  {
  private:
    mutable ACE_Thread_Mutex mutex_;

    EMANE::NEMId id_;

    EMANE::StatisticServiceProvider * pStatisticServiceProvider_;

 /**
  *
  * @brief ieee80211abg mac statistic items.
  *
  */
    EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToRegistrationIdMismatch_;
    EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToLackOfRxCtrlInfo_;

    EMANE::StatisticUnsignedInteger32 numDownstreamUcastDataDiscardDueToRetries_;
    EMANE::StatisticUnsignedInteger32 numDownstreamUcastRtsCtsDataDiscardDueToRetries_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataDiscardDueToNotForThisNEM_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataDiscardDueToDuplicates_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataDiscardDueToDuplicates_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataDiscardDueToSinr_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataDiscardDueToSinr_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataDiscardDueToClobberRxDuringTx_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataDiscardDueToClobberRxDuringTx_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataDiscardDueToClobberRxHiddenBusy_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataDiscardDueToClobberRxHiddenBusy_;

    EMANE::StatisticUnsignedInteger32 numDownstreamUcastDataDiscardDueToTxop_;
    EMANE::StatisticUnsignedInteger32 numDownstreamMcastDataDiscardDueToTxop_;

    EMANE::StatisticUnsignedInteger32 numDownstreamDiscardDueToFlowControl_;
    EMANE::StatisticUnsignedInteger32 numDownstreamDiscardDueToQueueRejection_;

    EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToQueueRejection_;
    EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToUnknownType_;

    EMANE::StatisticUnsignedInteger32 numDownstreamUcastDataSentToPhy_;
    EMANE::StatisticUnsignedInteger32 numDownstreamUcastRtsCtsDataSentToPhy_;
    EMANE::StatisticUnsignedInteger32 numDownstreamUcastCtsSentToPhy_;
    EMANE::StatisticUnsignedInteger32 numDownstreamMcastDataSentToPhy_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataRxFromPhy_;
    EMANE::StatisticUnsignedInteger32 numUpstreamUcastRtsCtsDataRxFromPhy_;
    EMANE::StatisticUnsignedInteger32 numUpstreamUcastCtsRxFromPhy_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataRxFromPhy_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataSentToTransport_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataSentToTransport_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataNoiseHiddenRx_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataNoiseHiddenRx_;

    EMANE::StatisticUnsignedInteger32 numUpstreamUcastDataNoiseRxCommon_;
    EMANE::StatisticUnsignedInteger32 numUpstreamMcastDataNoiseRxCommon_;

    EMANE::StatisticUnsignedInteger32 numOneHopNbrHighWaterMark_;
    EMANE::StatisticUnsignedInteger32 numTwoHopNbrHighWaterMark_;

    EMANE::StatisticUnsignedInteger32 numRxOneHopNbrListEvents_;
    EMANE::StatisticUnsignedInteger32 numRxOneHopNbrListInvalidEvents_;
    EMANE::StatisticUnsignedInteger32 numTxOneHopNbrListEvents_;


    /*
     * Statistic timers
     */
  public:
    MACStatistics (EMANE::NEMId, EMANE::StatisticServiceProvider * pStatisticServiceProvider);

    ~MACStatistics ();

    // discards
    void incrementUpstreamDiscardDueToRegistrationIdMismatch ();
    void incrementUpstreamDiscardDueToLackOfRxCtrlInfo ();

    void incrementDownstreamUnicastDataDiscardDueToRetries ();
    void incrementDownstreamUnicastRtsCtsDataDiscardDueToRetries ();

    void incrementDownstreamUnicastDataDiscardDueToTxop ();
    void incrementDownstreamMulticastDataDiscardDueToTxop ();

    void incrementDownstreamDiscardDueToFlowControl ();
    void incrementDownstreamDiscardDueToQueueRejection ();

    void incrementUpstreamDiscardDueToQueueRejection ();
    void incrementUpstreamDiscardDueToUnknownType ();

    void incrementUpstreamUnicastDataDiscardDueToNotForThisNEM ();

    void incrementUpstreamUnicastDataDiscardDueToDuplicates ();
    void incrementUpstreamMulticastDataDiscardDueToDuplicates ();

    void incrementUpstreamUnicastDataDiscardDueToSinr ();
    void incrementUpstreamMulticastDataDiscardDueToSinr ();

    void incrementUpstreamUnicastDataDiscardDueToClobberRxDuringTx ();
    void incrementUpstreamMulticastDataDiscardDueToClobberRxDuringTx ();

    void incrementUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy ();
    void incrementUpstreamMulticastDataDiscardDueToClobberRxHiddenBusy ();

    void incrementUpstreamMulticastNoiseHiddenRx ();
    void incrementUpstreamUnicastNoiseHiddenRx ();

    void incrementUpstreamMulticastNoiseRxCommon ();
    void incrementUpstreamUnicastNoiseRxCommon ();

    void incrementUpstreamUnicastDataSentToTransport ();
    void incrementUpstreamMulticastDataSentToTransport ();

    void incrementDownstreamUnicastDataSentToPhy ();
    void incrementDownstreamUnicastRtsCtsDataSentToPhy ();
    void incrementDownstreamUnicastCtsSentToPhy ();
    void incrementDownstreamMulticastDataSentToPhy ();

    void incrementUpstreamUnicastDataRxFromPhy ();
    void incrementUpstreamUnicastRtsCtsDataRxFromPhy ();
    void incrementUpstreamUnicastCtsRxFromPhy ();
    void incrementUpstreamMulticastDataRxFromPhy ();

    void updateOneHopNbrHighWaterMark (size_t num);
    void updateTwoHopNbrHighWaterMark (size_t num);

    // events
    void incrementRxOneHopNbrListEventCount ();
    void incrementRxOneHopNbrListInvalidEventCount ();
    void incrementTxOneHopNbrListEventCount ();
  };
}
#endif                          //MACSTATISTICS_HEADER_
