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


#include "macstatistics.h"

/**
 *
 * @brief constructor
 *
 */
IEEE80211ABG::MACStatistics::MACStatistics (EMANE::NEMId id,EMANE::StatisticServiceProvider *pStatisticServiceProvider):
  id_ (id),
  pStatisticServiceProvider_(pStatisticServiceProvider)
{
  pStatisticServiceProvider_->registerStatistic("numUpstreamDiscardDueToRegistrationIdMismatch",
                                                &numUpstreamDiscardDueToRegistrationIdMismatch_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamDiscardDueToLackOfRxCtrlInfo", 
                                                &numUpstreamDiscardDueToLackOfRxCtrlInfo_);


  pStatisticServiceProvider_->registerStatistic("numDownstreamUcastDataDiscardDueToRetries", 
                                                &numDownstreamUcastDataDiscardDueToRetries_);

  pStatisticServiceProvider_->registerStatistic("numDownstreamUcastRtsCtsDataDiscardDueToRetries", 
                                                &numDownstreamUcastRtsCtsDataDiscardDueToRetries_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataDiscardDueToNotForThisNEM", 
                                                &numUpstreamUcastDataDiscardDueToNotForThisNEM_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataDiscardDueToDuplicates", 
                                                &numUpstreamUcastDataDiscardDueToDuplicates_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataDiscardDueToDuplicates", 
                                                &numUpstreamMcastDataDiscardDueToDuplicates_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataDiscardDueToSinr", 
                                                &numUpstreamUcastDataDiscardDueToSinr_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataDiscardDueToSinr", 
                                                &numUpstreamMcastDataDiscardDueToSinr_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataDiscardDueToClobberRxDuringTx", 
                                                &numUpstreamUcastDataDiscardDueToClobberRxDuringTx_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataDiscardDueToClobberRxDuringTx", 
                                                &numUpstreamMcastDataDiscardDueToClobberRxDuringTx_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataDiscardDueToClobberRxHiddenBusy", 
                                                &numUpstreamUcastDataDiscardDueToClobberRxHiddenBusy_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataDiscardDueToClobberRxHiddenBusy", 
                                                &numUpstreamMcastDataDiscardDueToClobberRxHiddenBusy_);


  pStatisticServiceProvider_->registerStatistic("numDownstreamUcastDataDiscardDueToTxop", 
                                                &numDownstreamUcastDataDiscardDueToTxop_);

  pStatisticServiceProvider_->registerStatistic("numDownstreamMcastDataDiscardDueToTxop", 
                                                &numDownstreamMcastDataDiscardDueToTxop_);


  pStatisticServiceProvider_->registerStatistic("numDownstreamDiscardDueToFlowControl", 
                                                &numDownstreamDiscardDueToFlowControl_);

  pStatisticServiceProvider_->registerStatistic("numDownstreamDiscardDueToQueueRejection", 
                                                &numDownstreamDiscardDueToQueueRejection_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamDiscardDueToQueueRejection", 
                                                &numUpstreamDiscardDueToQueueRejection_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamDiscardDueToUnknownType", 
                                                &numUpstreamDiscardDueToUnknownType_);


  pStatisticServiceProvider_->registerStatistic("numDownstreamUcastDataSentToPhy", 
                                                &numDownstreamUcastDataSentToPhy_);

  pStatisticServiceProvider_->registerStatistic("numDownstreamUcastRtsCtsDataSentToPhy", 
                                                &numDownstreamUcastRtsCtsDataSentToPhy_);

  pStatisticServiceProvider_->registerStatistic("numDownstreamUcastCtsSentToPhy",  
                                                &numDownstreamUcastCtsSentToPhy_);

  pStatisticServiceProvider_->registerStatistic("numDownstreamMcastDataSentToPhy", 
                                                &numDownstreamMcastDataSentToPhy_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataNoiseHiddenRx", 
                                                &numUpstreamUcastDataNoiseHiddenRx_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataNoiseHiddenRx", 
                                                &numUpstreamMcastDataNoiseHiddenRx_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataNoiseRxCommon", 
                                                &numUpstreamUcastDataNoiseRxCommon_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataNoiseRxCommon", 
                                                &numUpstreamMcastDataNoiseRxCommon_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataRxFromPhy", 
                                                &numUpstreamUcastDataRxFromPhy_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataRxFromPhy", 
                                                &numUpstreamUcastDataRxFromPhy_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastRtsCtsDataRxFromPhy",  
                                                &numUpstreamUcastCtsRxFromPhy_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataRxFromPhy", 
                                                &numUpstreamMcastDataRxFromPhy_);


  pStatisticServiceProvider_->registerStatistic("numUpstreamUcastDataSentToTransport", 
                                                &numUpstreamUcastDataSentToTransport_);

  pStatisticServiceProvider_->registerStatistic("numUpstreamMcastDataSentToTransport", 
                                                &numUpstreamMcastDataSentToTransport_);

  pStatisticServiceProvider_->registerStatistic ("numOneHopNbrHighWaterMark", 
                                                 &numOneHopNbrHighWaterMark_);

  pStatisticServiceProvider_->registerStatistic ("numTwoHopNbrHighWaterMark",
                                                 &numTwoHopNbrHighWaterMark_);

  pStatisticServiceProvider_->registerStatistic ("numRxOneHopNbrListEvents",
                                                 &numRxOneHopNbrListEvents_);

  pStatisticServiceProvider_->registerStatistic ("numRxOneHopNbrListInvalidEvents",
                                                 &numRxOneHopNbrListInvalidEvents_);

  pStatisticServiceProvider_->registerStatistic ("numTxOneHopNbrListEvents",
                                                 &numTxOneHopNbrListEvents_);
}


/**
*
* @brief destructor
*
*/
IEEE80211ABG::MACStatistics::~MACStatistics ()
{
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamDiscardDueToRegistrationIdMismatch");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamDiscardDueToLackOfRxCtrlInfo");

  pStatisticServiceProvider_->unregisterStatistic("numDownstreamUcastDataDiscardDueToRetries");
  pStatisticServiceProvider_->unregisterStatistic("numDownstreamUcastRtsCtsDataDiscardDueToRetries");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataDiscardDueToNotForThisNEM");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataDiscardDueToDuplicates");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataDiscardDueToDuplicates");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataDiscardDueToSinr");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataDiscardDueToSinr");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataDiscardDueToClobberRxDuringTx");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataDiscardDueToClobberRxDuringTx");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataDiscardDueToClobberRxHiddenBusy");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataDiscardDueToClobberRxHiddenBusy");

  pStatisticServiceProvider_->unregisterStatistic("numDownstreamUcastDataDiscardDueToTxop");
  pStatisticServiceProvider_->unregisterStatistic("numDownstreamMcastDataDiscardDueToTxop");

  pStatisticServiceProvider_->unregisterStatistic("numDownstreamDiscardDueToFlowControl");
  pStatisticServiceProvider_->unregisterStatistic("numDownstreamDiscardDueToQueueRejection");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamDiscardDueToQueueRejection");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamDiscardDueToUnknownType");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataNoiseHiddenRx");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataNoiseHiddenRx");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataNoiseRxCommon");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataNoiseRxCommon");

  pStatisticServiceProvider_->unregisterStatistic("numDownstreamUcastDataSentToPhy");
  pStatisticServiceProvider_->unregisterStatistic("numDownstreamUcastRtsCtsDataSentToPhy");
  pStatisticServiceProvider_->unregisterStatistic("numDownstreamUcastCtsSentToPhy");
  pStatisticServiceProvider_->unregisterStatistic("numDownstreamMcastDataSentToPhy");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataRxFromPhy");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastRtsCtsDataRxFromPhy");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastCtsRxFromPhy");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataRxFromPhy");

  pStatisticServiceProvider_->unregisterStatistic("numUpstreamUcastDataSentToTransport");
  pStatisticServiceProvider_->unregisterStatistic("numUpstreamMcastDataSentToTransport");

  pStatisticServiceProvider_->unregisterStatistic("numOneHopNbrHighWaterMark");
  pStatisticServiceProvider_->unregisterStatistic("numTwoHopNbrHighWaterMark");

  pStatisticServiceProvider_->unregisterStatistic ("numRxOneHopNbrListEvents");
  pStatisticServiceProvider_->unregisterStatistic ("numRxOneHopNbrListInvalidEvents");
  pStatisticServiceProvider_->unregisterStatistic ("numTxOneHopNbrListEvents");
}


/**
*
* @brief increment discard due to registration id mismatch
*
*/
void 
IEEE80211ABG::MACStatistics::incrementUpstreamDiscardDueToRegistrationIdMismatch ()
{
  // bump counter
  ++numUpstreamDiscardDueToRegistrationIdMismatch_;
}


/**
*
* @brief increment discard due to lack of rx ctrl info
*
*/
void 
IEEE80211ABG::MACStatistics::incrementUpstreamDiscardDueToLackOfRxCtrlInfo ()
{
  // bump counter
  ++numUpstreamDiscardDueToLackOfRxCtrlInfo_;
}


/**
*
* @brief increment unicast data sent to phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamUnicastDataSentToPhy ()
{
  // bump counter
  ++numDownstreamUcastDataSentToPhy_;
}


/**
*
* @brief increment unicast rts/ctsdata sent to phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamUnicastRtsCtsDataSentToPhy ()
{
  // bump counter
  ++numDownstreamUcastRtsCtsDataSentToPhy_;
}


/**
*
* @brief increment unicast cts sent to phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamUnicastCtsSentToPhy ()
{
  // bump counter
  ++numDownstreamUcastCtsSentToPhy_;
}


/**
*
* @brief increment multicast data sent to phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamMulticastDataSentToPhy ()
{
  // bump counter
  ++numDownstreamMcastDataSentToPhy_;
}


/**
*
* @brief increment unicast data recv from phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataRxFromPhy ()
{
  // bump counter
  ++numUpstreamUcastDataRxFromPhy_;
}


/**
*
* @brief increment unicast rts/ctsdata recv from phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastRtsCtsDataRxFromPhy ()
{
  // bump counter
  ++numUpstreamUcastRtsCtsDataRxFromPhy_;
}


/**
*
* @brief increment unicast cts recv from phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastCtsRxFromPhy ()
{
  // bump counter
  ++numUpstreamUcastCtsRxFromPhy_;
}


/**
*
* @brief increment multicast data recv from phy
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastDataRxFromPhy ()
{
  // bump counter
  ++numUpstreamMcastDataRxFromPhy_;
}


/**
*
* @brief increment unicast data sent to net
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataSentToTransport ()
{
  // bump counter
  ++numUpstreamUcastDataSentToTransport_;
}


/**
*
* @brief increment multicast data sent to net
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastDataSentToTransport ()
{
  // bump counter
  ++numUpstreamMcastDataSentToTransport_;
}


/**
*
* @brief increment unicast data discard due to exhausted retries
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamUnicastDataDiscardDueToRetries ()
{
  // bump counter
  ++numDownstreamUcastDataDiscardDueToRetries_;
}


/**
*
* @brief increment multicast data discard due to exhausted retries
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamUnicastRtsCtsDataDiscardDueToRetries ()
{
  // bump counter
  ++numDownstreamUcastRtsCtsDataDiscardDueToRetries_;
}


/**
*
* @brief increment unicast data discard due to not for this NEM
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToNotForThisNEM ()
{
  // bump counter
  ++numUpstreamUcastDataDiscardDueToNotForThisNEM_;
}


/**
*
* @brief increment unicast data discard due to duplicates
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToDuplicates ()
{
  // bump counter
  ++numUpstreamUcastDataDiscardDueToDuplicates_;
}


/**
*
* @brief increment multicast data discard due to duplicates
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastDataDiscardDueToDuplicates ()
{
  // bump counter
  ++numUpstreamMcastDataDiscardDueToDuplicates_;
}


/**
*
* @brief increment unicast data discard due to sinr
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToSinr ()
{
  // bump counter
  ++numUpstreamUcastDataDiscardDueToSinr_;
}


/**
*
* @brief increment multicast data discard due to sinr
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastDataDiscardDueToSinr ()
{
  // bump counter
  ++numUpstreamMcastDataDiscardDueToSinr_;
}


/**
*
* @brief increment unicast data discard due to collision rx during tx
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToClobberRxDuringTx ()
{
  // bump counter
  ++numUpstreamUcastDataDiscardDueToClobberRxDuringTx_;
}


/**
*
* @brief increment multicast data discard due to collision rx during tx
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastDataDiscardDueToClobberRxDuringTx ()
{
  // bump counter
  ++numUpstreamMcastDataDiscardDueToClobberRxDuringTx_;
}


/**
*
* @brief increment unicast data discard due to collision rx busy hidden
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy()
{
  // bump counter
  ++numUpstreamUcastDataDiscardDueToClobberRxHiddenBusy_;
}


/**
*
* @brief increment multicast data discard due to collision rx busy hidden
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastDataDiscardDueToClobberRxHiddenBusy()
{
  // bump counter
  ++numUpstreamMcastDataDiscardDueToClobberRxHiddenBusy_;
}



/**
*
* @brief increment unicast data discard due to txop expired
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamUnicastDataDiscardDueToTxop ()
{
  // bump counter
  ++numDownstreamUcastDataDiscardDueToTxop_;
}


/**
*
* @brief increment unicast data discard due to flow control
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamDiscardDueToFlowControl ()
{
  // bump counter
  ++numDownstreamDiscardDueToFlowControl_;
}

/**
*
* @brief increment discard due to queue rejection
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamDiscardDueToQueueRejection ()
{
  // bump counter
  ++numDownstreamDiscardDueToQueueRejection_;
}


/**
*
* @brief increment discard due to queue rejection
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamDiscardDueToQueueRejection ()
{
  // bump counter
  ++numUpstreamDiscardDueToQueueRejection_;
}


/**
*
* @brief increment discard due to unknown type
*
*/
void
IEEE80211ABG::MACStatistics::incrementUpstreamDiscardDueToUnknownType ()
{
  // bump counter
  ++numUpstreamDiscardDueToUnknownType_;
}



/**
*
* @brief increment unicast data discard due to txop expired
*
*/
void
IEEE80211ABG::MACStatistics::incrementDownstreamMulticastDataDiscardDueToTxop ()
{
  // bump counter
  ++numDownstreamMcastDataDiscardDueToTxop_;
}


/**
*
* @brief increment unicastcast data collision due to hidden rx
*
*/
void 
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastNoiseHiddenRx ()
{
  // bump counter
  ++numUpstreamUcastDataNoiseHiddenRx_;
}



/**
*
* @brief increment multicastcast data collision due to hidden rx
*
*/
void 
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastNoiseHiddenRx ()
{
  // bump counter
  ++numUpstreamMcastDataNoiseHiddenRx_;
}



/**
*
* @brief increment unicastcast data collision due to rx common
*
*/
void 
IEEE80211ABG::MACStatistics::incrementUpstreamMulticastNoiseRxCommon ()
{
  // bump counter
  ++numUpstreamMcastDataNoiseRxCommon_;
}


/**
*
* @brief increment multicastcast data collision due to rx common
*
*/
void 
IEEE80211ABG::MACStatistics::incrementUpstreamUnicastNoiseRxCommon ()
{
  // bump counter
  ++numUpstreamUcastDataNoiseRxCommon_;
}



/**
*
* @brief set the one hop nbr high water mark
*
*/
void 
IEEE80211ABG::MACStatistics::updateOneHopNbrHighWaterMark (size_t num)
{
   if(numOneHopNbrHighWaterMark_ < num)
    {
       numOneHopNbrHighWaterMark_ = num;
    }
}




/**
*
* @brief set the two hop nbr high water mark
*
*/
void 
IEEE80211ABG::MACStatistics::updateTwoHopNbrHighWaterMark (size_t num)
{
   if(numTwoHopNbrHighWaterMark_ < num)
    {
       numTwoHopNbrHighWaterMark_ = num;
    }
}


/**
*
* @brief increment number rx one hop nbr list events
*
*/
void 
IEEE80211ABG::MACStatistics::incrementRxOneHopNbrListEventCount ()
{
  // bump counter
  ++numRxOneHopNbrListEvents_;
}


/**
*
* @brief increment number rx one hop nbr list invalid events
*
*/
void 
IEEE80211ABG::MACStatistics::incrementRxOneHopNbrListInvalidEventCount ()
{
  // bump counter
  ++numRxOneHopNbrListInvalidEvents_;
}



/**
*
* @brief increment number tx one hop nbr list events
*
*/
void 
IEEE80211ABG::MACStatistics::incrementTxOneHopNbrListEventCount ()
{
  // bump counter
  ++numTxOneHopNbrListEvents_;
}

