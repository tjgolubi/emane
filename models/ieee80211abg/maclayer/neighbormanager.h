/*
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef IEEE80211ABG_NEIGHBORMANAGER_HEADER_
#define IEEE80211ABG_NEIGHBORMANAGER_HEADER_


#include "emane/emanetypes.h"
#include "emane/emaneplatformserviceprovider.h"
#include "neighborentry.h"
#include "neighbor2hopentry.h"
#include "onehopnbrlistevent.h"

#include <ace/Basic_Types.h>
#include <ace/Time_Value.h>

#include <map>
#include <list>


namespace IEEE80211ABG
{
  class IEEE80211abgMACLayer;

/**
 * @class  NeighborManager
 *
 * @brief Defines the ieee 80211 abg 1 and 2 hop neighbor manager
 *
 */
  class NeighborManager
  {
  public:
    NeighborManager (EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService, IEEE80211abgMACLayer *pMgr);

    ~NeighborManager ();

    void updateDataChannelActivity (EMANE::NEMId src, ACE_UINT8 type, double dRxPowerMilliWatts, 
                                    const ACE_Time_Value & tvTime, const ACE_Time_Value & tvDuration);

    void updateCtrlChannelActivity (EMANE::NEMId src, EMANE::NEMId origin, ACE_UINT8 type, double dRxPowerMilliWatts,
                                    const ACE_Time_Value & tvTime, const ACE_Time_Value & tvDuration);

    void processEvent (const OneHopNbrListEvent &event);

    void resetStatistics (const ACE_Time_Value &tvDeltaT);

    void setNeighborTimeout(const ACE_Time_Value &tv);

    float getNumberOfEstimatedOneHopNeighbors() const;

    float getNumberOfEstimatedTwoHopNeighbors() const;

    float getHiddenChannelActivity(EMANE::NEMId src) const;

    float getNumberOfEstimatedCommonNeighbors(EMANE::NEMId src) const;

    float getNumberOfEstimatedHiddenNeighbors(EMANE::NEMId src) const;

    float getLocalNodeTx () const;

    size_t getTotalActiveOneHopNeighbors () const;

    ACE_Time_Value getTotalOneHopBandWidthUtilization() const;

    ACE_Time_Value getTotalTwoHopBandWidthUtilization() const;

    ACE_Time_Value getAverageMessageDuration() const;

    ACE_Time_Value getAllBandWidthUtilization(EMANE::NEMId src) const;

    double getAverageRxPowerPerMessageMilliWatts() const;

    double getAverageRxPowerPerMessageHiddenNodesMilliWatts() const;

    double getAverageRxPowerPerMessageCommonNodesMilliWatts() const;
    
    double getRandomRxPowerCommonNodesMilliWatts(EMANE::NEMId src) const;

    double getRandomRxPowerHiddenNodesMilliWatts(EMANE::NEMId src) const;

    ACE_Time_Value getLastOneHopNbrListTxTime() const;

  private:
    typedef std::map < EMANE::NEMId, NeighborEntry > NeighborEntryMap;
    typedef NeighborEntryMap::iterator NeighborEntryMapIter;
    typedef NeighborEntryMap::const_iterator NeighborEntryMapConstIter;

    typedef std::pair< NeighborEntryMapIter, bool> NeighborEntryInsertResult;

    typedef std::map < EMANE::NEMId, Neighbor2HopEntry > Neighbor2HopEntryMap;
    typedef Neighbor2HopEntryMap::iterator Neighbor2HopEntryMapIter;
    typedef Neighbor2HopEntryMap::const_iterator Neighbor2HopEntryMapConstIter;

    typedef std::pair< Neighbor2HopEntryMapIter, bool> Neighbor2HopEntryInsertResult;

    typedef std::map < EMANE::NEMId, NbrSet > NbrSetMap;
    typedef NbrSetMap::iterator NbrSetMapIter;
    typedef NbrSetMap::const_iterator NbrSetMapConstIter;

    typedef std::map <EMANE::NEMId, ACE_Time_Value> NbrUtilizationMap;
    typedef NbrUtilizationMap::iterator NbrUtilizationMapIter;
    typedef NbrUtilizationMap::const_iterator NbrUtilizationMapConstIter;

    typedef std::pair <float, float> ProbabilityPair;

    typedef std::map <EMANE::NEMId, ProbabilityPair> ProbabilityPairMap;
    typedef ProbabilityPairMap::iterator ProbabilityPairMapIter;
    typedef ProbabilityPairMap::reverse_iterator ProbabilityPairMapRevIter;
    typedef ProbabilityPairMap::const_iterator ProbabilityPairMapConstIter;

    typedef std::map <EMANE::NEMId, ProbabilityPairMap> ProbabilityPairMapMap;
    typedef ProbabilityPairMapMap::iterator ProbabilityPairMapMapIter;
    typedef ProbabilityPairMapMap::const_iterator ProbabilityPairMapMapConstIter;

    typedef std::map <EMANE::NEMId, double> RxPowerMap;
    typedef RxPowerMap::iterator RxPowerMapIter;
    typedef RxPowerMap::const_iterator RxPowerMapConstIter;


    EMANE::NEMId id_;

    EMANE::PlatformServiceProvider * pPlatformService_;

    IEEE80211abgMACLayer *pMgr_;

    NeighborEntryMap oneHopNbrMap_;

    Neighbor2HopEntryMap twoHopNbrMap_;

    NbrSetMap cachedOneHopNbrSetMap_;

    ACE_Time_Value tvTotalOneHopBandWidthUtilization_;

    ACE_Time_Value tvTotalTwoHopBandWidthUtilization_;

    size_t totalOneHopNumPackets_;

    size_t totalTwoHopNumPackets_;

    size_t numTotalActiveOneHopNeighbors_;

    ACE_Time_Value tvAverageMessageDuration_;

    double dAverageRxPowerPerMessageMilliWatts_;

    double dTotalRxPowerMilliWatts_;

    ACE_Time_Value tvAverageUtilizationPerOneHopNeighbor_;

    ACE_Time_Value tvAverageUtilizationPerTwoHopNeighbor_;

    float fEstimatedNumOneHopNeighbors_;

    float fEstimatedNumTwoHopNeighbors_;
 
    float fLocalNodeTx_;

    size_t sumCommonPackets_;

    size_t sumHiddenPackets_;

    double dCommonRxPowerMilliWatts_;

    double dHiddenRxPowerMilliWatts_;

    ACE_Time_Value tvBandWidthUtilizationThisNem_;

    ACE_Time_Value tvNeighborTimeout_;

    ACE_Time_Value tvLastOneHopNbrListTxTime_;

    NbrUtilizationMap oneHopUtilizationMap_;

    NbrUtilizationMap twoHopUtilizationMap_;

    ProbabilityPairMapMap commonProbabilityMapMap_;

    ProbabilityPairMapMap hiddenProbabilityMapMap_;

    RxPowerMap commonNbrAvgRxPowerMwMap_;

    RxPowerMap hiddenNbrAvgRxPowerMwMap_;

    void sendOneHopNbrListEvent_priv();

    bool flushOneHopNeighbors_priv(const ACE_Time_Value & tvCurrentTime, const ACE_Time_Value & tvTimeout);

    bool flushTwoHopNeighbors_priv(const ACE_Time_Value & tvCurrentTime, const ACE_Time_Value & tvTimeout);

    void resetCounters_priv();

    void calculateBwUtilization_priv(const ACE_Time_Value & tvDeltaT);

    NeighborEntryInsertResult addOneHopNeighbor_priv(EMANE::NEMId src);

    Neighbor2HopEntryInsertResult addTwoHopNeighbor_priv(EMANE::NEMId src);

    float getA_priv(const ACE_Time_Value &tvUtilization, const ACE_Time_Value & tvAverageUtilization) const;

    float getC_priv(const ACE_Time_Value & tvBandWidthUtilization, const ACE_Time_Value &tvDeltaT) const;

    float getH_priv(const ACE_Time_Value &tvUtilization, const ACE_Time_Value &tvDeltaT) const;

    float getChannelActivity_priv(const ACE_Time_Value &tvUtilization, const ACE_Time_Value &tvDeltaT) const;

    void setCommonAndHiddenProbability_priv();

    ProbabilityPairMap setProbability_priv (EMANE::NEMId id, const NbrUtilizationMap & map, 
                                            ACE_Time_Value tv, const NbrSet & nbrSet, const char * str) const;

    double getRandomRxPowerMilliWatts_priv(EMANE::NEMId src, float R1, 
                                          const ProbabilityPairMapMap & pmap, 
                                          const RxPowerMap & rmap) const;

  };
}
#endif                          //IEEE80211ABG_NEIGHBORMANAGER_HEADER_
