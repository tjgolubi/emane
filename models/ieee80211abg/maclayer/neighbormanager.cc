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

#include "neighbormanager.h"
#include "ieee80211abgmaclayer.h"
#include "utils.h"
#include "msgtypes.h"

#include <ace/OS_NS_time.h>
#include <sstream>

const char * MODULE = "NeighborManager";


IEEE80211ABG::NeighborManager::NeighborManager (EMANE::NEMId id, 
                                                EMANE::PlatformServiceProvider * pPlatformService, 
                                                IEEE80211abgMACLayer *pMgr):
  id_ (id), 
  pPlatformService_ (pPlatformService),
  pMgr_(pMgr),
  tvNeighborTimeout_(ACE_Time_Value::zero),
  tvLastOneHopNbrListTxTime_(ACE_Time_Value::zero)
{
   // reset counters
   resetCounters_priv();
}



IEEE80211ABG::NeighborManager::~NeighborManager ()
{ }



float 
IEEE80211ABG::NeighborManager::getHiddenChannelActivity(EMANE::NEMId src) const
{
  // find the src in the one hop nbr(s)
  const NeighborEntryMapConstIter nbrEntry = oneHopNbrMap_.find(src);

  float result = 0.0;

  // src found
  if(nbrEntry != oneHopNbrMap_.end())
   {
     // set result
     result = nbrEntry->second.getHiddenChannelActivity();
   }

  // return result
  return result;
}



float 
IEEE80211ABG::NeighborManager::getNumberOfEstimatedCommonNeighbors(EMANE::NEMId src) const
{
  // find the src
  const NeighborEntryMapConstIter nbrEntry = oneHopNbrMap_.find(src);

  float result = 0.0;

  // src found
  if(nbrEntry != oneHopNbrMap_.end())
   {
     // set result
     result = nbrEntry->second.getEstimatedNumCommonNeighbors();
   }

  // return result
  return result;
}


float 
IEEE80211ABG::NeighborManager::getNumberOfEstimatedHiddenNeighbors(EMANE::NEMId src) const
{
  // find the src
  const NeighborEntryMapConstIter nbrEntry = oneHopNbrMap_.find(src);

  float result = 0.0;

  // src found
  if(nbrEntry != oneHopNbrMap_.end())
   {
     // set result
     result = fEstimatedNumOneHopNeighbors_ - nbrEntry->second.getEstimatedNumCommonNeighbors();
   }

  // min allowed is 0
  if(result < 0.0)
   {
     // reset result
     result = 0.0;
   }

  // return result
  return result; 
}



ACE_Time_Value 
IEEE80211ABG::NeighborManager::getAllBandWidthUtilization(EMANE::NEMId src) const
{
  // find the src
  const NeighborEntryMapConstIter nbrEntry = oneHopNbrMap_.find(src);

  // src found
  if(nbrEntry != oneHopNbrMap_.end())
   {
     // return result
     return nbrEntry->second.getBandwidtUtilization(MSG_TYPE_MASK_ALL_DATA);
   }
  else
   {
     // return zero
     return ACE_Time_Value::zero;
   }
}



float
IEEE80211ABG::NeighborManager::getNumberOfEstimatedOneHopNeighbors() const
{
  return fEstimatedNumOneHopNeighbors_;
}



float
IEEE80211ABG::NeighborManager::getNumberOfEstimatedTwoHopNeighbors() const
{
  return fEstimatedNumTwoHopNeighbors_;
}



ACE_Time_Value 
IEEE80211ABG::NeighborManager::getTotalOneHopBandWidthUtilization() const
{
  return tvTotalOneHopBandWidthUtilization_;
}



ACE_Time_Value 
IEEE80211ABG::NeighborManager::getTotalTwoHopBandWidthUtilization() const
{
  return tvTotalTwoHopBandWidthUtilization_;
}



ACE_Time_Value 
IEEE80211ABG::NeighborManager::getAverageMessageDuration() const
{
  return tvAverageMessageDuration_;        
}




float
IEEE80211ABG::NeighborManager::getLocalNodeTx () const
{
   return fLocalNodeTx_;
}



size_t
IEEE80211ABG::NeighborManager::getTotalActiveOneHopNeighbors () const
{
   return numTotalActiveOneHopNeighbors_;
}



double 
IEEE80211ABG::NeighborManager::getAverageRxPowerPerMessageMilliWatts() const
{
  return dAverageRxPowerPerMessageMilliWatts_;
}



double 
IEEE80211ABG::NeighborManager::getAverageRxPowerPerMessageHiddenNodesMilliWatts() const
{
  if(sumHiddenPackets_ > 0)
   {
     return dHiddenRxPowerMilliWatts_ / sumHiddenPackets_;
   }
  else
   {
     return 0.0;
   }
}



double 
IEEE80211ABG::NeighborManager::getAverageRxPowerPerMessageCommonNodesMilliWatts() const
{
  if(sumCommonPackets_ > 0)
   {
     return dCommonRxPowerMilliWatts_ / sumCommonPackets_;
   }
  else
   {
     return 0.0;
   }
}



double 
IEEE80211ABG::NeighborManager::getRandomRxPowerCommonNodesMilliWatts(EMANE::NEMId src) const
{
  // get random probability
  const float R1 = pPlatformService_->getRandomProbability();

  // get result using common info
  const float result = getRandomRxPowerMilliWatts_priv(src, R1, commonProbabilityMapMap_, commonNbrAvgRxPowerMwMap_);

#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, R1 %5.4f, result %5.4f", 
                          id_, MODULE, __func__, src, R1, result);
#endif

  // return result
  return result;
}



double 
IEEE80211ABG::NeighborManager::getRandomRxPowerHiddenNodesMilliWatts(EMANE::NEMId src) const
{
  // get random probability
  const float R1 = pPlatformService_->getRandomProbability();

  // get result using hidden info
  const double dResultmW = getRandomRxPowerMilliWatts_priv(src, R1,  hiddenProbabilityMapMap_, hiddenNbrAvgRxPowerMwMap_);

#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, R1 %5.4f, result %6.4lf mW", 
                          id_, MODULE, __func__, src, R1, dResultmW);
#endif

  // return result
  return dResultmW;
}



double 
IEEE80211ABG::NeighborManager::getRandomRxPowerMilliWatts_priv(EMANE::NEMId src,
                                                               float R1,
                                                               const ProbabilityPairMapMap & pmap,
                                                               const RxPowerMap & rmap) const
{

  // result
  double dResultmW = 0.0;

  // lookup probabilty pair map
  const ProbabilityPairMapMapConstIter pmmiter = pmap.find(src);

  // entry found
  if(pmmiter != pmap.end())
   {
     // for each common probabilty entry
     for(ProbabilityPairMapConstIter piter = pmmiter->second.begin(); piter != pmmiter->second.end(); ++piter)
      {
        // search probability ranges
        if((R1 >= piter->second.first) && (R1 <= piter->second.second))
         {
           // lookup common nbr
           const RxPowerMapConstIter riter = rmap.find(piter->first);

           // common nbr found
           if(riter != rmap.end())
            {
               // set result
               dResultmW = riter->second;
            }

            // done
            break;
         }
      }
   }

#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, R1 %5.4f, result %6.4lf", 
                          id_, MODULE, __func__, src, R1, dResultmW);
#endif

  // return result
  return dResultmW;
}



ACE_Time_Value 
IEEE80211ABG::NeighborManager::getLastOneHopNbrListTxTime() const
{
  return tvLastOneHopNbrListTxTime_;
}



void 
IEEE80211ABG::NeighborManager::setNeighborTimeout(const ACE_Time_Value &tv)
{
  tvNeighborTimeout_ = tv;
}



void
IEEE80211ABG::NeighborManager::resetStatistics (const ACE_Time_Value &tvDeltaT)
{
  // current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();

  // reset counters
  resetCounters_priv();

  // nbr timeout enabled
  if(tvNeighborTimeout_ != ACE_Time_Value::zero)
   {
     // time out old one hop nbrs
     if(flushOneHopNeighbors_priv(tvCurrentTime, tvNeighborTimeout_) == true)
      {
        // nbr table changed, send event
        sendOneHopNbrListEvent_priv();
      }

     // time out old two hop nbrs
     if(flushTwoHopNeighbors_priv(tvCurrentTime, tvNeighborTimeout_) == true)
      {
        // dont care
      }
   }
  else
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: nbr timeout disabled", id_, MODULE, __func__);
#endif
   }

  // calculate bw utilization
  calculateBwUtilization_priv (tvDeltaT);

  // for each one hop nbr
  for (NeighborEntryMapIter nbrEntry = oneHopNbrMap_.begin (); nbrEntry != oneHopNbrMap_.end (); ++nbrEntry)
    {
      // reset utilization
      nbrEntry->second.resetUtilization();
    }

  // for each two hop nbr
  for (Neighbor2HopEntryMapIter nbr2Entry = twoHopNbrMap_.begin (); nbr2Entry != twoHopNbrMap_.end (); ++nbr2Entry)
    {
      // reset utilization
      nbr2Entry->second.resetUtilization();
    }
}





void
IEEE80211ABG::NeighborManager::updateDataChannelActivity (EMANE::NEMId src, 
                                                          ACE_UINT8 type,
                                                          double dRxPowerMilliWatts, 
                                                          const ACE_Time_Value & tvTime, 
                                                          const ACE_Time_Value & tvDuration)
{
  bool changed = false;

  // add/update one hop nbr
  const NeighborEntryInsertResult nbrResult = addOneHopNeighbor_priv (src);

  // new nbr
  if(nbrResult.second == true)
   {
     // one hop nbr table changed
     changed = true;
   }

  // udpate the duration, last activity time and rx power
  nbrResult.first->second.updateChannelActivity (tvDuration, type, tvTime, dRxPowerMilliWatts);

  // nbr table changed
  if(changed == true)
   {
     // send one hop nbr list event
     sendOneHopNbrListEvent_priv();
   }
}




void
IEEE80211ABG::NeighborManager::updateCtrlChannelActivity (EMANE::NEMId src, 
                                                          EMANE::NEMId origin, 
                                                          ACE_UINT8 type,
                                                          double dRxPowerMilliWatts, 
                                                          const ACE_Time_Value & tvTime, 
                                                          const ACE_Time_Value & tvDuration)
{
  bool changed = false;

  // check unicast rts/cts origin is not us
  if(origin != id_)
   {
     // rts/cts origin not found
     if(oneHopNbrMap_.find(origin) == oneHopNbrMap_.end())
      {
        // add two hop nbr
        const Neighbor2HopEntryInsertResult nbr2Result = addTwoHopNeighbor_priv(origin);

        // udpate the duration (unicast msg), last activity time
        nbr2Result.first->second.updateChannelActivity (tvDuration, tvTime);
      }
   }

  // add/update one hop nbr
  const NeighborEntryInsertResult nbrResult = addOneHopNeighbor_priv(src);

  // new nbr
  if(nbrResult.second == true)
   {
     // one hop nbr table changed
     changed = true;
   }

  // do not include the duration, just set last activity time and rx power
  nbrResult.first->second.updateChannelActivity (ACE_Time_Value::zero, type, tvTime, dRxPowerMilliWatts);

  // nbr table changed
  if(changed == true)
   {
     // send one hop nbr list event
     sendOneHopNbrListEvent_priv();
   }
}




void 
IEEE80211ABG::NeighborManager::processEvent (const OneHopNbrListEvent &event)
{
  // one hop nbr list entries
  const OneHopNbrListEvent::OneHopNbrEntry *e = event.getEntries();

  // num entries
  const size_t numEntries = event.getNumberOfEntries();

  if(numEntries == 0)
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: ignore empty one_hop_nbr_list", 
                             id_, MODULE, __func__);
#endif

     // bump num rx invalid events 
     pMgr_->getStatistics().incrementRxOneHopNbrListInvalidEventCount ();

     // drop
     return;
   }
  else
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: handle one_hop_nbr_list from %hu, with %zd entries", 
                             id_, MODULE, __func__, e[0].u16NEMId_, numEntries);
#endif

     // bump num rx events 
     pMgr_->getStatistics().incrementRxOneHopNbrListEventCount ();
   }


  // the event src
  EMANE::NEMId src = 0;

  // the entries
  NbrSet entries;

  // each entry
  for(size_t i = 0; i < numEntries; ++i)
   {
      // the sender is the first entry
      if(i == 0)
       {
         // remember src
         src = e[i].u16NEMId_;

         // src is us (event bounced back)
         if(src == id_)
          {
             // drop
             return;
          }
       }
      else
       {
         // store the entries
         entries.insert(e[i].u16NEMId_);
       }
   }

  // search for one hop nbr
  const NeighborEntryMapIter nbrEntry = oneHopNbrMap_.find(src);

  // src is an active one hop nbr
  if(nbrEntry != oneHopNbrMap_.end())
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: update one_hop_nbr_list for nbr %hu, %zd entries", 
                             id_, MODULE, __func__,
                             src, 
                             entries.size());
#endif

     // set one hop nbrs of this one hop nbr
     nbrEntry->second.setOneHopNeighbors(entries);
   }
  else
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: unknown nbr %hu, one_hop_nbr_list with %zd entries will be cached", 
                             id_, MODULE, __func__,
                             src, 
                             entries.size());
#endif
   }

   // we want to keep a cache of events in case of unknown or timed out nodes 
   // find src of this event in the one hop nbr list event cache
   const NbrSetMapIter iter = cachedOneHopNbrSetMap_.find(src);

   // not already cached
   if(iter == cachedOneHopNbrSetMap_.end())
    {
      // insert into cache
      cachedOneHopNbrSetMap_[src] = entries;

#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: added event cache one_hop_nbr_list for nbr %hu, %zd entries", 
                              id_, MODULE, __func__,
                              src,
                              entries.size());
#endif
    }
  else
    {
      // update cache
      iter->second = entries;
 
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: updated event cache one_hop_nbr_list for nbr %hu, %zd entries", 
                              id_, MODULE, __func__,
                              src, 
                              entries.size());
#endif
    }
}




void
IEEE80211ABG::NeighborManager::calculateBwUtilization_priv (const ACE_Time_Value & tvDeltaT)
{
  // internal call
  
  // each one hop nbr
  for (NeighborEntryMapIter nbrEntry = oneHopNbrMap_.begin (); nbrEntry != oneHopNbrMap_.end (); ++nbrEntry)
    {
       // get bandwidth utilization all DATA msg types
       const ACE_Time_Value tvUtilization = nbrEntry->second.getBandwidtUtilization(MSG_TYPE_MASK_ALL_DATA);

       // check utilization
       if(tvUtilization > ACE_Time_Value::zero)
        {
          // sum total one hop bandwidth utilization
          tvTotalOneHopBandWidthUtilization_ += tvUtilization;

          // store the one hop utilization
          oneHopUtilizationMap_[nbrEntry->first] = tvUtilization;

#ifdef VERY_VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: one hop nbr %hu, bw %ld:%06ld, total bw %ld:%06ld",
                                  id_, MODULE, __func__, 
                                  nbrEntry->first,
                                  tvUtilization.sec(),
                                  tvUtilization.usec(),
                                  tvTotalOneHopBandWidthUtilization_.sec(),
                                  tvTotalOneHopBandWidthUtilization_.usec());
#endif

          // this nem
          if(nbrEntry->first == id_)
           {
             // set this nem bw utilization
             tvBandWidthUtilizationThisNem_ = tvUtilization;
           }
          // other nem(s)
          else
           {
             // sum number of one hop packets all DATA msg types
             totalOneHopNumPackets_ += nbrEntry->second.getNumberOfPackets(MSG_TYPE_MASK_ALL_DATA);

             // sum one hop rx power all DATA msg types
             dTotalRxPowerMilliWatts_ += nbrEntry->second.getRxPowerMilliWatts(MSG_TYPE_MASK_ALL_DATA);
           }
        }
    }

  // set the local node Tx value avoid / by 0
  if(tvTotalOneHopBandWidthUtilization_ != ACE_Time_Value::zero)
   {
      fLocalNodeTx_ = TV_TO_SEC(tvBandWidthUtilizationThisNem_) / TV_TO_SEC(tvTotalOneHopBandWidthUtilization_);
   }
  else
   {
      fLocalNodeTx_ = 0.0;
   }

#ifdef VERY_VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL,
                          "MACI %03hu %s::%s: %zd active one hop nbr(s), total [bw %ld:%06ld, pkts %zd, pwr %5.4lf mW], local tx %5.4f",
                                 id_, MODULE, __func__,
                                 oneHopUtilizationMap_.size(),
                                 tvTotalOneHopBandWidthUtilization_.sec(),
                                 tvTotalOneHopBandWidthUtilization_.usec(),
                                 totalOneHopNumPackets_,
                                 dTotalRxPowerMilliWatts_,
                                 fLocalNodeTx_);
#endif

  // each two hop nbr
  for (Neighbor2HopEntryMapIter nbr2Entry = twoHopNbrMap_.begin (); nbr2Entry != twoHopNbrMap_.end (); ++nbr2Entry)
    {
       // get bandwidth utilization
       const ACE_Time_Value tvUtilization = nbr2Entry->second.getBandwidtUtilization();

       // check any utilization
       if(tvUtilization > ACE_Time_Value::zero)
        {
          // store the two hop utilization
          twoHopUtilizationMap_[nbr2Entry->first] = tvUtilization;

          // sum total 2 hop bandwidth utilization
          tvTotalTwoHopBandWidthUtilization_ += tvUtilization;

          // sum number of 2 hop packets
          totalTwoHopNumPackets_ += nbr2Entry->second.getNumberOfPackets();

#ifdef VERY_VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: two hop nbr %hu, bw %ld:%06ld, total bw %ld:%06ld",
                                  id_, MODULE, __func__, 
                                  nbr2Entry->first,
                                  tvUtilization.sec(),
                                  tvUtilization.usec(),
                                  tvTotalTwoHopBandWidthUtilization_.sec(),
                                  tvTotalTwoHopBandWidthUtilization_.usec());
#endif

        }
    }

#ifdef VERY_VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: %zd active two hop nbr(s), total [bw %ld:%06ld, pkts %zd]",
                                 id_, MODULE, __func__, 
                                 twoHopUtilizationMap_.size(),
                                 tvTotalTwoHopBandWidthUtilization_.sec(),
                                 tvTotalTwoHopBandWidthUtilization_.usec(),
                                 totalTwoHopNumPackets_);
#endif

  // check two hop packet count, prevents / by 0
  if (totalTwoHopNumPackets_ > 0)
    {
      // sum B
      float B = 0.0;

      // average bandwidth utilization per active two hop nbr
      tvAverageUtilizationPerTwoHopNeighbor_ = tvTotalTwoHopBandWidthUtilization_ * (1.0 / twoHopUtilizationMap_.size());

      // each two hop utilization entry
      for(NbrUtilizationMapIter uiter = twoHopUtilizationMap_.begin(); uiter != twoHopUtilizationMap_.end(); ++uiter)
        {
          // get A
          const float A = getA_priv(uiter->second, tvAverageUtilizationPerTwoHopNeighbor_);

          // sum A^2
          B += (A*A);

#ifdef VERY_VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, 
                                  "MACI %03hu %s::%s: two hop A = %5.4f, B = %5.4f",
                                   id_, MODULE, __func__, 
                                   A, B);
#endif
        }

      // calculate the total estimated number of two hop nbrs
      fEstimatedNumTwoHopNeighbors_ = round(B);
    }


  // check one hop packet count, prevents / by 0
  if (totalOneHopNumPackets_ > 0)
    {
      // set num active nbrs, may include us
      numTotalActiveOneHopNeighbors_ = oneHopUtilizationMap_.size();

      // average bandwidth utilization per active one hop nbr
      tvAverageUtilizationPerOneHopNeighbor_ = tvTotalOneHopBandWidthUtilization_ * (1.0 / numTotalActiveOneHopNeighbors_);

      // average rx power per packet
      dAverageRxPowerPerMessageMilliWatts_ = dTotalRxPowerMilliWatts_ / totalOneHopNumPackets_;

      // average message duration (not including this nem)
      tvAverageMessageDuration_ = (tvTotalOneHopBandWidthUtilization_ - tvBandWidthUtilizationThisNem_) * (1.0 / totalOneHopNumPackets_);

      // sum B1
      float B1 = 0.0;
   
      float A1 = 0.0;

      float A2 = 0.0;

      // each one hop nbr
      for (NeighborEntryMapIter nbrEntry1 = oneHopNbrMap_.begin (); nbrEntry1 != oneHopNbrMap_.end (); ++nbrEntry1)
        {
          // check nbr activity all DATA msg types
          if(nbrEntry1->second.getNumberOfPackets (MSG_TYPE_MASK_ALL_DATA) > 0)
           {
             // get A
             A1 = getA_priv(nbrEntry1->second.getBandwidtUtilization(MSG_TYPE_MASK_ALL_DATA), 
                            tvAverageUtilizationPerOneHopNeighbor_);

             // sum A^2
             B1 += (A1*A1);

             // check nbr is not us
             if(nbrEntry1->first != id_)
              {
                // value B2
                float B2 = 0;

                // the sum of hidden bandwidth utilization
                ACE_Time_Value tvHiddenBandWidthUtilization = ACE_Time_Value::zero;

                // the sum of common rx power in mW
                double dCommonRxPowerMilliWatts = 0.0;

                // the sum of hidden rx power in mW
                double dHiddenRxPowerMilliWatts = 0.0;

                // the sum of common tx pkts
                size_t numCommonPackets = 0;
               
                // the sum of hidden tx pkts
                size_t numHiddenPackets = 0;
               
                // get the one hop nbrs of this nbr 
                const NbrSet nbrOneHopNbrSet = nbrEntry1->second.getOneHopNeighbors();

                std::stringstream ss;

                // list the nbrs of this nbr               
                for(NbrSetConstIter nbr = nbrOneHopNbrSet.begin(); nbr != nbrOneHopNbrSet.end(); ++nbr)
                 {
                   if(nbr == nbrOneHopNbrSet.begin())
                    {
                      ss << *nbr;
                    }
                   else
                    { 
                      ss << ", " << *nbr;
                    }
                 }

#ifdef VERBOSE_LOGGING
                pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: nbr %hu has %zu one hop nbrs: %s",
                                        id_, MODULE, __func__, nbrEntry1->first, nbrOneHopNbrSet.size(), ss.str().c_str());
#endif

                // nbrs hidden from this nbr
                NbrSet hiddenNbrs;
 
                // nbrs common with this nbr
                NbrSet commonNbrs;

                // each one hop nbr (again)
                for (NeighborEntryMapIter nbrEntry2 = oneHopNbrMap_.begin (); nbrEntry2 != oneHopNbrMap_.end (); ++nbrEntry2)
                 {
                    // does the nbr's one hop nbr set include our nbr
                    if(nbrOneHopNbrSet.find(nbrEntry2->first) != nbrOneHopNbrSet.end())
                     {
                        // get all DATA msg types
                        const int typeMask = MSG_TYPE_MASK_ALL_DATA;

                        // sum A 
                        A2 = getA_priv(nbrEntry2->second.getBandwidtUtilization(typeMask), tvAverageUtilizationPerOneHopNeighbor_);
 
                        // sum A^2
                        B2 +=(A2*A2);

                        // get common rx power 
                        const double dRxPowermW = nbrEntry2->second.getRxPowerMilliWatts(typeMask);

                        // get the common number of packets
                        const int numPackets = nbrEntry2->second.getNumberOfPackets(typeMask);

#ifdef VERY_VERBOSE_LOGGING
                        pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: our nbr %hu is common with nbr %hu, pkts %d, rxpwr %6.4lf mW",
                                                id_, MODULE, __func__, nbrEntry2->first, nbrEntry1->first, numPackets, dRxPowermW);
#endif

                        // add to common nbrs
                        commonNbrs.insert(nbrEntry2->first);

                        // save common rx power avoid / by 0
                        commonNbrAvgRxPowerMwMap_[nbrEntry2->first] = numPackets > 0 ? dRxPowermW / numPackets : 0;

                        // sum the common rx power
                        dCommonRxPowerMilliWatts += dRxPowermW;

                        // sum the common number of packets
                        numCommonPackets += numPackets;
                     }
                    // hidden from this nbr
                    else
                     {
                        // get all unicast and broadcast msg types
                        const int typeMask = (MSG_TYPE_MASK_UNICAST | MSG_TYPE_MASK_BROADCAST);

                        // sum the hidden bandwidth utilization
                        tvHiddenBandWidthUtilization += nbrEntry2->second.getBandwidtUtilization(typeMask);

                        // get hidden rx power
                        const float dRxPowermW = nbrEntry2->second.getRxPowerMilliWatts(typeMask);
 
                        // get the hidden number of packets
                        const int numPackets = nbrEntry2->second.getNumberOfPackets(typeMask);

#ifdef VERY_VERBOSE_LOGGING
                        pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: nbr %hu is hidden from nbr %hu, pkts %d, rxpwr %6.4lf mW",
                                                id_, MODULE, __func__, nbrEntry2->first, nbrEntry1->first, numPackets, dRxPowermW);
#endif

                        // add to hidden nbrs
                        hiddenNbrs.insert(nbrEntry2->first);
                       
                        // save hidden rx power avoid / by 0
                        hiddenNbrAvgRxPowerMwMap_[nbrEntry2->first] = numPackets > 0 ? dRxPowermW / numPackets : 0;

                        // sum the hidden rx power
                        dHiddenRxPowerMilliWatts += dRxPowermW;

                        // sum the hidden number of packets
                        numHiddenPackets += numPackets;
                     }
                 }

                 // set the common nbrs
                 nbrEntry1->second.setCommonNeighbors(commonNbrs);

                 // set the hidden nbrs
                 nbrEntry1->second.setHiddenNeighbors(hiddenNbrs);

                 // get the hidden channel activity
                 const float H = getH_priv(tvHiddenBandWidthUtilization, tvDeltaT);

#ifdef VERY_VERBOSE_LOGGING
                 pPlatformService_->log (EMANE::DEBUG_LEVEL, 
                                         "MACI %03hu %s::%s: one hop nbr %hu, A1 = %5.4f, A2 = %5.4f, B1 = %5.4f, B2 = %5.4f, H = %5.4f",
                                         id_, MODULE, __func__, 
                                         nbrEntry1->first, 
                                         A1, A2, B1, B2, H);
#endif
                 // set the estimated number of common nbrs this nbr
                 nbrEntry1->second.setEstimatedNumCommonNeighbors(round(B2));

                 // set the avg common rx power this nbr, avoid / by 0
                 nbrEntry1->second.setAverageCommonRxPowerMilliWatts(numCommonPackets > 0.0 ? dCommonRxPowerMilliWatts / numCommonPackets : 0.0);

                 // set the hidden channel activity this nbr 
                 nbrEntry1->second.setHiddenChannelActivity(H);

                 // set the avg hidden rx power this nbr, avoid / by 0
                 nbrEntry1->second.setAverageHiddenRxPowerMilliWatts(numHiddenPackets > 0.0 ? dHiddenRxPowerMilliWatts / numHiddenPackets : 0.0);

                 // set the overall sum of common pkts
                 sumCommonPackets_ += numCommonPackets;

                 // set the overall sum of hidden pkts
                 sumHiddenPackets_ += numHiddenPackets;

                 // set the overall pwr of common pkts
                 dCommonRxPowerMilliWatts_ += dCommonRxPowerMilliWatts;

                 // set the overall pwr of hidden pkts
                 dHiddenRxPowerMilliWatts_ += dHiddenRxPowerMilliWatts;
              }
           }
        }

      // set common and hidden probability
      setCommonAndHiddenProbability_priv();

      // calculate the total estimated number of one hop nbrs
      fEstimatedNumOneHopNeighbors_ = round(B1);


#ifdef VERY_VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, 
                              "MACI %03hu %s::%s: est nbrs [one hop %5.4f, two hop %5.4f], active nbrs %zd, elapsed time %ld:%06ld",
                              id_, MODULE, __func__, 
                              fEstimatedNumOneHopNeighbors_, 
                              fEstimatedNumTwoHopNeighbors_, 
                              numTotalActiveOneHopNeighbors_, 
                              tvDeltaT.sec(), 
                              tvDeltaT.usec());
#endif
    }
}



void
IEEE80211ABG::NeighborManager::resetCounters_priv()
{
  // internal call
 
  fLocalNodeTx_ = 0.0;

  totalOneHopNumPackets_ = 0;

  totalTwoHopNumPackets_ = 0;

  sumCommonPackets_ = 0;

  sumHiddenPackets_ = 0;

  numTotalActiveOneHopNeighbors_ = 0;

  dTotalRxPowerMilliWatts_ = 0.0;

  fEstimatedNumOneHopNeighbors_ = 0.0;

  fEstimatedNumTwoHopNeighbors_ = 0.0;

  dAverageRxPowerPerMessageMilliWatts_ = 0.0;

  dHiddenRxPowerMilliWatts_ = 0.0;

  dCommonRxPowerMilliWatts_ = 0.0;

  tvAverageMessageDuration_ = ACE_Time_Value::zero;

  tvTotalOneHopBandWidthUtilization_ = ACE_Time_Value::zero;

  tvTotalTwoHopBandWidthUtilization_ = ACE_Time_Value::zero;

  tvAverageUtilizationPerOneHopNeighbor_ = ACE_Time_Value::zero;

  tvAverageUtilizationPerTwoHopNeighbor_ = ACE_Time_Value::zero;

  tvBandWidthUtilizationThisNem_ = ACE_Time_Value::zero;

  oneHopUtilizationMap_.clear();

  twoHopUtilizationMap_.clear();
}



bool 
IEEE80211ABG::NeighborManager::flushOneHopNeighbors_priv(const ACE_Time_Value & tvCurrentTime, const ACE_Time_Value & tvTimeout)
{
  // internal call
  
  // number of expired entries
  size_t numExpired = 0;

  // each one hop nbr
  for (NeighborEntryMapIter nbrEntry = oneHopNbrMap_.begin (); nbrEntry != oneHopNbrMap_.end (); /* bump below */)
    {
      // nbr age
      const ACE_Time_Value tvNeighborAge = tvCurrentTime - nbrEntry->second.getLastActivityTime();

      // nbr timed out
      if (tvNeighborAge > tvTimeout)
        {
          // log first, then remove so nbr id is correct
#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: remove one hop nbr %hu, age %ld:%06ld, %zd nbrs remaining",
                                  id_, MODULE, __func__, 
                                  nbrEntry->first,
                                  tvNeighborAge.sec(),
                                  tvNeighborAge.usec(),
                                  oneHopNbrMap_.size() - 1);
#endif

          // remove entry and bump 
          oneHopNbrMap_.erase(nbrEntry++);

          // bump num expired
          ++numExpired;
        }
      else
        {
          // bump 
          ++nbrEntry;
        }
     }

   // return expired status
   return numExpired != 0;
}



bool 
IEEE80211ABG::NeighborManager::flushTwoHopNeighbors_priv(const ACE_Time_Value & tvCurrentTime, const ACE_Time_Value & tvTimeout)
{
   // internal call
   
   // number of expired entries
   size_t numExpired = 0;

   // each two hop nbr
   for (Neighbor2HopEntryMapIter nbr2Entry = twoHopNbrMap_.begin (); nbr2Entry != twoHopNbrMap_.end (); /* bump below */)
     {
       // nbr age
       const ACE_Time_Value tvNeighborAge = tvCurrentTime - nbr2Entry->second.getLastActivityTime();

       // nbr timed out
       if (tvNeighborAge > tvTimeout)
         {
           // log first, then remove so nbr id is correct
#ifdef VERBOSE_LOGGING
           pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: remove two hop nbr %hu, age %ld:%06ld, %zd nbrs remaining",
                                   id_, MODULE, __func__, 
                                   nbr2Entry->first,
                                   tvNeighborAge.sec(),
                                   tvNeighborAge.usec(),
                                   twoHopNbrMap_.size() - 1);
#endif

           // remove entry and bump 
           twoHopNbrMap_.erase(nbr2Entry++);

           // bump num expired
           ++numExpired;

         }
       else
         {
           // bump 
           ++nbr2Entry;
         }
     }

   // return expired status
   return numExpired != 0;
}



    
void 
IEEE80211ABG::NeighborManager::sendOneHopNbrListEvent_priv()
{ 
   // internal call
   
   // num nbrs plus us
   const size_t numEntries = oneHopNbrMap_.size() + 1;

   // nbr entry list
   OneHopNbrListEvent::OneHopNbrEntry entries[numEntries];

   // our id is first
   entries[0].u16NEMId_ = id_;

   // start at next position
   size_t pos = 1;

   // each one hop nbr
   for (NeighborEntryMapConstIter nbrEntry = oneHopNbrMap_.begin (); nbrEntry != oneHopNbrMap_.end (); ++nbrEntry, ++pos)
    {
      // insert nbr id
      entries[pos].u16NEMId_ = nbrEntry->first;
    }

   // create event
   OneHopNbrListEvent event (entries, numEntries);

   // send event
   pPlatformService_->sendEvent(0,                           // all platform(s)
                                0,                           // nem id
                                EMANE::COMPONENT_MACILAYER,  // mac layer
                                event);                      // the event

   // bump tx events
   pMgr_->getStatistics().incrementTxOneHopNbrListEventCount ();

   // update last event tx time
   tvLastOneHopNbrListTxTime_ = ACE_OS::gettimeofday();
}






IEEE80211ABG::NeighborManager::NeighborEntryInsertResult
IEEE80211ABG::NeighborManager::addOneHopNeighbor_priv (EMANE::NEMId src)
{
  // internal call

  // try to add one hop nbr
  const NeighborEntryInsertResult nbrResult = oneHopNbrMap_.insert (std::make_pair (src, NeighborEntry ()));

  // nbr added
  if(nbrResult.second == true)
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: added one hop nbr %hu, %zd total nbrs", 
                             id_, MODULE, __func__, src, oneHopNbrMap_.size());
#endif

     // set high water
     pMgr_->getStatistics ().updateOneHopNbrHighWaterMark (oneHopNbrMap_.size());

     // check one hop nbr list event cache
     const NbrSetMapIter iter = cachedOneHopNbrSetMap_.find(src);

     // entry found
     if(iter != cachedOneHopNbrSetMap_.end())
      {
#ifdef VERBOSE_LOGGING
        pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: copied one_hop_nbr_list from cache for nbr %hu, has %zd total nbrs", 
                                id_, MODULE, __func__, src, iter->second.size());
#endif

        // set one hop nbrs of this one hop nbr from the cache
        nbrResult.first->second.setOneHopNeighbors(iter->second);
      }
     else
      {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: no one_hop_nbr_list cache for nbr %hu, must wait for event update", 
                                 id_, MODULE, __func__, src);
#endif
      }
   }
  // existing nbr
  else
   {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: existing one hop nbr %hu, keep existing one_hop_nbr_list", 
                              id_, MODULE, __func__, src);
#endif
   }

   // return result
  return nbrResult;
}



IEEE80211ABG::NeighborManager::Neighbor2HopEntryInsertResult 
IEEE80211ABG::NeighborManager::addTwoHopNeighbor_priv (EMANE::NEMId src)
{
   // internal call
  
   // add 2 hop nbr 
   const Neighbor2HopEntryInsertResult nbr2Result = twoHopNbrMap_.insert (std::make_pair (src, Neighbor2HopEntry ()));

   // nbr added
   if(nbr2Result.second == true)
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: added two hop nbr %hu, %zd total nbrs", 
                              id_, MODULE, __func__, src, twoHopNbrMap_.size());
#endif
  
      // set high water
      pMgr_->getStatistics ().updateTwoHopNbrHighWaterMark (twoHopNbrMap_.size());
    }
   // existing nbr
   else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: existing two hop nbr %hu, %zd total nbrs", 
                              id_, MODULE, __func__, src, twoHopNbrMap_.size());
#endif

    }

   // return result
   return nbr2Result;
 }




float 
IEEE80211ABG::NeighborManager::getA_priv(const ACE_Time_Value &tvUtilization, const ACE_Time_Value & tvAverageUtilization) const
{
  // internal call
  
  const float A = getChannelActivity_priv(tvUtilization, tvAverageUtilization);

  // cap at 1.0
  if(A > 1.0)
   {
     return 1.0;
   }
  else
   {
     return A;
   }
}


  
float 
IEEE80211ABG::NeighborManager::getC_priv(const ACE_Time_Value & tvUtilization, const ACE_Time_Value &tvDeltaT) const
{
  // internal call
  
  const float C = getChannelActivity_priv(tvUtilization, tvDeltaT);

  // cap to 1.0
  if(C > 1.0)
   {
     return 1.0;
   }
  else
   {
     return C;
   }
}


 
float 
IEEE80211ABG::NeighborManager::getH_priv(const ACE_Time_Value & tvUtilization, const ACE_Time_Value &tvDeltaT) const
{
  // internal call
 
  const float H = getChannelActivity_priv(tvUtilization, tvDeltaT);

  // cap at 1.0
  if(H > 1.0)
   {
     return 1.0;
   }
  else
   {
     return H;
   }
}



float 
IEEE80211ABG::NeighborManager::getChannelActivity_priv(const ACE_Time_Value & tvUtilization, const ACE_Time_Value &tvDeltaT) const
{
  // internal call
  
  // check divide by zero
  if (tvDeltaT == ACE_Time_Value::zero)
   {
     // no utilzation
     return 0.0;
   }
  else
   {
     return TV_TO_SEC(tvUtilization) / TV_TO_SEC(tvDeltaT);
   }
}


void 
IEEE80211ABG::NeighborManager::setCommonAndHiddenProbability_priv()
{
   // for each one hop utilization
   for(NbrUtilizationMapIter uiter = oneHopUtilizationMap_.begin(); uiter != oneHopUtilizationMap_.end(); ++uiter)
    {
      // exclude ourself
      if(uiter->first != id_)
       {
          // remove local and src utilization from total one hop utilization
          const ACE_Time_Value tvAdjustedUtilization = tvTotalOneHopBandWidthUtilization_ - tvBandWidthUtilizationThisNem_ - uiter->second;

          // check remaining utlzation
          if(tvAdjustedUtilization > ACE_Time_Value::zero)
           {
             // lookup nbr
             const NeighborEntryMapIter niter = oneHopNbrMap_.find(uiter->first);

             // nbr found
             if(niter != oneHopNbrMap_.end())
              {
                // get common nbrs
                const NbrSet commonNbrs = niter->second.getCommonNeighbors();

                // get hidden nbrs
                const NbrSet hiddenNbrs = niter->second.getHiddenNeighbors();

                // get one hop utilzation 
                NbrUtilizationMap adjustedUtilzationMap = oneHopUtilizationMap_;

                // remove local node from utilization
                adjustedUtilzationMap.erase(id_);

                // remove src from utilization
                adjustedUtilzationMap.erase(uiter->first);

                // get and store common probability using adjusted one hop utilization for this src
                commonProbabilityMapMap_[uiter->first] = setProbability_priv(uiter->first, 
                                                                            adjustedUtilzationMap, 
                                                                            tvAdjustedUtilization, 
                                                                            commonNbrs,
                                                                            "common");

               // get and store hidden probability using adjusted one hop utilization for this src
               hiddenProbabilityMapMap_[uiter->first] = setProbability_priv(uiter->first, 
                                                                           adjustedUtilzationMap, 
                                                                           tvAdjustedUtilization,
                                                                           hiddenNbrs,
                                                                           "hidden");
             }
          }
       }
    }
}



IEEE80211ABG::NeighborManager::ProbabilityPairMap 
IEEE80211ABG::NeighborManager::setProbability_priv(EMANE::NEMId src, const NbrUtilizationMap & map, 
                                                   ACE_Time_Value tv, const NbrSet & nbrSet, const char *str) const
{
  // probability map result
  ProbabilityPairMap probabilityMap;

  // the adjusted utilization 
  NbrUtilizationMap adjustedUtilizationMap;

  // check utilization
  for(NbrUtilizationMapConstIter uiter = map.begin(); uiter != map.end(); ++uiter)
   { 
      // found
      if(nbrSet.find(uiter->first) != nbrSet.end())
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s_%s: src %hu, is %s w/r to nbr %hu", 
                                 id_, MODULE, __func__, str, src, str, uiter->first);
#else
  (void) src;
  (void) str;
#endif

         // add entry
         adjustedUtilizationMap[uiter->first] = uiter->second;
       }
      else
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s_%s: src %hu, is not %s w/r to nbr %hu, ignore", 
                                 id_, MODULE, __func__, str, src, str, uiter->first);
#endif

         // subtract utilization
         tv -= uiter->second;
       }
   }

  // check remaining utilization
  if(tv > ACE_Time_Value::zero)
   {
     // initial values
     float p1 = 0.0;
     float p2 = 0.0;

     // get num entries
     size_t num = adjustedUtilizationMap.size();

     // for each utilization
     for(NbrUtilizationMapIter uiter = adjustedUtilizationMap.begin(); uiter != adjustedUtilizationMap.end(); ++uiter, --num)
      {
         // not last entry
         if(num != 1)
          {
            // get ratio 
            p2 += TV_TO_SEC(uiter->second) / TV_TO_SEC(tv);
          }
         // last entry
         else
          {
            // set to 1
            p2 = 1.0;
          }
   
#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s_%s: src %hu, nbr %hu, p1 %5.4f, p2 %5.4f", 
                                  id_, MODULE, __func__, str, src, uiter->first, p1, p2);
#endif

          // insert range 
          probabilityMap[uiter->first] = ProbabilityPair(p1, p2);

          // bump range
          p1 = p2;
      }
   }

  // return result
  return probabilityMap;
}
