/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "noisemanager.h"
#include "emaneutils/netutils.h"



UniversalPHY::NoiseManager::NoiseManager (EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService) : 
  id_(id),
  pPlatformService_(pPlatformService)
{ }


UniversalPHY::NoiseManager::~NoiseManager()
{ }




EMANE::PHYRxFrequencyInfoItems 
UniversalPHY::NoiseManager::getRxFrequencyInfo (const ACE_UINT64 u64BandWidthHz, 
                                                const ACE_Time_Value &tvT0, 
                                                const EMANE::PHYTxFrequencyInfoItems & items,
                                                const double dRxSensitivityMilliWatt)
{
   EMANE::PHYRxFrequencyInfoItems result;

   // for each tx (received ota) freq info entry
   for(EMANE::PHYTxFrequencyInfoItemsConstIter txFreqIter = items.begin(); txFreqIter != items.end(); ++txFreqIter)
    {
#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu NoiseManager::%s: check rx freq %s against %zd FOI(s)",
                               id_, __func__,
                               EMANEUtils::formatFrequency(txFreqIter->getCenterFrequencyHz()).c_str(),
                               frequenciesOfInterest_.size());
#endif


      // total noise in milliwatts
      double dTotalNoiseMilliWatt = 0.0;

      // freq index
      size_t idx = 0;

      // check each frequency of interest
      for(UniversalPHY::NoiseManager::FrequencyEntryListIter foiIter = frequenciesOfInterest_.begin(); 
            foiIter != frequenciesOfInterest_.end(); ++foiIter, ++idx)
       {
         // get band overlap
         const float fOverlapRatio = foiIter->getBandOverlap(txFreqIter->getCenterFrequencyHz(), u64BandWidthHz);

         // if coverage is greater than zero
         if(fOverlapRatio > 0.0)
          {
            // get the nosie for this foi
            double dNoiseMilliWatt = 0.0;

            // check noise entries
            for(NoiseEntryListIter niter = noiseEntryListItems_[idx].begin(); niter != noiseEntryListItems_[idx].end(); /* bump below */)
             {
               // get entry time status
               const UniversalPHY::NoiseEntry::NOISE_STATUS noiseStatus = niter->getNoiseStatus(tvT0,                       // time
                                                                                                txFreqIter->getTxOffset(),  // offset
                                                                                                txFreqIter->getDuration()); // duration

               // entry has expired
               if(noiseStatus == UniversalPHY::NoiseEntry::NOISE_STATUS_EXPIRED)
                {
                  // remove entry and bump
                  niter = noiseEntryListItems_[idx].erase(niter); 
                } 
               // entry is within time window
               else if(noiseStatus == UniversalPHY::NoiseEntry::NOISE_STATUS_CURRENT)
                {
                  // accumulate noise
                  dNoiseMilliWatt += niter->getNoiseMilliWatt();

                  // bump 
                  ++niter;
                }
               // not in window, ignore
               else
                {
                  // bump 
                  ++niter;
                }
             }

            // accumulate noise
            dTotalNoiseMilliWatt += dNoiseMilliWatt;

#ifdef VERBOSE_LOGGING
            pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu NoiseManager::%s: freq %s, bandwidth %s, FOI %s, "
                                   "overlap %4.2f%%, duration %ld:%06ld, offset %ld:%06ld, noise %e mW, total noise %e mW",
                                   id_, __func__,
                                   EMANEUtils::formatFrequency(txFreqIter->getCenterFrequencyHz()).c_str(),
                                   EMANEUtils::formatFrequency(u64BandWidthHz).c_str(),
                                   EMANEUtils::formatFrequency(foiIter->getCenterFrequencyHz()).c_str(),
                                   fOverlapRatio * 100.0, 
                                   txFreqIter->getDuration().sec(), 
                                   txFreqIter->getDuration().usec(), 
                                   txFreqIter->getTxOffset().sec(), 
                                   txFreqIter->getTxOffset().usec(), 
                                   dNoiseMilliWatt,
                                   dTotalNoiseMilliWatt);
#endif
          }
       }

      // rx freq info
      EMANE::PHYRxFrequencyInfo info (txFreqIter->getCenterFrequencyHz(),        // freq
                                      txFreqIter->getTxOffset(),                 // offset
                                      txFreqIter->getDuration(),                 // duration
                                      MILLIWATT_TO_DB(dTotalNoiseMilliWatt + 
                                                      dRxSensitivityMilliWatt)); // noise floor in dBm

#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu NoiseManager::%s: %s", id_, __func__, info.format().c_str());
#endif

      // add info
      result.push_back(info);
    }

   return result;
}

     
 
void
UniversalPHY::NoiseManager::addNoiseMilliWatt (const double fNoiseMilliWatt, 
                                               const ACE_UINT64 u64BandWidthHz, 
                                               const ACE_Time_Value &tvT0, 
                                               const EMANE::PHYTxFrequencyInfoItems & items)
{
   // for each freq info entry
   for(EMANE::PHYTxFrequencyInfoItemsConstIter txFreqIter = items.begin(); txFreqIter != items.end(); ++txFreqIter)
    {
      // freq index
      size_t idx = 0;

      // check each frequency of interest
      for(UniversalPHY::NoiseManager::FrequencyEntryListIter foiIter = frequenciesOfInterest_.begin(); 
            foiIter != frequenciesOfInterest_.end(); ++foiIter, ++idx)
       {
         // get band overlap ratio
         const double fOverlapRatio = foiIter->getBandOverlap(txFreqIter->getCenterFrequencyHz(), u64BandWidthHz);

         // adjusted noise level
         const double fNoiseAdjustedMilliWatt = fNoiseMilliWatt * fOverlapRatio;

         // if noise greater than zero
         if(fNoiseAdjustedMilliWatt > 0.0)
          {
            // add noise at the initial time plus offset for the duratiopn
            noiseEntryListItems_[idx].push_back(UniversalPHY::NoiseEntry(tvT0,                       // initial time
                                                                         txFreqIter->getTxOffset(),  // time offset
                                                                         txFreqIter->getDuration(),  // duration
                                                                         fNoiseAdjustedMilliWatt));  // noise
          }

#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu NoiseManager::%s: freq %s, bandwidth %s, FOI %s, "
                                "overlap %4.2f%%, duration %ld:%06ld, offset %ld:%06ld, noise %3.2f mW",
                                id_, __func__,
                                EMANEUtils::formatFrequency(txFreqIter->getCenterFrequencyHz()).c_str(),
                                EMANEUtils::formatFrequency(u64BandWidthHz).c_str(),
                                EMANEUtils::formatFrequency(foiIter->getCenterFrequencyHz()).c_str(),
                                fOverlapRatio * 100.0, 
                                txFreqIter->getDuration().sec(), 
                                txFreqIter->getDuration().usec(), 
                                txFreqIter->getTxOffset().sec(), 
                                txFreqIter->getTxOffset().usec(), 
                                fNoiseAdjustedMilliWatt);
#endif
      }
   }
}


void 
UniversalPHY::NoiseManager::setFrequencies(const UniversalPHY::NoiseManager::FrequencyOfInterestSet & foiSet, 
                                           const ACE_UINT64 u64BandWidthHz)
{
   // clear all frequency entries
   frequenciesOfInterest_.clear();
 
   // clear all noise entries
   noiseEntryListItems_.clear();

   // for each foi
   for(UniversalPHY::NoiseManager::FrequencyOfInterestSetIter iter = foiSet.begin(); iter != foiSet.end(); ++iter)
    {
      // add frequency entry
      frequenciesOfInterest_.push_back(UniversalPHY::FrequencyEntry(*iter, u64BandWidthHz));

      // add noise list
      noiseEntryListItems_.push_back(UniversalPHY::NoiseManager::NoiseEntryList());
    }
}
