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

#ifndef UNIVERSALPHY_NOISEMANAGER_HEADER_
#define UNIVERSALPHY_NOISEMANAGER_HEADER_


#include "emane/emanetypes.h"
#include "emane/emaneplatformserviceprovider.h"
#include "emane/emanecommonphyfreqinfo.h"
#include "frequencyentry.h"
#include "noiseentry.h"

#include <ace/Basic_Types.h>
#include <ace/Time_Value.h>

#include <math.h>

#include <set>
#include <list>
#include <vector>

#define DB_TO_MILLIWATT(x) (pow(10.0, ((x) / 10.0)))

#define MILLIWATT_TO_DB(x) (10.0 * log10((x)))


namespace UniversalPHY {

/**
 * @class NoiseManager
 *
 * @brief manages the noise level for each frequency set
 *
 */
 class NoiseManager
  {
    public:
      // frequency set and interators
      typedef std::set<ACE_UINT64>                   FrequencyOfInterestSet;
      typedef FrequencyOfInterestSet::iterator       FrequencyOfInterestSetIter;
      typedef FrequencyOfInterestSet::const_iterator FrequencyOfInterestSetConstIter;

      /**
       *
       * @brief constructor
       *
       * @param id the NEM id of this instance
       * @param pPlatformService the platfrom service for this instance
       *
       */
      NoiseManager (EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService);

      ~NoiseManager ();

      /**
       * sets the center frequencies of interest and the bandwidth of those frequencie(s)
       *
       * @param foi             reference to the set of frequencies of interest 
       * @param u64BandWidthHz  the bandwidth of each frequency
       *
       */
      void setFrequencies(const FrequencyOfInterestSet & foi, ACE_UINT64 u64BandWidthHz);


      /**
       *
       * gets the noise in milliwatts for a given frequency, bandidth and interval, takes into account frequency overlap
       *
       * @param u64BandWidthHz          the bandwidth of each frequency
       * @param tvT0                    the absoulte time of interest
       * @param items                   the list of freq/duration/offset entries
       * @param dRxSensitivityMilliWatt the rx sensitivity in milli watts
       *
       */
      EMANE::PHYRxFrequencyInfoItems 
            getRxFrequencyInfo (const ACE_UINT64 u64BandWidthHz, 
                                const ACE_Time_Value & tvT0, 
                                const EMANE::PHYTxFrequencyInfoItems & items,
                                const double dRxSensitivityMilliWatt);

      /**
       *
       * adds the noise in milliwatts for a given frequency, bandwidth and interval
       *
       * @param dNoiseMilliWatt  the noise level to be added in milliwatts
       * @param u64BandWidthHz   the bandwidth of each frequency
       * @param tvT0             the absoulte time of interest
       * @param items            the list of freq/duration/offset entries
       *
       */
      void addNoiseMilliWatt (const double dNoiseMilliWatt, 
                              const ACE_UINT64 u64BandWidthHz, 
                              const ACE_Time_Value &tvT0,
                              const EMANE::PHYTxFrequencyInfoItems & items);

    private:
      typedef std::list<FrequencyEntry>    FrequencyEntryList;
      typedef FrequencyEntryList::iterator FrequencyEntryListIter;

      typedef std::list<NoiseEntry>    NoiseEntryList;
      typedef NoiseEntryList::iterator NoiseEntryListIter;

      typedef std::vector<NoiseEntryList>   NoiseEntryListItems;
      typedef NoiseEntryListItems::iterator NoiseEntryListItemsIter;

      EMANE::NEMId id_;

      EMANE::PlatformServiceProvider * pPlatformService_;

      FrequencyEntryList frequenciesOfInterest_;

      NoiseEntryListItems noiseEntryListItems_;
  };
}

#endif //UNIVERSALPHY_NOISEMANAGER_HEADER_
