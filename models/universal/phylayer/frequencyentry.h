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

#ifndef UNIVERSALPHY_FREQUENCYENTRY_HEADER_
#define UNIVERSALPHY_FREQUENCYENTRY_HEADER_


#include "emane/emanetypes.h"

#include <ace/Basic_Types.h>
#include <ace/Time_Value.h>

#include <list>


namespace UniversalPHY {

  /**
   * @class FrequencyEntry 
   *
   * @brief Defines a frequency entry (center frequency and bandwidth).
   *
   */
 class FrequencyEntry {
    public:

     /**
      *
      * @brief   parameter constructor
      *
      * @param   u64CenterFrequencyHz   center frequency in Hz
      * @param   u64BandWidthHz         bandwidth in Hz
      *
      */
     FrequencyEntry(const ACE_UINT64 u64CenterFrequencyHz, const ACE_UINT64 u64BandWidthHz);


     /**
      *
      * @param   u64CenterFrequencyHz   center frequency in Hz
      * @param   u64BandWidthHz         bandwidth in Hz
      *
      * @return  returns percentage of band overlap
      *
      */
     float getBandOverlap(const ACE_UINT64 u64CenterFrequencyHz, const ACE_UINT64 u64BandWidthHz) const;


     /**
      *
      * @brief  get center frequency
      *
      * @return  returns center frequency in Hz
      *
      */
     ACE_UINT64 getCenterFrequencyHz() const;


     /**
      *
      * @brief  get band width
      *
      * @return  returns bandwidth in Hz
      *
      */
     ACE_UINT64 getBandWidthHz() const;

   private:
     const ACE_UINT64 u64CenterFrequencyHz_;
     const ACE_UINT64 u64BandWidthHz_;

     const float fLow_;
     const float fHigh_;
  };
}

#endif //UNIVERSALPHY_FREQUENCYENTRY_HEADER_
