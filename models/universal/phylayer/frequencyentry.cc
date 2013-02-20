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


#include "frequencyentry.h"


UniversalPHY::FrequencyEntry::FrequencyEntry(const ACE_UINT64 u64CenterFrequencyHz, const ACE_UINT64 u64BandWidthHz) :
  u64CenterFrequencyHz_(u64CenterFrequencyHz),
  u64BandWidthHz_(u64BandWidthHz),
  fLow_ (u64CenterFrequencyHz - ((float) u64BandWidthHz / 2.0)),
  fHigh_(u64CenterFrequencyHz + ((float) u64BandWidthHz / 2.0))
{ }



ACE_UINT64 
UniversalPHY::FrequencyEntry::getCenterFrequencyHz() const
{
  return u64CenterFrequencyHz_;
}



ACE_UINT64
UniversalPHY::FrequencyEntry::getBandWidthHz() const
{
  return u64BandWidthHz_;
}



float 
UniversalPHY::FrequencyEntry::getBandOverlap(const ACE_UINT64 u64CenterFrequencyHz, const ACE_UINT64 u64BandWidthHz) const
{
  // signal low
  const float fl = u64CenterFrequencyHz - ((float) u64BandWidthHz / 2.0);

  // signal high
  const float fh = u64CenterFrequencyHz + ((float) u64BandWidthHz / 2.0);

  // percent in band
  float fRatio;

  // signal is somewhere in band
  if((fl < fHigh_) && (fh > fLow_))
   {
    // low is within lower bound
    if(fl >= fLow_)
     {
      // high is within upper bound
      if(fh <= fHigh_)
       {
         // full coverage
         fRatio = 1.0;
       }
      // exceeded upper bound
      else
       {
         // therfore partial coverage
         fRatio = ((float)(fHigh_ - fl) / (float)u64BandWidthHz);
       }
     }
    // low is below lower bound
    else
     {
       // the signal is at or beyond
       if(fh <= fHigh_)
        {
          // therfore partial coverage
          fRatio = ((float)(fh - fLow_) / (float)u64BandWidthHz);
        }
       else
        {
          fRatio = ((float)(fHigh_ - fLow_) / (float)u64BandWidthHz);
        }
     }
   }
  // not in band at all
  else
   {
     // no coverage
     fRatio = 0.0;
   }

  // return ratio 
  return fRatio;
}
