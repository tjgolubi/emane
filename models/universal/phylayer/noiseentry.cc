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


#include "noiseentry.h"


UniversalPHY::NoiseEntry::NoiseEntry () :
  tvTime_(ACE_Time_Value::zero),
  tvDuration_(ACE_Time_Value::zero),
  tvEndTime_(ACE_Time_Value::zero),
  dNoiseMilliWatt_(0.0)
{ }




UniversalPHY::NoiseEntry::NoiseEntry (const ACE_Time_Value &tvT0, 
                                      const ACE_Time_Value & tvOffset, 
                                      const ACE_Time_Value &tvDuration, 
                                      const double dNoiseMilliWatt) :
  tvTime_(tvT0 + tvOffset),
  tvDuration_(tvDuration),
  tvEndTime_(tvT0 + tvOffset + tvDuration),
  dNoiseMilliWatt_(dNoiseMilliWatt)
{ }



double 
UniversalPHY::NoiseEntry::getNoiseMilliWatt() const
{
  return dNoiseMilliWatt_;
}



UniversalPHY::NoiseEntry::NOISE_STATUS 
UniversalPHY::NoiseEntry::getNoiseStatus(const ACE_Time_Value & tvT0, const ACE_Time_Value & tvOffset, const ACE_Time_Value & tvDuration)
{
  // entry has expired
  if(tvEndTime_ <= tvT0)
   {
     return UniversalPHY::NoiseEntry::NOISE_STATUS_EXPIRED;
   }
  else
   {
     // entry is current
     if(tvTime_ < (tvT0 + tvOffset + tvDuration))
      {
        return UniversalPHY::NoiseEntry::NOISE_STATUS_CURRENT;
      }
     // entry is in the future
     else
      {
        return UniversalPHY::NoiseEntry::NOISE_STATUS_FUTURE;
      }
    }
}
