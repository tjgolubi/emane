/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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


#include "antennaprofilepattern.h"
#include "emane/emaneconstants.h"

#include <ace/OS_NS_stdio.h>

UniversalPHY::AntennaProfilePattern::AntennaProfilePattern() :
   u16ProfileId_(0)
{ }


UniversalPHY::AntennaProfilePattern::AntennaProfilePattern (const ACE_UINT16 u16ProfileId, 
                                                            const ElevationBearingGainMap & antennaGain,
                                                            const ElevationBearingGainMap & antennaBlockage,
                                                            const EMANE::PositionNEU & placement) :
   u16ProfileId_(u16ProfileId),
   antennaGain_(antennaGain),
   antennaBlockage_(antennaBlockage),
   placement_(placement)
{ }



ACE_UINT16 UniversalPHY::AntennaProfilePattern::getAntennaProfileId() const
 {
   return u16ProfileId_;
 }



float UniversalPHY::AntennaProfilePattern::getGain(const int bearing, const int elevation) const
{
   // get antenna gain
   return lookupValue(bearing, elevation, antennaGain_, -EMANE::FULL_PATH_LOSS_DB);
}



float UniversalPHY::AntennaProfilePattern::getBlockage(const int bearing, const int elevation) const
{
   // get antenna blockage
   return lookupValue(bearing, elevation, antennaBlockage_, 0);
}


float UniversalPHY::AntennaProfilePattern::lookupValue(const int bearing, const int elevation, 
                                                      const ElevationBearingGainMap & map, const float def) const
{
   for(ElevationBearingGainMapConstIter iter1 = map.begin(); iter1 != map.end(); ++iter1)
    {
      const int pos1 = AntennaProfilePattern::in_range(iter1->first, elevation);

      if(pos1 < 0)
       {
         break;
       }
      else if (pos1 > 0)
       {
         continue;
       }
      else
       {
         for(BearingGainMapConstIter iter2 = iter1->second.begin(); iter2 != iter1->second.end(); ++iter2)
          {
            const int pos2 = AntennaProfilePattern::in_range(iter2->first, bearing);

            if(pos2 < 0)
             {
               break;
             }
            else if (pos2 > 0)
             {
               continue;
             }
            else
             {
#ifdef EMANE_DEBUG
               printf("found, elevation %d, bearing %d, value %f\n", elevation, bearing, iter2->second);
#endif
               return iter2->second;
             }
          }
       }
    }

#ifdef EMANE_DEBUG
      printf("not found, elevation %d, bearing %d, default %f\n", 
              elevation, bearing, def);
#endif

   return def;
}



const EMANE::PositionNEU & 
UniversalPHY::AntennaProfilePattern::getAntennaPlacement () const
{
  return placement_;
}
