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

#ifndef UNIVERSALPHY_ANTENNAPROFILEPATTERN_HEADER_
#define UNIVERSALPHY_ANTENNAPROFILEPATTERN_HEADER_

#include "emane/emanetypes.h"
#include "emane/emanepositiongeodetic.h"
#include "emane/emanepositionneu.h"

#include <ace/Basic_Types.h>

#include <map>

#include <ace/OS_NS_stdio.h>

namespace UniversalPHY {

  /**
   *
   * @class AntennaProfilePattern
   *
   * @brief Defines the antenna profile gain and blockage patterns
   *
   */
  class AntennaProfilePattern
  {
    public: 

     /**
      *
      * @brief  defines an arc range (low high)
      *
      */
    typedef std::pair<int, int>  ArcRangePair;

     /**
      *
      * @param r arc range
      * @param val the value to search for
      *
      * @return returns -1 if val is low, 
      *         returns 0 if val is in range, 
      *         returns 1 if val is high
      *
      */
    inline int in_range(const ArcRangePair & r, const int val) const
     {
       // value to too low
       if(val < r.first)
        {
#ifdef EMANE_DEBUG
          printf("val is too low: (%d < %d) give up now\n", val, r.first);
#endif
          return -1;
        }
       // value is higher than this range
       else if(val > r.second)
        {
#ifdef EMANE_DEBUG
          printf("val is too high: (%d > %d) keep trying\n", val, r.second);
#endif
          return 1;  
        }
       // middle
       else 
        {
#ifdef EMANE_DEBUG
          printf("val found (%d <= %d <= %d)\n", r.first, val, r.second);
#endif
          return 0;
        }
     }

    // mapping of a bearing and gain value
    typedef std::map<ArcRangePair, float>   BearingGainMap;
    typedef BearingGainMap::iterator        BearingGainMapIter;
    typedef BearingGainMap::const_iterator  BearingGainMapConstIter;

    // mapping of an elevation to a bearing gain mapping
    typedef std::map<ArcRangePair, BearingGainMap>  ElevationBearingGainMap;
    typedef ElevationBearingGainMap::iterator       ElevationBearingGainMapIter;
    typedef ElevationBearingGainMap::const_iterator ElevationBearingGainMapConstIter;

    // mapping of an antenna profile pattern
    typedef std::map<ACE_UINT16, AntennaProfilePattern>  AntennaProfilePatternMap;
    typedef AntennaProfilePatternMap::iterator           AntennaProfilePatternMapIter;
    typedef AntennaProfilePatternMap::const_iterator     AntennaProfilePatternMapConstIter;

    /**
     *
     * @brief  default constructor
     *
     */
     AntennaProfilePattern();

    /**
     *
     * @brief  parameter constructor
     *
     * @param u16ProfileId the antenna profile id
     * @param gain the antenna gain pattern
     * @param blockage the antenna blockage pattern
     * @param placement the antenna placement
     *
     */
     AntennaProfilePattern (const ACE_UINT16 u16ProfileId, 
                            const ElevationBearingGainMap & gain,
                            const ElevationBearingGainMap & blockage,
                            const EMANE::PositionNEU & placement);

    /**
     *
     * @return returns the antenna profile id
     *
     */
     ACE_UINT16 getAntennaProfileId() const;

    /**
     *
     * @param bearing            antenna bearing
     * @param elevation          antenna elevation
     *
     * @return returns the antenna gain in dBi
     *
     */
     float getGain(const int bearing, const int elevation) const;

    /**
     *
     * @param bearing            antenna bearing
     * @param elevation          antenna elevation
     *
     * @return returns the antenna blockage in dBi
     *
     */
     float getBlockage(const int bearing, const int elevation) const;

    /**
     *
     * @return returns a const reference to the antenna placement
     *
     */
     const EMANE::PositionNEU & getAntennaPlacement () const;

    private:
      ACE_UINT16 u16ProfileId_;

      ElevationBearingGainMap antennaGain_;

      ElevationBearingGainMap antennaBlockage_;

      EMANE::PositionNEU placement_;

      float lookupValue(const int bearing, const int elevation, 
                          const ElevationBearingGainMap & map, const float fDefault) const;
   };

}
#endif //UNIVERSALPHY_ANTENNAPROFILEPATTERN_HEADER_
