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

#ifndef UNIVERSALPHY_ANTENNAPROFILEMANAGER_HEADER_
#define UNIVERSALPHY_ANTENNAPROFILEMANAGER_HEADER_

#include "emane/emanetypes.h"
#include "emane/emaneexception.h"
#include "emane/emaneantennamode.h"
#include "emane/emaneplatformserviceprovider.h"
#include "emane/emanecommonphyheader.h"
#include "emane/emanepositiongeodetic.h"
#include "emane/emanevelocityvector.h"
#include "antennaprofileparser.h"

#include "antennaprofile.h"
#include "antennaprofilepattern.h"

#include <string>
#include <map>

#include <ace/Basic_Types.h>

namespace UniversalPHY {

  /**
   *
   * @class  AntennaProfileManager 
   *
   * @brief  manages antenna profile info
   *
   */
  class AntennaProfileManager {
    public: 

    /**
     * @brief constructor
     *
     * @param id the NEM id of this instance
     * @param pPlatformService pointer to the platform servicd
     *
     */
     AntennaProfileManager(const EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService);


    /**
     *
     * @param bEnableAntennaProfile antenna profile enable/disable
     * @param uri                   antenna profile uri
     * @param u16ProfileId          antenna profile id
     *
     * @exception throws EMANEException
     *
     */
     void configure(const bool bEnableAntennaProfile, const std::string & uri, const ACE_UINT16 u16ProfileId)
        throw (EMANE::EMANEException);


    /**
     *
     * @return returns the antenna profile id
     *
     */
     ACE_UINT16 getAntennaProfileId() const;


    /**
     *
     * @param fValue sets the fixed antenna gain in dBi
     *
     */
     void setFixedAntennaGaindBi(const float fValue);


    /**
     *
     * @param mode antenna gain mode (fixed or profile)
     *
     */
     void setAntennaMode(const EMANE::ANTENNA_MODE mode);


    /**
     *
     * @return returns the antenna gain mode 
     *
     */
     EMANE::ANTENNA_MODE getAntennaMode() const;


    /**
     *
     * @brief  returns a formated string of paramters and values for logging
     *
     */
     std::string format() const;


    /**
     *
     * @return returns the antenna gain in dBi
     *
     */
     float getFixedGaindBi() const;


    /**
     *
     * @brief  get antenna gain based on pointing and position
     *
     * @param phyHeader      the phy header associated with this transmission
     * @param remoteProfile  the remote antenna profile info
     * @param localPosition  the local position info
     * @param remotePosition the remote position info
     * @param localVelocity  the local velocity info
     * @param remoteVelocity the remote velocity info
     * @param fLocalAntennaAzimuthDegrees local antenna azimuth
     * @param fLocalAntennaElevationDegrees local antenna elevation
     * @param bHaveValidPositions indicator the vaild position info is available
     *
     * @return antenna gain in dBi
     *
     */
     float getGaindBi(const EMANE::CommonPHYHeader & phyHeader, 
                      const AntennaProfile & remoteProfile,
                      const EMANE::PositionGeodetic & localPosition,
                      const EMANE::PositionGeodetic & remotePosition,
                      const EMANE::VelocityVector & localVelocity,
                      const EMANE::VelocityVector & remoteVelocity,
                      const float fLocalAntennaAzimuthDegrees,
                      const float fLocalAntennaElevationDegrees,
                      const bool bHaveValidPositions) const;

    /**
     *
     * @param u16ProfileId the requested antenna profile id
     *
     * @return returns true if a change was made to antenna profile info else returns false
     *
     */
     bool handleAntennaProfileChange(const ACE_UINT16 u16ProfileId);

    private:
      void loadProfiles (const std::string & uri)
        throw (EMANE::EMANEException);

      EMANE::PositionGeodetic adjustPosition(const EMANE::PositionGeodetic & position, 
                                             const EMANE::VelocityVector & velocity) const;

      bool setAntennaProfileId (const ACE_UINT16 u16ProfileId);

      float getGainValue (xmlNodePtr cur);
       
      const EMANE::NEMId id_;

      EMANE::PlatformServiceProvider * pPlatformService_;

      AntennaProfilePattern::AntennaProfilePatternMap antennaProfilePatterns_;

      AntennaProfilePattern localAntennaProfile_;

      float fixedGaindBi_;

      EMANE::ANTENNA_MODE antennaMode_;
   };
}
#endif //UNIVERSALPHY_ANTENNAPROFILEMANAGER_HEADER_
