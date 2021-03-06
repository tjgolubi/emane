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

#ifndef UNIVERSALPHY_ANTENNAPROFILE_HEADER_
#define UNIVERSALPHY_ANTENNAPROFILE_HEADER_

#include "emane/emanetypes.h"

#include <ace/Basic_Types.h>

namespace UniversalPHY {

  /**
   * @class AntennaProfile
   *
   * @brief defines antenna profile info.
   *
   */
  class AntennaProfile
  {
    public: 

    /**
     *
     * @brief default constructor
     *
     */
     AntennaProfile();

    /**
     *
     * @brief parameter constructor
     *
     * @param u16ProfileId the antenna profile id
     * @param fAzimuth the initial antenna azimuth
     * @param fElevation the initial antenna elevation
     *
     */
     AntennaProfile(const ACE_UINT16 u16ProfileId, const float fAzimuth, const float fElevation);

    /**
     *
     * @return returns the current antenna profile id
     *
     */
     ACE_UINT16 getAntennaProfileId() const;

    /**
     *
     * @return returns the antenna azimuth in degrees
     *
     */
     float getAzimuthDegrees() const;


    /**
     *
     * @return returns the antenna elevation in degrees
     *
     */
     float getElevationDegrees() const;

    /**
     *
     * @param u16ProfileId  sets the current antenna profile id
     *
     */
     void setAntennaProfileId(ACE_UINT16 u16ProfileId);

    /**
     *
     * @param fValue sets the antenna azimuth in degrees
     *
     */
     void setAzimuthDegrees(float fValue);

    /**
     *
     * @param fValue sets the antenna elevation in degrees
     *
     */
     void setElevationDegrees(float fValue);

    /**
     *
     * @param rhs value to compare
     *
     * @return returns true if equal else returns false
     */
     bool operator == (const AntennaProfile & rhs) const;

    private:
      ACE_UINT16  u16ProfileId_;

      float       fAzimuthDegrees_;

      float       fElevationDegrees_;
   };
}
#endif //UNIVERSALPHY_ANTENNAPROFILE_HEADER_
