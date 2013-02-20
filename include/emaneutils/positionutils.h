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

#ifndef EMANE_POSITIONUTILS_HEADER_
#define EMANE_POSITIONUTILS_HEADER_

#include "emane/emanedirection.h"
#include "emane/emanepositiongeodetic.h"
#include "emane/emanepositionneu.h"



namespace EMANEUtils {

 /**
  *
  * @param positionLocal   local NEM position
  * @param placementLocal  local antenna placement relative to the local position
  * @param positionRemote  remote NEM position
  * @param placementRemote remote antenna placement relative to the remote position
  *
  * @return returns the direction from the local to the remote positions
  *
  */
  EMANE::Direction getDirection(const EMANE::PositionGeodetic & positionLocal, 
                                const EMANE::PositionNEU & placementLocal, 
                                const EMANE::PositionGeodetic & positionRemote, 
                                const EMANE::PositionNEU & placementRemote);


 /**
  *
  * @param  fAzReference the reference azimuth
  * @param  fAzPointing  the pointing azimuth
  * @param  fElReference the reference elevation
  * @param  fElPointing  the pointing elevation
  *
  * @return returns the azimuth and elevation angles
  *
  */
  EMANE::PositionGeodetic::AzEl getLookupAngles(const float fAzReference, 
                                                const float fAzPointing, 
                                                const float fElReference, 
                                                const float fElPointing);

 /**
  *
  * @param  fHeightMeters1 the height of position 1 in meters
  * @param  fHeightMeters2 the height of position 2 in meters
  * @param  fDistanceMeters the distance in meters
  *
  * @return returns true the is positions are above the horizon else returns false
  *
  */

  bool checkHorizon(const float fHeightMeters1, const float fHeightMeters2, const float fDistanceMeters);

 /**
  *
  * @param p1 position 1
  * @param p2 position 2
  *
  * @return returns the distance between the 2 positions
  *
  */
  float getDistance(const EMANE::PositionGeodetic &p1, const EMANE::PositionGeodetic &p2);


 /**
  *
  * @param fDeltaX  position delta X
  * @param fDeltaY  position delta Y
  * @param fDeltaZ  position delta Z
  * @param fLatRadians latitude in radians
  * @param fLonRadians longitude in radians
  *
  * @return returns the position in North East Up
  *
  */
  EMANE::PositionNEU convertECEFtoNEU(const float fDeltaX, const float fDeltaY, const float fDeltaZ, 
                                      const float fLatRadians, const float fLonRadians);
 /**
  *
  * @param rNEU reference to position in North East Up (out param)
  * @param fYawRadians yaw in radians
  * @param fPitchRadians pitch in radians
  * @param fRollRadians roll in radians
  *
  */
  void rotateNEUforYawPitchRoll(EMANE::PositionNEU & rNEU, const float fYawRadians, 
                                const float fPitchRadians, const float fRollRadians);
}

#include "positionutils.inl"

#endif //EMANE_POSITIONUTILS_HEADER_
