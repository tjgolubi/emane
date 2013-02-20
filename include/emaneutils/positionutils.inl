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




inline EMANE::Direction 
EMANEUtils::getDirection(const EMANE::PositionGeodetic &positionLocal, 
                         const EMANE::PositionNEU      &placementLocal,
                         const EMANE::PositionGeodetic &positionRemote, 
                         const EMANE::PositionNEU      &placementRemote)
{
   // positions in XYZ
   const EMANE::PositionGeodetic::XYZ xyz1 = positionLocal.getXYZ();
   const EMANE::PositionGeodetic::XYZ xyz2 = positionRemote.getXYZ();

   // delta YPR
   const float fDeltaYawRadians   = positionLocal.getYawRadians()   - positionRemote.getYawRadians();
   const float fDeltaPitchRadians = positionLocal.getPitchRadians() - positionRemote.getPitchRadians();
   const float fDeltaRollRadians  = positionLocal.getRollRadians()  - positionRemote.getRollRadians();

   // remote placement
   EMANE::PositionNEU placementRemoteRotated = placementRemote;

   // Rotate ECEF to NEU (North, East, Up)
   EMANE::PositionNEU NEU = convertECEFtoNEU(xyz2.x_ - xyz1.x_,
                                             xyz2.y_ - xyz1.y_,
                                             xyz2.z_ - xyz1.z_,
                                             positionLocal.getLatitudeRadians(), 
                                             positionLocal.getLongitudeRadians());

   // adjusted for yaw, pitch roll
   rotateNEUforYawPitchRoll(NEU, -positionLocal.getYawRadians(), positionLocal.getPitchRadians(), positionLocal.getRollRadians());

   // adjusted for yaw, pitch roll
   rotateNEUforYawPitchRoll(placementRemoteRotated, -fDeltaYawRadians, fDeltaPitchRadians, fDeltaRollRadians);

   // Update NEU to include antenna location on the platform.  This accounts for both local and remote platforms.
   NEU.fNorthMeters_ = NEU.fNorthMeters_ - placementLocal.fNorthMeters_ + placementRemoteRotated.fNorthMeters_;
   NEU.fEastMeters_  = NEU.fEastMeters_  - placementLocal.fEastMeters_  + placementRemoteRotated.fEastMeters_;
   NEU.fUpMeters_    = NEU.fUpMeters_    - placementLocal.fUpMeters_    + placementRemoteRotated.fUpMeters_; 

   // get distance
   const float fDistanceMeters = EMANEUtils::NORMALIZE_VECTOR(NEU.fNorthMeters_, NEU.fEastMeters_, NEU.fUpMeters_);

   // get elevation
   const float fElevationDegrees = asin(NEU.fUpMeters_ / fDistanceMeters) * (180.0 / M_PIl);

   // get azimuth
   float fAzimuthDegrees = 0.0;

   if(NEU.fNorthMeters_ == 0.0)
    {
     if(NEU.fEastMeters_ > 0.0)
      {
        fAzimuthDegrees = 90.0;
      }
     else
      {
        fAzimuthDegrees = 270.0;
      }
    }
   else
    {
     if(NEU.fEastMeters_ == 0.0)
      {
       if(NEU.fNorthMeters_ > 0.0)
        {
          fAzimuthDegrees = 0.0;
        }
       else
        {
          fAzimuthDegrees = 180.0;
        }
      }
     else                  
      {
        fAzimuthDegrees = atan(NEU.fEastMeters_ / NEU.fNorthMeters_) * (180.0 / M_PIl);
      }

     if(NEU.fNorthMeters_ < 0.0)
      {
        fAzimuthDegrees += 180.0;
      }

     if((NEU.fNorthMeters_ > 0.0) && (NEU.fEastMeters_ < 0.0))
      {
        fAzimuthDegrees += 360.0;
      }
    }

   return EMANE::Direction(fAzimuthDegrees, fElevationDegrees, fDistanceMeters);
}


inline EMANE::PositionGeodetic::AzEl
EMANEUtils::getLookupAngles(const float fAzReference, const float fAzPointing, const float fElReference, const float fElPointing)
{
  // Az = Az(reference) – Az(pointing)
  float fAzimuth = fAzReference - fAzPointing;

  // El = El(reference) – El(pointing)
  float fElevation = fElReference - fElPointing;

  // ensure (0.0 <= Az < 360.0)
  EMANEUtils::AI_TO_BE_DEGREES(fAzimuth, 0.0, 360.0);

  // ensure (0.0 <= El < 360.0)
  EMANEUtils::AI_TO_BE_DEGREES(fElevation, 0.0, 360.0);

  // ensure (-90 <= El <= 90)
  EMANEUtils::NEG_90_TO_POS_90_DEGREES(fElevation);

  return EMANE::PositionGeodetic::AzEl(fAzimuth, fElevation);
}


inline bool 
EMANEUtils::checkHorizon(const float fHeightMeters1, const float fHeightMeters2, const float fDistanceMeters)
{
   const float fDH1 = 3570 * sqrt(fHeightMeters1 < 0.0 ? 0.0 : fHeightMeters1);

   const float fDH2 = 3570 * sqrt(fHeightMeters2 < 0.0 ? 0.0 : fHeightMeters2);

   if((fDH1 + fDH2) > fDistanceMeters)
    {
      // pass
      return true;
    }
   else
    { 
      // fail
      return false;
    }
}


inline float
EMANEUtils::getDistance(const EMANE::PositionGeodetic &p1, const EMANE::PositionGeodetic &p2)
{
  if(p1 == p2)
   {
     // co-located
     return 0.0;
   }
  else
   {
     const EMANE::PositionGeodetic::XYZ xyz1 = p1.getXYZ();
     const EMANE::PositionGeodetic::XYZ xyz2 = p2.getXYZ();

     // return distance
     return EMANEUtils::NORMALIZE_VECTOR((xyz2.x_ - xyz1.x_), (xyz2.y_ - xyz1.y_), (xyz2.z_ - xyz1.z_));
   }
}



inline EMANE::PositionNEU
EMANEUtils::convertECEFtoNEU(const float fDeltaX, const float fDeltaY, const float fDeltaZ, const float fLatRadians, const float fLonRadians)
{
   const float N = -fDeltaX * sin(fLatRadians) * cos(fLonRadians) - 
                    fDeltaY * sin(fLatRadians) * sin(fLonRadians) + 
                    fDeltaZ * cos(fLatRadians);

   const float E = -fDeltaX * sin(fLonRadians) + fDeltaY * cos(fLonRadians);      

   const float U =  fDeltaX * cos(fLatRadians) * cos(fLonRadians) + 
                    fDeltaY * cos(fLatRadians) * sin(fLonRadians) + 
                    fDeltaZ * sin(fLatRadians);

   return EMANE::PositionNEU(N, E, U);
}


inline void
EMANEUtils::rotateNEUforYawPitchRoll(EMANE::PositionNEU & rNEU, const float fYawRadians, const float fPitchRadians, const float fRollRadians)
{
   // check if adjustment needed
   if((fYawRadians != 0) || (fPitchRadians != 0) || (fRollRadians != 0))
    {
      const EMANE::PositionNEU NEUOrig = rNEU;

      // Rotate NEU to account for (yaw, pitch and roll)  (order of rotion applied here is yaw, pitch, roll)
      rNEU.fNorthMeters_ = NEUOrig.fNorthMeters_ * cos(fYawRadians) * cos(fPitchRadians) - 
                           NEUOrig.fEastMeters_  * sin(fYawRadians) * cos(fPitchRadians) + 
                           NEUOrig.fUpMeters_    * sin(fPitchRadians);

      rNEU.fEastMeters_ =  NEUOrig.fNorthMeters_ * (cos(fYawRadians)   * sin(fPitchRadians) * sin(fRollRadians) + 
                           sin(fYawRadians)      * cos(fRollRadians))  +
                           NEUOrig.fEastMeters_  * (cos(fYawRadians)   * cos(fRollRadians)  - 
                           sin(fYawRadians)      * sin(fPitchRadians)  * sin(fRollRadians)) -
                           NEUOrig.fUpMeters_    * (cos(fPitchRadians) * sin(fRollRadians));

      rNEU.fUpMeters_   = -NEUOrig.fNorthMeters_ * (cos(fYawRadians)   * sin(fPitchRadians) * cos(fRollRadians) + 
                           sin(fYawRadians)      * sin(fRollRadians))  +
                           NEUOrig.fEastMeters_  * (sin(fYawRadians)   * sin(fPitchRadians) * cos(fRollRadians) + 
                           cos(fYawRadians)      * sin(fRollRadians))  + 
                           NEUOrig.fUpMeters_    * (cos(fPitchRadians) * cos(fRollRadians));

    }
}
