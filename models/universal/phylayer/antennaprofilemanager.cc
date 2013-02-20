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


#include "emane/emaneconstants.h"
#include "emaneutils/positionutils.h"

#include "antennaprofilemanager.h"

#include <ace/OS_NS_stdio.h>

#include <cmath>
#include <sstream>


namespace {
 const char * MODULE = "AntennaProfileManager";

 const xmlChar * toXmlChar(const char * s)
  { 
    return reinterpret_cast<const xmlChar*> (s); 
  }
}



UniversalPHY::AntennaProfileManager::AntennaProfileManager(const EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService) :
   id_(id),
   pPlatformService_(pPlatformService),
   fixedGaindBi_(0.0),
   antennaMode_(EMANE::ANTENNA_MODE_NONE)
{ }



void UniversalPHY::AntennaProfileManager::setFixedAntennaGaindBi(const float fValue)
{
   if(fixedGaindBi_ != fValue)
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s change from %5.4f to %5.4f", 
                             id_, __func__, MODULE, fixedGaindBi_, fValue);
#endif
      fixedGaindBi_ = fValue;
    }
}


void UniversalPHY::AntennaProfileManager::setAntennaMode(const EMANE::ANTENNA_MODE mode)
{
   if(antennaMode_ != mode)
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s change from %d to %d", 
                             id_, __func__, MODULE, antennaMode_, mode);
#endif
      antennaMode_ = mode;
    }
}


EMANE::ANTENNA_MODE UniversalPHY::AntennaProfileManager::getAntennaMode() const
{
  return antennaMode_;
}


float UniversalPHY::AntennaProfileManager::getFixedGaindBi() const
{
  return fixedGaindBi_;
}



ACE_UINT16 UniversalPHY::AntennaProfileManager::getAntennaProfileId() const
{
  return localAntennaProfile_.getAntennaProfileId();
}



float UniversalPHY::AntennaProfileManager::getGaindBi(const EMANE::CommonPHYHeader & phyHeader, 
                                                       const AntennaProfile & remoteProfile,
                                                       const EMANE::PositionGeodetic & localPosition,
                                                       const EMANE::PositionGeodetic & remotePosition,
                                                       const EMANE::VelocityVector & localVelocity,
                                                       const EMANE::VelocityVector & remoteVelocity,
                                                       const float fLocalAntennaAzimuthDegrees,
                                                       const float fLocalAntennaElevationDegrees,
                                                       const bool bHaveValidPositions) const
{
  // remote and local antenna gain mode are fixed, so use fixed values, thats all folks
  if((phyHeader.getAntennaMode() == EMANE::ANTENNA_MODE_FIXED) && (antennaMode_ == EMANE::ANTENNA_MODE_FIXED))
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s both NEM's using fixed gain remote %5.4lf dBi, local %5.4lf dBi", 
                            id_, __func__, MODULE, phyHeader.getAntennaGaindBi(), getFixedGaindBi());
#endif

     // use fixed gain for both NEM's
     return phyHeader.getAntennaGaindBi() + getFixedGaindBi();
   } 
  // check if valid positions are avaliable
  else if(bHaveValidPositions == false)
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s complete position info not available, using minimal gain %5.3lf dBi", 
                            id_, __func__, MODULE, -EMANE::FULL_PATH_LOSS_DB);
#endif

     // unknown position(s) use minimal gain
     return -EMANE::FULL_PATH_LOSS_DB;
   }
  else
   {
     // local gain
     float fLocalGaindBi = 0;

     // local blockage
     float fLocalBlockagedBi = 0;

     // remote gain
     float fRemoteGaindBi = 0;

     // remote blockage
     float fRemoteBlockagedBi = 0;

     // the distance from fwd/rev direction
     float fDistanceMeters = 0;

     // remote placement
     EMANE::PositionNEU remoteAntennaPlacement;

     // adjust local position due to velocity
     const EMANE::PositionGeodetic adjustedLocalPosition = adjustPosition(localPosition, localVelocity);

     // adjust remote position due to velocity
     const EMANE::PositionGeodetic adjustedRemotePosition = adjustPosition(remotePosition, remoteVelocity);

     // remote antenna gain mode
     if(phyHeader.getAntennaMode() == EMANE::ANTENNA_MODE_FIXED)
      {
#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s remote NEM uses fixed gain %5.4f dBi", 
                               id_, __func__, MODULE, phyHeader.getAntennaGaindBi());
#endif

        // use fixed gain from phy header
        fRemoteGaindBi = phyHeader.getAntennaGaindBi();
      }
     else
      {
        // get remote nem antenna profile info
        const UniversalPHY::AntennaProfilePattern::AntennaProfilePatternMapConstIter iter = 
              antennaProfilePatterns_.find(remoteProfile.getAntennaProfileId());

        // check if we know about this profile
        if(iter == antennaProfilePatterns_.end())
         {

#ifdef VERBOSE_LOGGING
           pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s remote NEM uses unknown antenna profile id %u, use min gain %5.4f",
                                  id_, __func__, MODULE, 
                                  remoteProfile.getAntennaProfileId(), -EMANE::FULL_PATH_LOSS_DB);
#endif

           // unknown profile use minimal gain
           return -EMANE::FULL_PATH_LOSS_DB;
         }
        else
         {
           // get antenna placement
           remoteAntennaPlacement = iter->second.getAntennaPlacement();

           // get reverse direction (them to us)
           const EMANE::Direction refAngle = EMANEUtils::getDirection(adjustedRemotePosition, 
                                                                      remoteAntennaPlacement, 
                                                                      adjustedLocalPosition, 
                                                                      localAntennaProfile_.getAntennaPlacement());

           // get the actual az el values to us
           const EMANE::PositionGeodetic::AzEl azel = EMANEUtils::getLookupAngles(refAngle.getAzimuthDegrees(),
                                                                                  remoteProfile.getAzimuthDegrees(),
                                                                                  refAngle.getElevationDegrees(),
                                                                                  remoteProfile.getElevationDegrees());

           // get distance
           fDistanceMeters = refAngle.getDistanceMeters();

           // get remote gain
           fRemoteGaindBi = iter->second.getGain(azel.az_, azel.el_);

           // get remote blockage
           fRemoteBlockagedBi = iter->second.getBlockage(refAngle.getAzimuthDegrees(), refAngle.getElevationDegrees());

#ifdef VERBOSE_LOGGING
           pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s remote NEM uses antenna profile id %hu, %s, %s, gain/blockage %5.4f/%5.4f dBi",
                                  id_, __func__, MODULE, 
                                  remoteProfile.getAntennaProfileId(), 
                                  refAngle.format().c_str(), 
                                  azel.format().c_str(), 
                                  fRemoteGaindBi,
                                  fRemoteBlockagedBi);
#endif
         }
      }

     // local antenna gain mode
     if(antennaMode_ == EMANE::ANTENNA_MODE_FIXED)
      {
        // use of configured fixed gain
        fLocalGaindBi = getFixedGaindBi();

#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s local NEM uses fixed gain %5.4f dBi", 
                               id_, __func__, MODULE, fLocalGaindBi);
#endif
      } 
     else
      {
        // get forward direction (us to them)
        const EMANE::Direction refAngle = EMANEUtils::getDirection(adjustedLocalPosition, 
                                                                   localAntennaProfile_.getAntennaPlacement(),
                                                                   adjustedRemotePosition, 
                                                                   remoteAntennaPlacement);

        // get the actual az el values to them
        const EMANE::PositionGeodetic::AzEl azel = EMANEUtils::getLookupAngles(refAngle.getAzimuthDegrees(), 
                                                                               fLocalAntennaAzimuthDegrees,
                                                                               refAngle.getElevationDegrees(),
                                                                               fLocalAntennaElevationDegrees);

        // get distance
        fDistanceMeters = refAngle.getDistanceMeters();

        // get local gain
        fLocalGaindBi = localAntennaProfile_.getGain(azel.az_, azel.el_);

        // get local blockage
        fLocalBlockagedBi = localAntennaProfile_.getBlockage(refAngle.getAzimuthDegrees(), refAngle.getElevationDegrees());

#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s local NEM uses antenna profile id %hu, %s, %s, gain/blockage %5.4f/%5.4f dBi",
                               id_, __func__, MODULE, 
                               localAntennaProfile_.getAntennaProfileId(), 
                               refAngle.format().c_str(), 
                               azel.format().c_str(), 
                               fLocalGaindBi,
                               fLocalBlockagedBi);
#endif
      }
 
     // check if antennas are below the horizon
     if (EMANEUtils::checkHorizon(localPosition.getAltitudeMeters()  + localAntennaProfile_.getAntennaPlacement().fUpMeters_, // local antenna Z
                                  remotePosition.getAltitudeMeters() + remoteAntennaPlacement.fUpMeters_,                     // remote antenna Z
                                  fDistanceMeters) == false)                                                                  // distance
      {
#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s failed horizon text, using minimal gain %5.3lf dBi", 
                               id_, __func__, MODULE, -EMANE::FULL_PATH_LOSS_DB);
#endif

        // below horizon use minimal gain
        return -EMANE::FULL_PATH_LOSS_DB;
      }
     else
      {
        // return total gain dBi
        return  fRemoteGaindBi + fRemoteBlockagedBi + fLocalGaindBi + fLocalBlockagedBi;
      }
   }
}



std::string UniversalPHY::AntennaProfileManager::format() const
{
   char fmtBuff[256];

   ACE_OS::snprintf(fmtBuff, sizeof(fmtBuff), "antenna profile: %hu, fixed gain %5.4f, mode %s",
                    getAntennaProfileId(),
                    getFixedGaindBi(),
                    EMANE::antennaModeToString(getAntennaMode()));

   return fmtBuff;
}



void 
UniversalPHY::AntennaProfileManager::configure(const bool bEnableAntennaProfile, 
                                               const std::string & uri, ACE_UINT16 u16ProfileId)
      throw (EMANE::EMANEException)
{
    // if antenna profile is enabled but the uri is empty, complain 
    if((bEnableAntennaProfile == true) && uri.empty())
      {
         const char * msg = " antennaprofilemanifesturi must be given when antennaprofileenable is on";

         pPlatformService_->log(EMANE::ERROR_LEVEL,"%03hu %s::%s%s", id_, __func__, MODULE, msg);
              
         std::stringstream ssDescription;
         ssDescription << MODULE << msg << std::ends;
         throw EMANE::EMANEException("UniversalPHY::AntennaProfileManager", ssDescription.str());
      }

    // if antenna profile is enabled but the profile id is not set, complain
    if((bEnableAntennaProfile == true) && (u16ProfileId == 0))
      {
         const char * msg = " a valid (non-zero) antennaprofileid must be given when antennaprofileenable is on";

         pPlatformService_->log(EMANE::ERROR_LEVEL,"%03hu %s::%s%s", id_, __func__, MODULE, msg);
              
         std::stringstream ssDescription;
         ssDescription << MODULE << msg << std::ends;
         throw EMANE::EMANEException("UniversalPHY::AntennaProfileManager", ssDescription.str());
      }

    // if we get a uri, use it 
    if(uri.empty() == false)
      {
        // create an antenna profile parser
        AntennaProfileParser parser(id_, pPlatformService_);

        // get all defined antenna patterns
        antennaProfilePatterns_ = parser.getAntennaProfiles(uri);

        // if profile mode is enabled, check if we can load our profile
        if((bEnableAntennaProfile == true) && (setAntennaProfileId(u16ProfileId) == false))
          {
            const char * msg = " our antennaprofileid was not defined in the antenna profile definition file";

            pPlatformService_->log(EMANE::ERROR_LEVEL,"%03hu %s::%s%s", id_, __func__, MODULE, msg);
              
            std::stringstream ssDescription;
            ssDescription << MODULE << msg << std::ends;
            throw EMANE::EMANEException("UniversalPHY::AntennaProfileManager", ssDescription.str());
          }
       }

     // now remember the profile mode
     if(bEnableAntennaProfile == true)
      {
        // set the antenna pofile to profile mode
        antennaMode_ = EMANE::ANTENNA_MODE_PROFILE;
      }
     else
      {
        // set the antenna profile to fixed mode
        antennaMode_ = EMANE::ANTENNA_MODE_FIXED;
      }
}



bool UniversalPHY::AntennaProfileManager::setAntennaProfileId (const ACE_UINT16 u16ProfileId)
{
   // check our profile id
   if(localAntennaProfile_.getAntennaProfileId() != u16ProfileId)
    {
      // lookup our antenna profile by id
      const UniversalPHY::AntennaProfilePattern::AntennaProfilePatternMapConstIter iter = antennaProfilePatterns_.find(u16ProfileId);

     // save our profile
     if(iter != antennaProfilePatterns_.end())
      {
        localAntennaProfile_ = iter->second;

#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s requested profile id %hu, has been set", 
                               id_, __func__, MODULE, u16ProfileId);
#endif

        return true;
      }
     else
      {
#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s requested profile id %hu, not in existing profile description list, ignore", 
                               id_, __func__, MODULE, u16ProfileId);
#endif

        return false;
      }
   }
  else
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s requested profile id %hu, is already active", 
                               id_, __func__, MODULE, u16ProfileId);
#endif

     return false;
   }
}


bool 
UniversalPHY::AntennaProfileManager::handleAntennaProfileChange(const ACE_UINT16 u16ProfileId)
 {
    return setAntennaProfileId(u16ProfileId);     
 }


EMANE::PositionGeodetic
UniversalPHY::AntennaProfileManager::adjustPosition(const EMANE::PositionGeodetic & position,
                                                    const EMANE::VelocityVector & velocity) const
{

   float yaw = velocity.getAzimuthDegrees() + position.getYawDegrees();

   // set yaw to [0 to 360)
   EMANEUtils::AI_TO_BE_DEGREES(yaw, 0.0, 360.0);

   float pitch = velocity.getElevationDegrees() + position.getPitchDegrees();

   // set dpitch to [0 to 360)
   EMANEUtils::AI_TO_BE_DEGREES(pitch, 0.0, 360.0);

   // set pitch to [-90 to 90]
   EMANEUtils::NEG_90_TO_POS_90_DEGREES(pitch);

   EMANE::PositionGeodetic adjusted(position.getLatitudeDegrees(),   // original lat
                                    position.getLongitudeDegrees(),  // original lon
                                    position.getAltitudeMeters(),    // original alt
                                    yaw,                             // adjusted yaw
                                    pitch,                           // adjusted pitch
                                    position.getRollDegrees());      // original roll

#ifdef VERBOSE_LOGGING
   pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s \n\t input %s\n\t%s\n\t adjusted %s", 
                          id_, __func__, 
                          MODULE, 
                          position.format().c_str(), 
                          velocity.format().c_str(), 
                          adjusted.format().c_str());
#endif

   return adjusted;
}

