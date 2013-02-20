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

#include <math.h>

#include "pathlossmanager.h"
#include "emane/emaneconstants.h"
#include "emaneutils/positionutils.h"



#define PathLossInvalid UniversalPHY::PathLossManager::PathLoss ()

namespace 
{
 const float FSPL_CONST = 41.916900439033640;
}


UniversalPHY::PathLossManager::PathLossManager (EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService) : 
  id_(id),
  pPlatformService_(pPlatformService),
  pathLossMode_(PathLossManager::PATH_LOSS_MODE_NONE),
  bDefaultConnectivity_(false)
{ }


UniversalPHY::PathLossManager::~PathLossManager()
{ }



UniversalPHY::PathLossManager::PathLoss 
UniversalPHY::PathLossManager::getPathLoss (EMANE::NEMId src, ACE_UINT64 u64FrequencyHz)
{
  // path loss mode
  switch(pathLossMode_)
   {
     // path loss 
     case::UniversalPHY::PathLossManager::PATH_LOSS_MODE_PATH_LOSS:
      {
         // get path loss from path loss
         return getPathLossPathLoss(src);
      }

     // two ray
     case::UniversalPHY::PathLossManager::PATH_LOSS_MODE_TWO_RAY:
      {
         // get path loss from location two ray
         return getPathLossDistance(src);
      }

     // free space
     case::UniversalPHY::PathLossManager::PATH_LOSS_MODE_FREE_SPACE:
      {
         // get path loss from location free space
         return getPathLossDistance(src, u64FrequencyHz);
      }

     default:
      {
         // invalid path loss
         return PathLossInvalid;
      }
   }
}



void 
UniversalPHY::PathLossManager::handleEvent(const EMANE::EventId & eventId, const EMANE::EventObjectState & eventState)
{
  switch(eventId)
   {
    case PathlossEvent::EVENT_ID:
      {
         // load event
         load(PathlossEvent(eventState));

         // check current path loss mode is pathloss
         if(pathLossMode_ == PATH_LOSS_MODE_PATH_LOSS)
          {
            if(bDefaultConnectivity_ == true)
             {
#ifdef VERBOSE_LOGGING
               pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: disable path loss default connectivity mode", 
                                      id_, __func__);
#endif

               // disable default connectivity
               bDefaultConnectivity_ = false;
             }
          }
      }
    break;

    case LocationEvent::EVENT_ID:
      {
         // load event
         load(LocationEvent(eventState));

         // check current path loss mode is location
         if((pathLossMode_ == PATH_LOSS_MODE_FREE_SPACE) || (pathLossMode_ == PATH_LOSS_MODE_TWO_RAY))
          {
            if(bDefaultConnectivity_ == true)
             {
#ifdef VERBOSE_LOGGING
               pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: disable location default connectivity mode", 
                                      id_, __func__);
#endif

               // disable default connectivity
               bDefaultConnectivity_ = false;
             }
          }
      }
    break;

#ifdef VERBOSE_LOGGING
    default:
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: unknown event id %u, ignore", 
                                      id_, __func__, eventId);
#endif
   }
}



void 
UniversalPHY::PathLossManager::setPathLossMode (UniversalPHY::PathLossManager::PathLossMode pathLossMode)
{
  pathLossMode_ = pathLossMode;
}



void 
UniversalPHY::PathLossManager::setDefaultConnectivity (bool bDefaultConnectivity)
{
  bDefaultConnectivity_ = bDefaultConnectivity;
}


bool
UniversalPHY::PathLossManager::getDefaultConnectivity () const
{
  return bDefaultConnectivity_;
}


UniversalPHY::PathLossManager::PathLoss 
UniversalPHY::PathLossManager::getPathLossPathLoss (EMANE::NEMId src)
{
  // lookup path loss based on path loss events
  const UniversalPHY::PathLossEntry::PathLossEntryMapIter pathLossIter = pathLossPathLossMap_.find(src);

  // entry found
  if(pathLossIter != pathLossPathLossMap_.end())
   {
     // get path loss from path loss entry event
     float fPathLoss = pathLossIter->second.getPathLoss();

     // get local position (us)
     const UniversalPHY::PathLossManager::PathLossPositionMapIter local = pathLossPositionMap_.find(id_);

     // get remote position (them)
     const UniversalPHY::PathLossManager::PathLossPositionMapIter remote = pathLossPositionMap_.find(src); 

     // check for position info
     if(local == pathLossPositionMap_.end())
      {
#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: no position info found for local nem %hu", 
                               id_, __func__, id_);
#endif

        // return a valid path loss entry, no distance info avaliable
        return UniversalPHY::PathLossManager::PathLoss(fPathLoss, -1);
      }
     else if (remote == pathLossPositionMap_.end())
      {
#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: no position info found for remote nem %hu", 
                               id_, __func__, src);
#endif

        // return a valid path loss entry, no distance info avaliable
        return UniversalPHY::PathLossManager::PathLoss(fPathLoss, -1);
      }
     else
      {
        // get distance info
        const UniversalPHY::PathLossManager::DistanceInfo distanceInfo = getDistanceInfo(local, remote);

        // return path loss data
        return UniversalPHY::PathLossManager::PathLoss (fPathLoss, 
                                                        distanceInfo.fDistanceMeters_, 
                                                        local->second.positionStatus_.position_,  
                                                        remote->second.positionStatus_.position_,
                                                        local->second.positionStatus_.velocity_,  
                                                        remote->second.positionStatus_.velocity_);
      }
   }
  else
   {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: path loss entry not found for %hu", 
                             id_, __func__, src);
#endif

      return PathLossInvalid;
   }
}


UniversalPHY::PathLossManager::PathLoss 
UniversalPHY::PathLossManager::getPathLossDistance (EMANE::NEMId src, ACE_UINT64 u64FrequencyHz)
{
  // get local position (us)
  const UniversalPHY::PathLossManager::PathLossPositionMapIter local = pathLossPositionMap_.find(id_);

  // get remote position (them)
  const UniversalPHY::PathLossManager::PathLossPositionMapIter remote = pathLossPositionMap_.find(src); 

  // check for position info
  if(local == pathLossPositionMap_.end())
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: no position info found for nem %hu", 
                            id_, __func__, id_);
#endif

     // return invalid path loss
     return PathLossInvalid;
   }
  else if (remote == pathLossPositionMap_.end())
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: no position info found for nem %hu", 
                            id_, __func__, src);
#endif

     // return invalid path loss
     return PathLossInvalid;
   }
  else
   {
     // get distance info
     const UniversalPHY::PathLossManager::DistanceInfo distanceInfo = getDistanceInfo(local, remote);

     // path loss
     float fPathLoss;

     // if pathloss is invalid based on position or freq, need to recalculate
     if((distanceInfo.bDirty_ == true) || (remote->second.pathLossStatus_.u64FrequencyHz_ != u64FrequencyHz))

      {
        // get path loss baseed on position, freq, dist
        fPathLoss = getPathLossByPosition(local->second.positionStatus_.position_, 
                                          remote->second.positionStatus_.position_, 
                                          u64FrequencyHz, 
                                          distanceInfo.fDistanceMeters_);


#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: recalculated path loss %5.4f dB for nem %hu", 
                               id_, __func__, fPathLoss, src);
#endif

        // update path loss cache
        remote->second.pathLossStatus_ = UniversalPHY::PathLossManager::PathLossStatus(fPathLoss, u64FrequencyHz);
      }
     else
      {
        // use cached value for path loss
        fPathLoss = remote->second.pathLossStatus_.fPathLoss_;

#ifdef VERBOSE_LOGGING
        pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: use cached path loss %5.4f dB for nem %hu", 
                                id_, __func__, fPathLoss, src);
#endif
      }

     // return path loss data
     return UniversalPHY::PathLossManager::PathLoss (fPathLoss, 
                                                     distanceInfo.fDistanceMeters_, 
                                                     local->second.positionStatus_.position_,  
                                                     remote->second.positionStatus_.position_,
                                                     local->second.positionStatus_.velocity_,  
                                                     remote->second.positionStatus_.velocity_);
   }
}




void 
UniversalPHY::PathLossManager::load(const PathlossEvent & event)
{
  // path loss entries
  const PathlossEvent::PathlossEntry *e = event.getEntries();

  // for each path loss entry
  for(size_t i = 0; i < event.getNumberOfEntries(); ++i)
    {
      // get fwd path loss value
      const float fPathLoss = e[i].i32PathLossDBScaled_ / PathlossEvent::PATHLOSSSCALE;

      // insert entry
      std::pair<UniversalPHY::PathLossEntry::PathLossEntryMap::iterator, bool> result = 
          pathLossPathLossMap_.insert(std::make_pair(e[i].u16TxNode_, PathLossEntry(fPathLoss)));

      // insert failed, duplicate entry
      if(result.second == false)
        {
          // entry updated, update path loss value
          if(result.first->second.updatePathLoss(fPathLoss) == true)
            {
#ifdef VERBOSE_LOGGING
              pPlatformService_->log(EMANE::DEBUG_LEVEL,
                                     "PHYI %03hu PathLossManager::%s: updated %03hu pathloss: %5.4f dB",
                                     id_, __func__, 
                                     e[i].u16TxNode_,
                                     fPathLoss);
#endif
            }
          // same value
          else
            {
#ifdef VERBOSE_LOGGING
              pPlatformService_->log(EMANE::DEBUG_LEVEL,
                                     "PHYI %03hu PathLossManager::%s: no change %03hu pathloss: %5.4f dB",
                                     id_, __func__, 
                                     e[i].u16TxNode_,
                                     fPathLoss);
#endif
            }
        }
      else
        {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log(EMANE::DEBUG_LEVEL,
                                 "PHYI %03hu PathLossManager::%s: added %03hu pathloss: %5.4f dB",
                                 id_, __func__, 
                                 e[i].u16TxNode_,
                                 fPathLoss);
#endif
        }
    }
}




void 
UniversalPHY::PathLossManager::load(const LocationEvent & event)
{
  // location entries
  const LocationEvent::LocationEntry *e = event.getEntries();

  // for each location entry
  for(size_t i = 0; i < event.getNumberOfEntries(); ++i)
   {
      const float fLatDegrees   = e[i].i32LatitudeMARCS_  / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      const float fLonDegrees   = e[i].i32LongitudeMARCS_ / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      const float fYawDegrees   = e[i].i32YawMARCS_       / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      const float fPitchDegrees = e[i].i32PitchMARCS_     / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      const float fRollDegrees  = e[i].i32RollMARCS_      / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      const float fAltMeters    = e[i].i32AltitudeMeters_;

      const float fVelAzimuth   = e[i].i32VelocityAzimuthMARCS_     / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      const float fVelElevation = e[i].i32VelocityElevationMARCS_   / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      const float fVelMagnitude = e[i].u32VelocityCMPS_ / 100.0;


      // create a position entry 
      const EMANE::PositionGeodetic position (fLatDegrees,     // lat
                                              fLonDegrees,     // lon
                                              fAltMeters,      // alt
                                              fYawDegrees,     // yaw
                                              fPitchDegrees,   // pitch
                                              fRollDegrees);   // roll

      // create a velocity entry 
      const EMANE::VelocityVector velocity (fVelAzimuth,       // velocity az
                                            fVelElevation,     // velocity al
                                            fVelMagnitude);    // velocity magnitude meters per second

      // attempt to insert position/velocity
      PathLossPositionMapInsertResult result = 
            pathLossPositionMap_.insert(std::make_pair(e[i].u16Node_, 
                                                       UniversalPHY::PathLossManager::PathLossPosition(
                                                          PositionStatus(position, velocity))));

      // insert failed, indicates entry already exists
      if(result.second == false)
        {
          // existing entry updated, path loss value is invalid based on position/orientation/velocity change
          if(result.first->second.update(position, velocity) == true)
            {
#ifdef VERBOSE_LOGGING
              pPlatformService_->log(EMANE::DEBUG_LEVEL,
                                     "PHYI %03hu PathLossManager::%s: updated %03hu, %s %s", 
                                     id_, __func__, 
                                     e[i].u16Node_, 
                                     result.first->second.positionStatus_.position_.format().c_str(),
                                     result.first->second.positionStatus_.velocity_.format().c_str());
#endif

              // if the entry is this NEM then all other entries are considered invalid
              if(e[i].u16Node_ == id_)
               {
                  for(UniversalPHY::PathLossManager::PathLossPositionMapIter iter = pathLossPositionMap_.begin();
                      iter != pathLossPositionMap_.end(); ++iter)
                   {
                     // set position dirty flag
                     iter->second.positionStatus_.bDirty_ = true;
                   }
               }
            }
          // same value
          else
            {
#ifdef VERBOSE_LOGGING
              pPlatformService_->log(EMANE::DEBUG_LEVEL,
                                     "PHYI %03hu PathLossManager::%s: no change %03hu, %s %s", 
                                     id_, __func__, 
                                     e[i].u16Node_, 
                                     result.first->second.positionStatus_.position_.format().c_str(),
                                     result.first->second.positionStatus_.velocity_.format().c_str());
#endif
            }
        }
      else
        {
#ifdef VERBOSE_LOGGING
           pPlatformService_->log(EMANE::DEBUG_LEVEL,
                                  "PHYI %03hu PathLossManager::%s: added %03hu, %s %s", 
                                   id_, __func__, 
                                   e[i].u16Node_, 
                                   result.first->second.positionStatus_.position_.format().c_str(),
                                   result.first->second.positionStatus_.velocity_.format().c_str());
#endif
        }
    }
}





UniversalPHY::PathLossManager::DistanceInfo
UniversalPHY::PathLossManager::getDistanceInfo(const UniversalPHY::PathLossManager::PathLossPositionMapIter & local,  
                                               const UniversalPHY::PathLossManager::PathLossPositionMapIter & remote)
{
   UniversalPHY::PathLossManager::DistanceInfo result;

   // check if positions have changed
   // when our position changes all entries are set to dirty also
   if((result.bDirty_ = remote->second.positionStatus_.bDirty_) == true)
    {
      // get new distance
      result.fDistanceMeters_ = EMANEUtils::getDistance(local->second.positionStatus_.position_, 
                                                        remote->second.positionStatus_.position_);

      // reset distance cached value
      remote->second.positionStatus_.fDistanceMeters_ = result.fDistanceMeters_;

      // unset dirty flag
      remote->second.positionStatus_.bDirty_ = false;

#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: recalculated distance %5.4f meters", 
                             id_, __func__, result.fDistanceMeters_);
#endif
    }
   else
    {
      // use cached value for distance
      result.fDistanceMeters_ = remote->second.positionStatus_.fDistanceMeters_;

#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: use cached distance %5.4f meters", 
                             id_, __func__, result.fDistanceMeters_);
#endif
    }

   return result;
}


float
UniversalPHY::PathLossManager::getPathLossByPosition(const EMANE::PositionGeodetic &p1, 
                                                     const EMANE::PositionGeodetic &p2, 
                                                     ACE_UINT64 u64FrequencyHz, float fDistance) const
{
   float fPathLoss = 0.0;

   if(fDistance > 0.0)
    {
      // free space mode
      if(pathLossMode_ == UniversalPHY::PathLossManager::PATH_LOSS_MODE_FREE_SPACE)
       {
         // calculate path loss freq in MHz, distance in Km
         fPathLoss = 20.0 * log10(FSPL_CONST * (u64FrequencyHz / 1.0e6) * (fDistance / 1.0e3));

#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: free space mode distance %5.4lf, pathloss %5.4f dB", 
                                id_, __func__, fDistance, fPathLoss);
#endif
       }
      // two ray mode
      else if(pathLossMode_ == UniversalPHY::PathLossManager::PATH_LOSS_MODE_TWO_RAY)
       {
         // calculate path loss (distance in meters)
         fPathLoss = (40.0 * log10(fDistance)) - (20.0 * (log10(p1.getAltitudeMeters()) + log10(p2.getAltitudeMeters())));

#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: 2ray mode distance %5.4lf, pathloss %5.4f dB", 
                                id_, __func__, fDistance, fPathLoss);
#endif
       }
     else
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: no loss mode, distance %5.4lf, pathloss %5.4f dB", 
                             id_, __func__, fDistance, fPathLoss);
#endif
      }
    }
   else
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu PathLossManager::%s: distance is 0, pathloss %5.4f dB", 
                              id_, __func__, fPathLoss);
#endif
    }

   return fPathLoss;
}



