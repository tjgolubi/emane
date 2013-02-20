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

#ifndef UNIVERSALPHY_PATHLOSSMANAGER_HEADER_
#define UNIVERSALPHY_PATHLOSSMANAGER_HEADER_

#include "pathlossentry.h"

#include "emane/emanetypes.h"
#include "emane/emanepositiongeodetic.h"
#include "emane/emanevelocityvector.h"
#include "emane/emaneplatformserviceprovider.h"
#include "emaneevents/pathlossevent.h"
#include "emaneevents/locationevent.h"

#include <set>

namespace UniversalPHY {

    // spped of light in meters per second
    const float SOL_MPS = 299792458.0;

/**
 *
 * @class  PathLossManager 
 *
 * @brief Manages pathloss for each NEM based on position or pathloss
 *
 */
 class PathLossManager
  {
    public:
        /**
         *
         * @enum PathLossMode
         *
         * @brief path loss mode
         *
         */
      enum PathLossMode
        {
          PATH_LOSS_MODE_NONE         = 0,  /**< path loss mode none       */
          PATH_LOSS_MODE_FREE_SPACE   = 1,  /**< path loss mode free space */
          PATH_LOSS_MODE_TWO_RAY      = 2,  /**< path loss mode two ray    */
          PATH_LOSS_MODE_PATH_LOSS    = 3,  /**< path loss mode path loss (pre calculated) */
        };

    /**
     *
     * @class  PathLoss
     *
     * @brief Defines the path loss value and validity fetched for an NEM
     *
     */
      class PathLoss {
       public: 
        /**
         *
         * @brief default constructor
         *
         */
        PathLoss() :
          fPathLossdBm_(0),
          fDistanceMeters_(-1),
          bValid_(false)
        { }

        /**
         *
         * @brief parameter constructor
         *
         * @param fValue the path loss value
         * @param fDistanceMeters distance in meters
         *
         */
        PathLoss(const float fValue, 
                 const float fDistanceMeters) :
          fPathLossdBm_(fValue),
          fDistanceMeters_(fDistanceMeters),
          bValid_(true)
        { }

        /**
         *
         * @brief parameter constructor
         *
         * @param fValue the path loss value
         * @param fDistanceMeters distance in meters
         * @param localPosition local position
         * @param remotePosition remote position
         * @param localVelocity local velocity
         * @param remoteVelocity remote velocity
         *
         */
        PathLoss(const float fValue, 
                 const float fDistanceMeters, 
                 const EMANE::PositionGeodetic & localPosition, 
                 const EMANE::PositionGeodetic & remotePosition,
                 const EMANE::VelocityVector & localVelocity, 
                 const EMANE::VelocityVector & remoteVelocity) :
          fPathLossdBm_(fValue),
          fDistanceMeters_(fDistanceMeters),
          bValid_(true),
          localPosition_(localPosition),
          remotePosition_(remotePosition),
          localVelocity_(localVelocity),
          remoteVelocity_(remoteVelocity)
        { }

        /**
         *
         * @return returns the local position
         *
         */
        const EMANE::PositionGeodetic & getLocalPosition () const
         {
           return localPosition_;
         }

        /**
         *
         * @return returns the local velocity
         *
         */
        const EMANE::VelocityVector & getLocalVelocity () const
         {
           return localVelocity_;
         }


        /**
         *
         * @return returns the remote velocity
         *
         */
        const EMANE::VelocityVector & getRemoteVelocity () const
         {
           return remoteVelocity_;
         }

        /**
         *
         * @return returns the remote position
         *
         */
        const EMANE::PositionGeodetic & getRemotePosition () const
         {
           return remotePosition_;
         }

        /**
         *
         * @return returns the path loss in dBm
         *
         */
        float getPathLoss () const
         {
           return fPathLossdBm_;
         }

        /**
         *
         * @return returns the distance in meters
         *
         */
        float getDistanceMeters () const
         {
           return fDistanceMeters_;
         }

        /**
         *
         * @return returns the propagation delay based on distance if available
         *
         */
        ACE_Time_Value getPropagationDelay() const
         { 
           ACE_Time_Value tv = ACE_Time_Value::zero;

           if(fDistanceMeters_ > 0.0)
            {
              tv.set(fDistanceMeters_  / SOL_MPS);
            }

           return tv;
         }

        /**
         *
         * @return returns true if the path loss is valid else returns false
         *
         */
        bool isValid() const
         {
            return bValid_;
         }

       private:
        float  fPathLossdBm_;
        float  fDistanceMeters_;
      
        bool    bValid_;

        EMANE::PositionGeodetic   localPosition_;  
        EMANE::PositionGeodetic   remotePosition_;  
        EMANE::VelocityVector     localVelocity_;  
        EMANE::VelocityVector     remoteVelocity_;  
      };

      /**
       * @struct PathLossStatus
       *
       * @brief path loss and associated frequency
       *
       */
      struct PathLossStatus {
         float         fPathLoss_;        // the path loss value to remote NEM
         ACE_UINT64    u64FrequencyHz_;   // the frequency for distance based pathloss 

         /**
          *
          * @brief  default constructor
          *
          */
          PathLossStatus () :
           fPathLoss_(0),
           u64FrequencyHz_(0)
          { }

         /**
          *
          * @brief  parameter constructor
          *
          * @param fPathLoss the path loss value
          * @param u64freq the frequency associate with the path loss when based on distance
          *
          */
          PathLossStatus (const float fPathLoss, const ACE_UINT64 u64freq) :
           fPathLoss_(fPathLoss),
           u64FrequencyHz_(u64freq)
          { }
      };

      /**
       * @struct PositionStatus
       *
       * @brief position status attributes
       *
       */
      struct PositionStatus {
         EMANE::PositionGeodetic position_;            // position 
         EMANE::VelocityVector   velocity_;            // velocity
         float                   fDistanceMeters_;     // distance
         bool                    bDirty_;              // position/distance changed

         /**
          *
          * @brief  default constructor
          *
          */
         PositionStatus () :
           fDistanceMeters_(0),
           bDirty_(true)
         { }

         /**
          *
          * @brief  parameter constructor
          *
          * @param position  position info
          * @param velocity  velocity info
          *
          */
         PositionStatus (const EMANE::PositionGeodetic & position, const EMANE::VelocityVector & velocity) :
           position_(position),
           velocity_(velocity),
           fDistanceMeters_(0),
           bDirty_(true)
         { }
      };

      /**
       * @struct DistanceInfo
       *
       * @brief distance status
       *
       */
      struct DistanceInfo {
         float    fDistanceMeters_;   // distance to remote NEM
         bool     bDirty_;            // distance changed

        /**
         *
         * @brief  default constructor
         *
         */
         DistanceInfo () :
           fDistanceMeters_(0),
           bDirty_(true)
         { }

        /**
         *
         * @brief  parameter constructor
         *
         * @param fDistance distance to remote NEM
         * @param bDirty position dirty flag
         */
         DistanceInfo (const float fDistance, const bool bDirty) :
           fDistanceMeters_(fDistance),
           bDirty_(bDirty)
         { }
      };

      /**
       * @struct PathLossPosition
       *
       * @brief path loss status pathloss/position
       *
       */
      struct PathLossPosition {
        PathLossStatus pathLossStatus_;    // path loss status
        PositionStatus positionStatus_;    // position status

        /**
         *
         * @brief  default constructor
         *
         */
        PathLossPosition ();

        /**
         *
         * @brief  parameter constructor
         *
         * @param positionStatus position status of remote NEM
         */

        PathLossPosition (const PositionStatus & positionStatus) :
          positionStatus_(positionStatus)
        { }


        /**
         * @brief update position/velocity
         *
         * @param position      const reference new position info 
         * @param velocity      const reference new velocity info 
         *
         * @return returns true if changed, else returns false 
         *
         */

        bool update(const EMANE::PositionGeodetic &position, const EMANE::VelocityVector & velocity)
         {
           bool bPositionUpdate = positionStatus_.position_.updatePosition(position);
           bool bVelocityUpdate = positionStatus_.velocity_.updateVelocity(velocity);

           return bPositionUpdate || bVelocityUpdate;
         }
      };

      // mapping of NEM to path loss position
      typedef std::map<EMANE::NEMId, PathLossPosition>  PathLossPositionMap;
      typedef PathLossPositionMap::iterator             PathLossPositionMapIter;
      typedef PathLossPositionMap::const_iterator       PathLossPositionMapConstIter;
      typedef std::pair <PathLossPositionMapIter, bool> PathLossPositionMapInsertResult;


      /**
       * @brief constructor
       *
       * @param id the NEM id of this instance
       * @param pPlatformService pointer to the platform servicd
       *
       */
      PathLossManager (EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService);

      ~PathLossManager ();

      /**
       * @param src              the nem to that we received a msg from
       * @param u64FrequencyHz   the frequency that was used (if applicable, not used in pathloss mode).
       *
       * @return returns the path loss status for a given src NEM and frequency
       *
       */
      PathLoss getPathLoss (EMANE::NEMId src, ACE_UINT64 u64FrequencyHz);

      /**
       *  
       * @param pathLossMode the path loss mode
       *
       */
      void setPathLossMode (PathLossMode  pathLossMode);

      /**
       *  
       * @param bDefaultConnectivity the default connectivity mode
       *
       */
      void setDefaultConnectivity (bool bDefaultConnectivity);

      /**
       *  
       * @return the default connectivity mode
       *
       */
      bool getDefaultConnectivity () const;

      /**
       *  
       * @param eventId    the event id
       * @param eventState the event state
       *
       */
      void handleEvent(const EMANE::EventId & eventId, const EMANE::EventObjectState & eventState);

   private:
     EMANE::NEMId id_;

     EMANE::PlatformServiceProvider * pPlatformService_;

     PathLossMode pathLossMode_;

     bool bDefaultConnectivity_;

     // path loss based on path loss events
     PathLossEntry::PathLossEntryMap pathLossPathLossMap_;

     // position status
     PathLossPositionMap pathLossPositionMap_;

     void load(const PathlossEvent & event);

     void load(const LocationEvent & event);

     PathLoss getPathLossPathLoss (EMANE::NEMId src);

     PathLoss getPathLossDistance (EMANE::NEMId src, ACE_UINT64 u64FrequencyHz = 0);

     DistanceInfo getDistanceInfo(const UniversalPHY::PathLossManager::PathLossPositionMapIter & local,  
                                  const UniversalPHY::PathLossManager::PathLossPositionMapIter & remote);

     float getPathLossByPosition(const EMANE::PositionGeodetic &p1, const EMANE::PositionGeodetic &p2, 
                                 ACE_UINT64 u64FrequencyHz, float fDistance) const;
 };
}

#endif //UNIVERSALPHY_PATHLOSSMANAGER_HEADER_
