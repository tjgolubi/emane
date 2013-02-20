/*
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANE_POSITIONGEODETIC_HEADER_
#define EMANE_POSITIONGEODETIC_HEADER_

#include "emane/emanetypes.h"
#include "emaneutils/conversionutils.h"

#include <map>
#include <math.h>
#include <ace/OS_NS_stdio.h>

namespace EMANE {
/**
 * @class PositionGeodetic
 * 
 * @brief Defines the position of an NEM in (X,Y,Z) coordinates
 *
 **/

 class PositionGeodetic {
   public:

   /**
    * @struct  XYZ
    * 
    * @brief Defines (X,Y,Z) coordinates
    *
    **/
     struct XYZ {
         float x_, y_, z_;

        /**
         *
         * @brief  parameter constructor
         *
         * @param x the x component in meters
         * @param y the y component in meters
         * @param z the z component in meters
         *
         */
         XYZ(float x, float y, float z) :
          x_(x), y_(y), z_(z)
         { }

        /**
         *
         * @brief  default constructor
         *
         */
         XYZ() :
          x_(0), y_(0), z_(0)
         { }

        /**
         *
         * @brief  returns a formated string of paramters and values for logging
         *
         */
         std::string format() const
          {
            char fmtBuff[1024];

            snprintf(fmtBuff, sizeof(fmtBuff), "XYZ: X:%8.6f, Y:%8.6f, Z:%8.6f", x_, y_, z_);

            return fmtBuff;
          }
      };

   /**
    * @struct  AzEl
    * 
    * @brief Defines Azimuth and Elevation coordinates
    *
    **/
     struct AzEl {
        float az_;
        float el_;

        /**
         *
         * @brief  default constructor
         *
         */
        AzEl() :
          az_(0), el_(0)
         { }

        /**
         *
         * @brief  parameter constructor
         *
         * @param az the azimuth in meters
         * @param el the elevation in meters
         *
         */
         AzEl(const float az, const float el) :
          az_(az), el_(el)
         { }

        /**
         *
         * @brief  returns a formated string of paramters and values for logging
         *
         */
         std::string format() const
          {
            char fmtBuff[1024];

            snprintf(fmtBuff, sizeof(fmtBuff), "AzEl: az:%8.6f, el:%8.6f", az_, el_);

            return fmtBuff;
          }
      };


    /**
     * @brief default constructor
     */
    PositionGeodetic();


    /**
     * @brief parameter constructor
     *
     * @param fLatDegrees      latitude in degrees
     * @param fLonDegrees      longitude in degrees
     * @param fAltMeters       altitude in meters
     * @param fYawDegrees      optional yaw in degrees
     * @param fPitchDegrees    optional pitch in degrees
     * @param fRollDegrees     optional roll in degrees
     *
     */
     PositionGeodetic(float fLatDegrees, float fLonDegrees, float fAltMeters,
                      float fYawDegrees = 0, float fPitchDegrees = 0, float fRollDegrees = 0);

    /**
     * @brief update position info
     *
     * @param position      const reference new position info 
     *
     * @return returns true if position has been changed, false if the position has not changed
     *
     */
     bool updatePosition(const PositionGeodetic & position);

    /**
     * @brief get latitude in degrees
     *
     * @return returns latitude in degrees
     *
     */
     float getLatitudeDegrees() const;

    /**
     * @brief get longitude in degrees
     *
     * @return returns longitude in degrees
     *
     */
     float getLongitudeDegrees() const;

    /**
     * @brief get altitude in meters
     *
     * @return returns altitude in meters
     *
     */
     float getAltitudeMeters() const;
 
    /**
     * @brief get latitude in radians
     *
     * @return returns latitude in radians
     *
     */
     float getLatitudeRadians() const;

    /**
     * @brief get longitude in radians
     *
     * @return returns longitude in radians
     *
     */
     float getLongitudeRadians() const;

    /**
     * @brief get yaw in radians
     *
     * @return returns yaw in radians
     *
     */
     float getYawRadians() const;

    /**
     * @brief get pitch in radians
     *
     * @return returns pitch in radians
     *
     */
     float getPitchRadians() const;

     /**
     * @brief get roll in radians
     *
     * @return returns roll in radians
     *
     */
     float getRollRadians() const;

    /**
     * @brief get yaw in degrees
     *
     * @return returns yaw in degrees
     *
     */
     float getYawDegrees() const;

    /**
     * @brief get pitch in degrees
     *
     * @return returns pitch in degrees
     *
     */
     float getPitchDegrees() const;

     /**
     * @brief get roll in degrees
     *
     * @return returns roll in degrees
     *
     */
     float getRollDegrees() const;

    /**
     * @brief get the position in (X,Y,Z) coordiantes
     *
     * @return returns the position in (X,Y,Z) coordiantes
     *
     */
     XYZ getXYZ() const;

    /**
     * position equality operator
     *
     * @param rhs position entry to compare against current position
     *
     */
     bool operator == (const PositionGeodetic &rhs) const;

    /**
     *
     * @brief  returns a formated string of paramters and values for logging
     *
     */
    std::string format() const;
 
    /**
     * defines a mapping of NEM's to position entries
     *
     */
     typedef std::map<EMANE::NEMId, PositionGeodetic> PositionGeodeticMap;
     typedef PositionGeodeticMap::iterator            PositionGeodeticMapIter;
     typedef PositionGeodeticMap::const_iterator      PositionGeodeticMapConstIter;

   private:
     float fLatDegrees_;
     float fLonDegrees_;
     float fAltMeters_;
     float fYawDegrees_;
     float fPitchDegrees_;
     float fRollDegrees_;
    
     void toXYZ();

     XYZ xyz_;
  };

#include "emanepositiongeodetic.inl"
}

#endif //EMANE_POSITIONGEODETIC_HEADER_
