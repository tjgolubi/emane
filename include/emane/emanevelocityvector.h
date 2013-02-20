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

#ifndef EMANE_VELOCITYVECTOR_HEADER_
#define EMANE_VELOCITYVECTOR_HEADER_

#include "emane/emanetypes.h"
#include "emaneutils/conversionutils.h"

#include <map>
#include <math.h>
#include <ace/OS_NS_stdio.h>

namespace EMANE {
/**
 * @class VelocityVector
 * 
 * @brief Defines a velocity vector in terms of azimuth, elevation and magnitude
 *
 **/

 class VelocityVector {
   public:

    /**
     * @brief default constructor
     */
    VelocityVector();


    /**
     * @brief parameter constructor
     *
     * @param fAzimuthDegrees   azimuth in degrees
     * @param fElevationDegrees elevation in degrees
     * @param fMagnitudeMPS     magnitude in meters per second
     *
     */
     VelocityVector(float fAzimuthDegrees, float fElevationDegrees, float fMagnitudeMPS);


    /**
     * @brief get azimuth in degrees
     *
     * @return returns azimuth in degrees
     *
     */
     float getAzimuthDegrees() const;

    /**
     * @brief get elevation in degrees
     *
     * @return returns elevation in degrees
     *
     */
     float getElevationDegrees() const;

    /**
     * @brief get magnitude in meters per second
     *
     * @return returns magnitude in meters per second
     *
     */
     float getMagnitudeMPS() const;
 
    /**
     * value equality operator
     *
     * @param rhs value entry to compare against current value
     *
     */
     bool operator == (const VelocityVector &rhs) const;

    /**
     * @brief update velocity info
     *
     * @param velocity      const reference new velocity info 
     *
     * @return returns true if velocity has been changed, false if the velocity has not changed
     *
     */
     bool updateVelocity(const VelocityVector & velocity);

    /**
     *
     * @brief  returns a formated string of paramters and values for logging
     *
     */
    std::string format() const;
 
    /**
     * defines a mapping of NEM's to value entries
     *
     */
     typedef std::map<EMANE::NEMId, VelocityVector> VelocityVectorMap;
     typedef VelocityVectorMap::iterator            VelocityVectorMapIter;
     typedef VelocityVectorMap::const_iterator      VelocityVectorMapConstIter;

   private:
     float fAzimuthDegrees_;
     float fElevationDegrees_;
     float fMagnitudeMPS_;
  };

#include "emanevelocityvector.inl"
}

#endif //EMANE_VELOCITYVECTOR_HEADER_
