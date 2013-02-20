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

#ifndef EMANE_DIRECTION_HEADER_
#define EMANE_DIRECTION_HEADER_

#include <string>
#include <ace/OS_NS_stdio.h>

namespace EMANE {

  /**
   *
   * @brief  Direction defines the direction of an NEM.
   *
   */
  class Direction {
     public: 

      Direction();
     /**
      *
      * @brief  Direction initializer.
      *
      * @param  fAzimuthDegrees    azimuth in degrees
      * @param  fElevationDegrees  elevation in degrees
      * @param  fDistanceMeters    distance in meters
      *
      */
      Direction(const float fAzimuthDegrees, const float fElevationDegrees, const float fDistanceMeters);

     /**
      *
      * @brief  get Direction azimuth in degrees
      *
      * @return  Direction azimuth in degrees
      *
      */
      float getAzimuthDegrees() const;


     /**
      *
      * @brief  get Direction elevation in degrees
      *
      * @return Direction elevation in degrees
      *
      */
      float getElevationDegrees() const;


     /**
      *
      * @brief  get distance in meters
      *
      * @return Distance in meters
      *
      */
      float getDistanceMeters() const;

      /**
       *
       * @brief  returns a formated string of paramters and values for logging
       *
       */
      std::string format() const;

     private:     
      float   fAzimuthDegrees_;
      float   fElevationDegrees_;
      float   fDistanceMeters_;
  };

#include "emanedirection.inl"

}

#endif //EMANE_DIRECTION_HEADER_
