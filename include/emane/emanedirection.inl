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




inline EMANE::Direction::Direction() :
  fAzimuthDegrees_(0),
  fElevationDegrees_(0),
  fDistanceMeters_(0)
{ }

inline EMANE::Direction::Direction(const float fAzimuthDegrees, const float fElevationDegrees, const float fDistanceMeters) :
  fAzimuthDegrees_(fAzimuthDegrees),
  fElevationDegrees_(fElevationDegrees),
  fDistanceMeters_(fDistanceMeters)
{ }

inline float EMANE::Direction::getAzimuthDegrees() const
 {
   return fAzimuthDegrees_;
 }

inline float EMANE::Direction::getElevationDegrees() const
 {
   return fElevationDegrees_;
 }


inline float EMANE::Direction::getDistanceMeters() const
 {
   return fDistanceMeters_;
 }

inline std::string EMANE::Direction::format() const
 {
   char fmtBuff[256];
      
   snprintf(fmtBuff, sizeof(fmtBuff), "direction: azimuth %8.6f deg, elevation %8.6f deg, distance %5.4f meters", 
            getAzimuthDegrees(), 
            getElevationDegrees(),
            getDistanceMeters());

   return fmtBuff;
 }
