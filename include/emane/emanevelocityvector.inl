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


inline EMANE::VelocityVector::VelocityVector() :
  fAzimuthDegrees_(0),
  fElevationDegrees_(0),
  fMagnitudeMPS_(0)
{ };


inline EMANE::VelocityVector::VelocityVector(float fAzimuthDegrees, 
                                             float fElevationDegrees, 
                                             float fMagnitudeMPS) :
  fAzimuthDegrees_(fAzimuthDegrees),
  fElevationDegrees_(fElevationDegrees),
  fMagnitudeMPS_(fMagnitudeMPS)
{ };


inline float
EMANE::VelocityVector::getAzimuthDegrees() const
{
  return fAzimuthDegrees_;
}


inline float
EMANE::VelocityVector::getElevationDegrees() const
{
  return fElevationDegrees_;
}


inline float
EMANE::VelocityVector::getMagnitudeMPS() const
{
  return fMagnitudeMPS_;
}


inline bool 
EMANE::VelocityVector::operator == (const VelocityVector &rhs) const
{
   return (fElevationDegrees_  == rhs.fElevationDegrees_   &&
           fAzimuthDegrees_    == rhs.fAzimuthDegrees_     &&
           fMagnitudeMPS_      == rhs.fMagnitudeMPS_);
}


inline bool 
EMANE::VelocityVector::updateVelocity(const VelocityVector & velocity)
{
  bool result = false;

  // check az
  if(fAzimuthDegrees_ != velocity.fAzimuthDegrees_)
   {
     fAzimuthDegrees_ =  velocity.fAzimuthDegrees_;

     result = true;
   }

  // check el
  if(fElevationDegrees_ != velocity.fElevationDegrees_)
   {
     fElevationDegrees_ = velocity.fElevationDegrees_;
  
     result = true;
   }

  // check magnitude
  if(fMagnitudeMPS_ != velocity.fMagnitudeMPS_)
   {
     fMagnitudeMPS_ = velocity.fMagnitudeMPS_;

     result = true;
   }

  return result;
}



inline std::string 
EMANE::VelocityVector::format() const
{
   char fmtBuff[256];

   snprintf(fmtBuff, sizeof(fmtBuff), "velocity: az:%8.6f deg, el:%8.6f deg, mag:%5.4f mps",
            getAzimuthDegrees(), getElevationDegrees(), getMagnitudeMPS());

   return fmtBuff;
}
