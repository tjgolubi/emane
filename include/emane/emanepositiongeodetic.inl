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



inline EMANE::PositionGeodetic::PositionGeodetic() :
  fLatDegrees_(0),
  fLonDegrees_(0),
  fAltMeters_(0),
  fYawDegrees_(0),
  fPitchDegrees_(0),
  fRollDegrees_(0)
{
  toXYZ();
};


inline EMANE::PositionGeodetic::PositionGeodetic(float fLatDegrees, 
                                                 float fLonDegrees, 
                                                 float fAltMeters, 
                                                 float fYawDegrees,
                                                 float fPitchDegrees,
                                                 float fRollDegrees) :
  fLatDegrees_(fLatDegrees),
  fLonDegrees_(fLonDegrees),
  fAltMeters_(fAltMeters),
  fYawDegrees_(fYawDegrees),
  fPitchDegrees_(fPitchDegrees),
  fRollDegrees_(fRollDegrees)
{ 
  toXYZ();
};


inline float
EMANE::PositionGeodetic::getLatitudeDegrees() const
{
  return fLatDegrees_;
}


inline float
EMANE::PositionGeodetic::getLongitudeDegrees() const
{
  return fLonDegrees_;
}


inline float
EMANE::PositionGeodetic::getAltitudeMeters() const
{
  return fAltMeters_;
}


inline float
EMANE::PositionGeodetic::getLatitudeRadians() const
{
  return EMANEUtils::DEGREES_TO_RADIANS(fLatDegrees_);
}


inline float
EMANE::PositionGeodetic::getLongitudeRadians() const
{
  return EMANEUtils::DEGREES_TO_RADIANS(fLonDegrees_);
}


inline float
EMANE::PositionGeodetic::getYawRadians() const
{
  return EMANEUtils::DEGREES_TO_RADIANS(fYawDegrees_);
}


inline float
EMANE::PositionGeodetic::getPitchRadians() const
{
  return EMANEUtils::DEGREES_TO_RADIANS(fPitchDegrees_);
}


inline float
EMANE::PositionGeodetic::getRollRadians() const
{
  return EMANEUtils::DEGREES_TO_RADIANS(fRollDegrees_);
}


inline float
EMANE::PositionGeodetic::getYawDegrees() const
{
  return fYawDegrees_;
}


inline float
EMANE::PositionGeodetic::getPitchDegrees() const
{
  return fPitchDegrees_;
}


inline float
EMANE::PositionGeodetic::getRollDegrees() const
{
  return fRollDegrees_;
}


inline EMANE::PositionGeodetic::XYZ 
EMANE::PositionGeodetic::getXYZ() const
{
  return xyz_;
}


inline void
EMANE::PositionGeodetic::toXYZ()
{
  const float fLatitudeRadians = getLatitudeRadians();

  const float fLongitudeRadians = getLongitudeRadians();

  const float fAltitudeMeters = getAltitudeMeters();

  const double R = EMANE::SEMI_MAJOR / sqrt (1.0 - (EMANE::ECC2 * (pow(sin(fLatitudeRadians), 2.0))));

  const double X = (R + fAltitudeMeters)  * cos(fLatitudeRadians) * cos(fLongitudeRadians);

  const double Y = (R + fAltitudeMeters)  * cos(fLatitudeRadians) * sin(fLongitudeRadians);

  const double Z = ((1.0 - ECC2) * R + fAltitudeMeters) * sin(fLatitudeRadians);

  xyz_ = EMANE::PositionGeodetic::XYZ(X, Y, Z);
}


inline bool 
EMANE::PositionGeodetic::operator == (const PositionGeodetic &rhs) const
{
   return (fLonDegrees_   == rhs.fLonDegrees_   &&
           fLatDegrees_   == rhs.fLatDegrees_   &&
           fAltMeters_    == rhs.fAltMeters_    &&
           fYawDegrees_   == rhs.fYawDegrees_   &&
           fPitchDegrees_ == rhs.fPitchDegrees_ && 
           fRollDegrees_  == rhs.fRollDegrees_);
}


inline bool 
EMANE::PositionGeodetic::updatePosition(const EMANE::PositionGeodetic & position)
{
   bool changed = false;

   // check latitude change
   if(fLatDegrees_ != position.fLatDegrees_)
    {
      // reset
      fLatDegrees_ = position.fLatDegrees_;

      // position changed
      changed = true;
    }

   // check longitude change
   if(fLonDegrees_ != position.fLonDegrees_)
    {
      // reset
      fLonDegrees_ = position.fLonDegrees_;

      // position changed
      changed = true;
    }

   // check altitude change
   if(fAltMeters_ != position.fAltMeters_)
    {
      // reset
      fAltMeters_ = position.fAltMeters_;

      // position changed
      changed = true;
    }

   // check yaw change
   if(fYawDegrees_ != position.fYawDegrees_)
    {
      // reset
      fYawDegrees_ = position.fYawDegrees_;

      // orientation changed
      changed = true;
    }

   // check pitch change
   if(fPitchDegrees_ != position.fPitchDegrees_)
    {
      // reset
      fPitchDegrees_ = position.fPitchDegrees_;

      // orientation changed
      changed = true;
    }

   // check roll change
   if(fRollDegrees_ != position.fRollDegrees_)
    {
      // reset
      fRollDegrees_ = position.fRollDegrees_;

      // orientation changed
      changed = true;
    }

   // the update caused a significant change
   if(changed == true)
    {
      // update xyz
      toXYZ();

      // something changed
      return true;
    }
   else
    {
      // no change
      return false;
    }
}



inline std::string 
EMANE::PositionGeodetic::format() const
{
   char fmtBuff[1024];

   snprintf(fmtBuff, sizeof(fmtBuff), "position: lat:%8.6f deg, lon:%8.6f deg, alt:%5.4f m, yaw:%8.6f deg, pitch:%8.6f deg, roll:%8.4f deg\n\t%s",
            getLatitudeDegrees(), 
            getLongitudeDegrees(), 
            getAltitudeMeters(), 
            getYawDegrees(), 
            getPitchDegrees(), 
            getRollDegrees(), 
            xyz_.format().c_str());

   return fmtBuff;
}
