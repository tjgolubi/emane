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

#include "emane/emanenet.h"

#include <sstream>

inline 
EMANE::UniversalPhyControlAntennaProfileMessage::UniversalPhyControlAntennaProfileMessage() :
 u16ProfileId_(0),
 i32AzimuthMARCS_(0),
 i32ElevationMARCS_(0)
{ }

inline 
EMANE::UniversalPhyControlAntennaProfileMessage::UniversalPhyControlAntennaProfileMessage(const void * buf, const size_t len)
         throw(UniversalPHYControlAntennaProfileMessageException)
 {
   if(len != sizeof(*this))
    {
      std::stringstream ss;
      ss << "Buffer size mismatch expected "
         <<  sizeof(*this)
         << " given "
         << len
         << std::ends;

      throw(UniversalPHYControlAntennaProfileMessageException(ss.str()));
    }
   else
    {
      memcpy(this, buf, sizeof(*this));

      toHostByteOrder();
    }
}


inline 
EMANE::UniversalPhyControlAntennaProfileMessage::UniversalPhyControlAntennaProfileMessage(float fAzimuthDegrees,
                                                                                          float fElevationDegrees,
                                                                                          ACE_UINT16 u16ProfileId) :
 u16ProfileId_(u16ProfileId),
 i32AzimuthMARCS_(fAzimuthDegrees * EMANE::MILLI_ARC_SECONDS_PER_DEGREE),
 i32ElevationMARCS_(fElevationDegrees * EMANE::MILLI_ARC_SECONDS_PER_DEGREE)
{ }
   
inline float
EMANE::UniversalPhyControlAntennaProfileMessage::getAntennaElevationDegrees() const 
{ 
  return static_cast<float> (i32ElevationMARCS_) / EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 
}


inline void
EMANE::UniversalPhyControlAntennaProfileMessage::setAntennaElevationDegrees(float fAntennaElevationDegrees)
{ 
  i32ElevationMARCS_ = fAntennaElevationDegrees * EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 
}


inline float
EMANE::UniversalPhyControlAntennaProfileMessage::getAntennaAzimuthDegrees() const 
{ 
  return static_cast<float> (i32AzimuthMARCS_) / EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 
}


inline void
EMANE::UniversalPhyControlAntennaProfileMessage::setAntennaAzimuthDegrees(float fAntennaAzimuthDegrees)
{ 
  i32AzimuthMARCS_ = fAntennaAzimuthDegrees * EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 
}


inline ACE_UINT16
EMANE::UniversalPhyControlAntennaProfileMessage::getAntennaProfileId() const 
{ 
  return u16ProfileId_;
}


inline void
EMANE::UniversalPhyControlAntennaProfileMessage::setAntennaProfileId(ACE_UINT16 u16ProfileId)
{ 
  u16ProfileId_ = u16ProfileId;
}


inline std::string
EMANE::UniversalPhyControlAntennaProfileMessage::format() const
{
  char fmtBuff[256];

  snprintf(fmtBuff, sizeof(fmtBuff), "antennainfo: elevation %5.4f, azimuth %5.4f, profile id %hu",
           getAntennaElevationDegrees(), 
           getAntennaAzimuthDegrees(),
           getAntennaProfileId());

  return fmtBuff;
}

    
inline void
EMANE::UniversalPhyControlAntennaProfileMessage::toNetworkByteOrder()
{
  // long
  i32ElevationMARCS_ = ACE_HTONL(i32ElevationMARCS_);
  i32AzimuthMARCS_   = ACE_HTONL(i32AzimuthMARCS_);

  // short
  u16ProfileId_      = ACE_HTONS(u16ProfileId_);
}


inline void
EMANE::UniversalPhyControlAntennaProfileMessage::toHostByteOrder()
{
  // long
  i32ElevationMARCS_ = ACE_NTOHL(i32ElevationMARCS_);
  i32AzimuthMARCS_   = ACE_NTOHL(i32AzimuthMARCS_);

  // short
  u16ProfileId_      = ACE_NTOHS(u16ProfileId_);
}

