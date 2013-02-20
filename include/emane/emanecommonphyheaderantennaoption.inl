/*
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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
#include "emane/emaneconstants.h"

inline
EMANE::CommonPHYHeaderAntennaOption::CommonPHYHeaderAntennaOption(const ACE_UINT16 u16ProfileId,
                                                                  const float fAntennaAzimuthDeg,
                                                                  const float fAntennaElevationDeg):
  CommonPHYHeaderOption(COMMON_PHY_OPTION)
{
  memset(&data_,0,sizeof(data_));
  
  data_.u16ProfileId_ = u16ProfileId;
  data_.u32AntennaAzimuthMARCS_   = fAntennaAzimuthDeg   * EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
  data_.i32AntennaElevationMARCS_ = fAntennaElevationDeg * EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
}

inline
EMANE::CommonPHYHeaderAntennaOption::CommonPHYHeaderAntennaOption(const CommonPHYHeaderOptionObjectState & state)
  throw(CommonPHYHeaderException):
  CommonPHYHeaderOption(COMMON_PHY_OPTION)
{
  if(state.length() == sizeof(data_))
    {
      memcpy(&data_, state.get(), sizeof(data_));

      toHostByteOrder(data_);
    }
  else
    {
      throw(CommonPHYHeaderException("CommonPHYHeaderAntennaOption size mismatch"));
    }
}

inline
ACE_UINT8 EMANE::CommonPHYHeaderAntennaOption::getAntennaProfileId() const
{
  return data_.u16ProfileId_;
}


inline
float EMANE::CommonPHYHeaderAntennaOption::getAntennaElevationDegrees() const
{
  return data_.i32AntennaElevationMARCS_ / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
}

inline
float EMANE::CommonPHYHeaderAntennaOption::getAntennaAzimuthDegrees() const
{
  return data_.u32AntennaAzimuthMARCS_ / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
}

inline
std::string EMANE::CommonPHYHeaderAntennaOption::format()
{
  char buff[256];
  
  snprintf(buff, sizeof(buff), "antenna: profile %hu, azimuth %3.2f deg, elevation %3.2f deg",
           getAntennaProfileId(), 
           getAntennaAzimuthDegrees(),
           getAntennaElevationDegrees());

  return buff;
}

inline      
void EMANE::CommonPHYHeaderAntennaOption::toNetworkByteOrder(Data & data) const
{
  // long
  data.u32AntennaAzimuthMARCS_   = ACE_HTONL(data.u32AntennaAzimuthMARCS_);
  data.i32AntennaElevationMARCS_ = ACE_HTONL(data.i32AntennaElevationMARCS_);

  // short
  data.u16ProfileId_ = ACE_HTONS(data.u16ProfileId_);
}

inline    
void EMANE::CommonPHYHeaderAntennaOption::toHostByteOrder(Data & data) const
{
  // long
  data.u32AntennaAzimuthMARCS_   = ACE_NTOHL(data.u32AntennaAzimuthMARCS_);
  data.i32AntennaElevationMARCS_ = ACE_NTOHL(data.i32AntennaElevationMARCS_);

  // short
  data.u16ProfileId_ = ACE_NTOHS(data.u16ProfileId_);
}

inline
EMANE::CommonPHYHeaderOptionObjectState EMANE::CommonPHYHeaderAntennaOption::getObjectState() const
{
  Data data = data_;

  toNetworkByteOrder(data);

  return CommonPHYHeaderOptionObjectState(COMMON_PHY_OPTION,&data,sizeof(data));
}
