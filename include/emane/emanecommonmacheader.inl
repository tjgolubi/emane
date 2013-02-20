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

#include "emaneutils/netutils.h"

inline
EMANE::CommonMACHeader::CommonMACHeader(UpstreamPacket & pkt)
  throw(CommonMACHeaderException)
{
  // test to make sure there are enough bytes in the packet for 
  // the common MAC header to be present
  if(pkt.length() < sizeof(data_))
    {
      throw(CommonMACHeaderException("Packet length too small to contain header"));
    }
  else
    {
      // copy header
      memcpy(&data_, pkt.get(),sizeof(data_));
  
      // strip header from pkt
      pkt.strip(sizeof(data_));

      // convert the header to host byte order
      toHostByteOrder(data_);

      // verify the packet chekcum
      if(verifyCheckSum() == false)
        {
          throw(CommonMACHeaderException("Header checksum invalid"));
        }
      // verify the packet version
      else if(checkVersion() == false)
        {
          throw(CommonMACHeaderException("Header version mismatch"));
        }
    }
}

inline
EMANE::CommonMACHeader::CommonMACHeader(EMANE::RegistrationId registrationId)
{
  memset(&data_,0,sizeof(data_));
  data_.u16Version_ = COMMON_MAC_HEADER_VERSION;
  data_.u16CheckSum_ = 0;
  data_.u16RegistrationId_ = registrationId;
}

inline
EMANE::RegistrationId EMANE::CommonMACHeader::getRegistrationId() const
{
  return  data_.u16RegistrationId_;
}

inline
bool EMANE::CommonMACHeader::verifyCheckSum() const
{
  return EMANEUtils::inet_cksum(&data_, sizeof(data_)) == 0xFFFF;
}

inline
bool EMANE::CommonMACHeader::checkVersion() const
{
  return data_.u16Version_ == COMMON_MAC_HEADER_VERSION;
}

inline
void EMANE::CommonMACHeader::toNetworkByteOrder(Data & data) const
{
  // short
  data.u16Version_              = ACE_HTONS(data.u16Version_);
  data.u16CheckSum_             = ACE_HTONS(data.u16CheckSum_);
  data.u16RegistrationId_       = ACE_HTONS(data.u16RegistrationId_);
}

inline
void EMANE::CommonMACHeader::toHostByteOrder(Data & data) const
{
  // short
  data.u16Version_              = ACE_NTOHS(data.u16Version_);
  data.u16CheckSum_             = ACE_NTOHS(data.u16CheckSum_);
  data.u16RegistrationId_       = ACE_NTOHS(data.u16RegistrationId_);
}

inline
void EMANE::CommonMACHeader::appendTo(DownstreamPacket & pkt) const
{
  // local copy of packet data
  Data m(data_);

  // set checksum
  m.u16CheckSum_ = ~EMANEUtils::inet_cksum(&m, sizeof(m));

  // convert the header to network byte order
  toNetworkByteOrder(m);

  // add the header to the packet
  pkt.prepend(&m, sizeof(m));
}
