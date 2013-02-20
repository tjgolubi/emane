/*
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef TIMINGSHIMHEADER_HEADER_
#define TIMINGSHIMHEADER_HEADER_

#include "emane/emanenet.h"

namespace EMANE
{

  /**
   *
   * @brief Timing Analysis Shim Header
   *
   */
  struct TimingShimHeader
  {

    EMANE::UINT32  u32TimeSec_;
    EMANE::UINT32  u32TimeUsec_;
    EMANE::UINT16  u16PacketID_;
    EMANE::UINT16  u16Source_;

    /**
     *
     * @brief  Timing Analysis shim header initializer.
     *
     */
    TimingShimHeader():
      u32TimeSec_(0),
      u32TimeUsec_(0),
      u16PacketID_(0),
      u16Source_(0){}
    
    /**
     *
     * @brief  Timing Analysis shim header initializer.
     *
     * @param timeSec   Seconds portion of the time value
     * @param timeUsec  Useconds portion of the time value
     * @param packetID  the packet id
     * @param source    the packet source
     */
    TimingShimHeader(EMANE::UINT32 timeSec,
                     EMANE::UINT32 timeUsec,
                     EMANE::UINT16 packetID,
                     EMANE::UINT16 source):
      u32TimeSec_(timeSec),
      u32TimeUsec_(timeUsec),
      u16PacketID_(packetID),
      u16Source_(source){}
    
    /**
     *
     * @brief convert shim header from host to network byte order.
     *
     */
    void toNetworkByteOrder()
    {
      u32TimeSec_  = EMANE::HTONL(u32TimeSec_);
      u32TimeUsec_ = EMANE::HTONL(u32TimeUsec_);
      u16PacketID_ = EMANE::HTONS(u16PacketID_);
      u16Source_   = EMANE::HTONS(u16Source_);
    }

    /**
     *
     * @brief convert shim header from network to host byte order.
     *
     */
    void toHostByteOrder()
    {
      u32TimeSec_  = EMANE::NTOHL(u32TimeSec_);
      u32TimeUsec_ = EMANE::NTOHL(u32TimeUsec_);
      u16PacketID_ = EMANE::NTOHS(u16PacketID_);
      u16Source_   = EMANE::NTOHS(u16Source_);
    }
  
  } __attribute__((packed));
}
#endif
