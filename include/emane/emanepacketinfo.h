/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEPACKETINFO_HEADER_
#define EMANEPACKETINFO_HEADER_

#include "emane/emanetypes.h"

namespace EMANE
{

  /**
   * @class PacketInfo
   *
   * @brief Store source, destination, and type of service information for a packet.
   * Source and destination are stored by NEM identifier.
   */
  struct PacketInfo
  {
    NEMId         source_;        // source NEM
    NEMId         destination_;   // dest  NEM
    TypeOfService dscp_;          // ip priority in terms of dscp


    /**
     *
     * @brief  parameter constructor
     *
     * @param source the src NEM
     * @param destination the destination NEM
     * @param dscp the ip/tos converted to dscp
     *
     */
    PacketInfo(NEMId source,
               NEMId destination,
               TypeOfService dscp):
      source_(source),
      destination_(destination),
      dscp_(dscp)
      { }

    /**
     *
     * @brief  default constructor
     *
     */
    PacketInfo():
      source_(0),
      destination_(0),
      dscp_(0){}

    ~PacketInfo(){}
  };

  // All 1's NEMId represents a broad cast packet
  const NEMId NEM_BROADCAST_MAC_ADDRESS =  static_cast<NEMId>(-1); 
}

#endif //EMANEPACKETINFO_HEADER_ 
