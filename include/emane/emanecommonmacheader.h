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

#ifndef EMANE_COMMONMACHEADER_HEADER_
#define EMANE_COMMONMACHEADER_HEADER_

#include "emane/emanetypes.h"
#include "emane/emaneupstreampacket.h"
#include "emane/emanedownstreampacket.h"
#include "emane/emanecommonmacheaderexception.h"

#include <ace/Basic_Types.h>

namespace EMANE
{
  /**
   * @class CommonMACHeader
   *
   * @brief  the mac header common to all MAC implementations
   *
   */

  class CommonMACHeader
  {
  public:

    /**
     *
     * @brief  common mac header initializer.
     *
     * @param pkt reference to an UpstreamPacket
     */
    CommonMACHeader(UpstreamPacket & pkt) 
      throw(CommonMACHeaderException);
  
    /**
     *
     * @brief  common mac header initializer.
     *
     * @param registrationId    mac registration Id
     */
    CommonMACHeader(EMANE::RegistrationId   registrationId);

    /**
     *
     * @return returns the registrationId
     *
     */

    EMANE::RegistrationId getRegistrationId() const;

    /**
     *
     * @return returns true if the header checksum is valid, else false
     *
     */
    bool verifyCheckSum() const;

    /**
     *
     * @return returns true if the version is valid, else false
     *
     */
    bool checkVersion() const;

    /**
     *
     * @param pkt DownstreamPacket to append to header
     *
     */
    void appendTo(DownstreamPacket & pkt) const;
    
  private:
    const static ACE_UINT16 COMMON_MAC_HEADER_VERSION = 0x0001;
   
  /**
   * @struct Data
   *
   * @brief  the CommonMACHeader private data
   *
   */
    struct Data
    {
      ACE_UINT16            u16Version_;            // mac header version
      ACE_UINT16            u16CheckSum_;           // header checksum
      EMANE::RegistrationId u16RegistrationId_;     // mac id, see mac registration id's
    } __attribute__((packed)) data_;

    void toHostByteOrder(Data & data) const;
    
    void toNetworkByteOrder(Data & data) const;
  };
}

#include "emane/emanecommonmacheader.inl"

#endif //EMANE_COMMONMACHEADER_HEADER_
