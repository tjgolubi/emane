/*
 * Copyright (c) 2009-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef ETHERNETTRANSPORT_HEADER_
#define ETHERNETTRANSPORT_HEADER_

#include "emane/emanetransport.h"
#include "emaneutils/netutils.h"
#include <ace/Basic_Types.h>
#include <ace/Thread_Mutex.h>

#include <map>

namespace EMANE 
{
  /**
   * @class EthernetTransport
   *
   * @brief EMANE Ethernet Transport
   *
   */
  class EthernetTransport : public EMANE::Transport
  {
  public:
    EthernetTransport(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService);
  
    ~EthernetTransport();
  
  protected:
    virtual int parseFrame(const EMANEUtils::EtherHeader *pEthHeader, EMANE::NEMId & dst, ACE_UINT8 & dscp);
    virtual int verifyFrame(const void * buf, size_t len);

    void updateArpCache(const EMANEUtils::EtherHeader *pEthHeader, EMANE::NEMId nemId);
    void addEntry(const EMANEUtils::EtherAddr& addr, EMANE::NEMId nemId);

    EMANE::NEMId lookupArpCache(const EMANEUtils::EtherAddr *pEthAddr);

    bool bBroadcastMode_;
    bool bArpCacheMode_;

  private:
    ACE_Thread_Mutex mutex_;

/**
 * @brief EtherAddr compare operator
 *
 */
    struct ltmacaddr
     {
/**
 * @param a1  ether addr 1
 * @param a2  ether addr 2
 *
 * @return returns true if a1 is less than a2, else returns false
 *
 */

       bool operator()(const EMANEUtils::EtherAddr& a1, const EMANEUtils::EtherAddr& a2) const
        {
          return ACE_OS::memcmp(&a1, &a2, EMANEUtils::ETH_ALEN) < 0;
        }
     };

    typedef std::map<EMANEUtils::EtherAddr, EMANE::NEMId, ltmacaddr> EthAddrMap;
    typedef EthAddrMap::const_iterator EthAddrMapConstIter;
    typedef EthAddrMap::iterator EthAddrMapIter;

    EthAddrMap macCache_;
  };
}
#endif //ETHERNETTRANSPORT_HEADER_
