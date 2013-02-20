/*
 * Copyright (c) 2008-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef RAWTRANSPORT_HEADER_
#define RAWTRANSPORT_HEADER_

#include <pcap.h>

#include "ethernettransport.h"
#include "emaneutils/netutils.h"
#include "emaneutils/bitpool.h"

#include <ace/Thread_Mutex.h>

#include <set>


/**
 * @class RawTransport
 *
 * @brief Raw Device EMANE Transport
 */
class RawTransport : public EMANE::EthernetTransport
{
public:
  RawTransport(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService);
  
  ~RawTransport();
  
  void initialize()
    throw(EMANE::InitializeException);
  
  void configure(const EMANE::ConfigurationItems & items)
    throw(EMANE::ConfigureException);
  
  void start()
    throw(EMANE::StartException);
  
  void stop()
    throw(EMANE::StopException);
  
  void destroy()
    throw();
  
  void processUpstreamPacket(EMANE::UpstreamPacket & pkt, const EMANE::ControlMessage & msg);
  
  void processUpstreamControl(const EMANE::ControlMessage & msg);

  void processTimedEvent(ACE_UINT32 taskType, long eventId,  const ACE_Time_Value &tv, const void *arg);

  void processEvent(const EMANE::EventId&, const EMANE::EventObjectState&);

private:
  std::string sTargetDevice_; 

  EMANEUtils::EtherAddr macAddr_;

  ACE_thread_t threadRead_;

  pcap_t *pPcapHandle_;

  BitPool *pBitPool_;

 /**
  *
  * @brief read device method reads from raw device and sends packets downstream.
  *
  * @retval NULL
  *
  */
  ACE_THR_FUNC_RETURN readDevice();

 /**
  *
  * @brief isOutbound checks packet direction based on ethernet src addr in win32.
  * 
  * @param pEtherHeader pointer to the frame ethernet header.
  *
  * @retval in win32 returns 1 if packet is outbound, otherwsise returns 0.
  *
  */
 int isOutbound(const EMANEUtils::EtherHeader *pEtherHeader);

#ifdef WIN32
  // needed to check src mac address since we can not determine packet direction in win32
  
  ACE_Thread_Mutex mutex_;

  struct ltmacaddr
   {
     bool operator()(const EMANEUtils::EtherAddr& a1, const EMANEUtils::EtherAddr& a2) const
    {
      return ACE_OS::memcmp(&a1, &a2, EMANEUtils::ETH_ALEN) < 0;
    }
  };

  typedef std::set<EMANEUtils::EtherAddr, ltmacaddr> EthAddrSet;
  typedef EthAddrSet::const_iterator EthAddrSetConstIter;
  typedef EthAddrSet::iterator EthAddrSetIter;

  EthAddrSet upstreamHostSrcMacAddrHistorySet_;
#endif

};

#endif //RAWTRANSPORT_HEADER_
