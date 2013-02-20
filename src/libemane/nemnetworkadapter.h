/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANENEMNETWORKADAPTER_HEADER_
#define EMANENEMNETWORKADAPTER_HEADER_

#include "networkadapterexception.h"

#include "emane/emaneupstreamtransport.h"
#include "emane/emanedownstreampacket.h"

#include "emaneutils/componenttypes.h"

#include <ace/INET_Addr.h>
#include <ace/SOCK_Dgram.h>
#include <ace/Thread.h>
#include <ace/Thread_Mutex.h>

namespace EMANE
{
  /**
   * @class NEMNetworkAdapter
   *
   * @brief Entry point for packets and crontrol sourced and sinked
   * from the network layer
   */
  class NEMNetworkAdapter : public UpstreamTransport
  {
  public:
    NEMNetworkAdapter(NEMId id);
    
    ~NEMNetworkAdapter();
    
    void processUpstreamPacket(UpstreamPacket & pkt,
                               const ControlMessage & msg);
    
    void processUpstreamControl(const ControlMessage &);
    
    /**
     * Open the newtork layer interface
     *
     * @param localAddress Local address to bind to
     * @param remoteAddress RemoteAddress to send data to
     */
    void open(const ACE_INET_Addr & localAddress, const ACE_INET_Addr & remoteAddress)
      throw(NetworkAdapterException);
    
    void close();
    
  private:
    ACE_SOCK_Dgram udp_;
    ACE_INET_Addr localAddress_;
    ACE_INET_Addr remoteAddress_;
    NEMId id_;
    ACE_thread_t thread_;
    ACE_Thread_Mutex mutex_;
    bool bOpen_;
    ACE_INET_Addr addr_;

    ACE_THR_FUNC_RETURN processNetworkMessage();
  };
}

#endif //EMANENETNETWORKOTAADAPTER_HEADER_
