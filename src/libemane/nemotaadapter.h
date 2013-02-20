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

#ifndef EMANENEMOTAADAPTER_HEADER_
#define EMANENEMOTAADAPTER_HEADER_

#include "otauser.h"

#include "emane/emanedownstreamtransport.h"

#include "emaneutils/componenttypes.h"

#include <queue>

#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>
#include <ace/Thread.h>

namespace EMANE
{
  /**
   * @class NEMOTAAdapter
   *
   * @brief Over-the-Air adapter handles the intra
   * and inter platform OTA transport
   */
  class NEMOTAAdapter : public OTAUser,
                        public DownstreamTransport
  {
  public:
    NEMOTAAdapter(NEMId id);
    
    ~NEMOTAAdapter();
    
    /**
     * process OTA packet
     *
     * @param pkt Downstream packet
     * @param msg Control Message
     */

    void processOTAPacket(UpstreamPacket pkt, ControlMessage msg);
  
    /**
     *
     * process downstream packet
     *
     * @param pkt reference to the DownstreamPacket
     * @param msg referenceto the ControlMessage
     *
     */ 
    void processDownstreamPacket(DownstreamPacket     & pkt,
                                 const ControlMessage & msg);

    void open();

    void close();
    
  private:
    typedef std::pair<DownstreamPacket, ControlMessage> DownstreamQueueEntry;

    typedef std::queue<DownstreamQueueEntry> DownstreamPacketQueue;

    NEMId id_;
    ACE_thread_t thread_;
    DownstreamPacketQueue queue_;
    ACE_Thread_Mutex mutex_;
    ACE_Condition<ACE_Thread_Mutex> cond_;
    bool bCancel_;
    
    ACE_THR_FUNC_RETURN processPacketQueue();

    void processDownstreamControl(const ControlMessage &){}
  };
}

#endif //EMANENEMOTAADAPTER_HEADER_
