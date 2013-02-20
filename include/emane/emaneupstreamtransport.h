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

#ifndef EMANEUPSTREAMTRANSPORT_HEADER_
#define EMANEUPSTREAMTRANSPORT_HEADER_

#include "emane/emanecontrolmessage.h"
#include "emane/emaneupstreampacket.h"
#include "emane/emanedownstreampacket.h"

namespace EMANE
{
  class DownstreamTransport;

  /**
   * @class UpstreamTransport
   *
   * @brief UpstreamTransport allows for processing upstream data and 
   * control messages.
   */
  class UpstreamTransport
  {
  public:
    virtual ~UpstreamTransport(){}

    /**
     * Process upstream packet
     *
     * @param pkt reference to the UpstreamPacket to process
     * @param msg optional reference to the ControlMessage
     */
    virtual void processUpstreamPacket(UpstreamPacket & pkt, 
                                       const ControlMessage & msg = EMPTY_CONTROL_MESSAGE) = 0;
    /**
     * Process upstream control message
     *
     * @param msg reference to the ControlMessage
     * 
     */
    virtual void processUpstreamControl(const ControlMessage & msg) = 0;
    

    /**
     * Set the downstream transport.
     *
     * @param pDownstreamTransport Pointer to the downstream transport
     * of this upstream transport.
     */
    virtual void setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
    {
      pDownstreamTransport_ = pDownstreamTransport; 
    }


    /**
     * Send downsteam packet
     *
     * @param pkt reference to the DownstreamPacket to process
     * @param msg optional reference to the ControlMessage
     */
    void sendDownstreamPacket(DownstreamPacket & pkt, 
                              const ControlMessage & msg = EMPTY_CONTROL_MESSAGE);
    

    /**
     * Send downstream control message
     *
     * @param msg reference to the ControlMessage
     * 
     */
    void sendDownstreamControl(const ControlMessage & msg);


  protected:

    UpstreamTransport():
      pDownstreamTransport_(0){}

                             
  private:
    DownstreamTransport * pDownstreamTransport_;
  };
}

#include "emane/emaneupstreamtransport.inl"

#endif //EMANEUPSTREAMTRANSPORT_HEADER_
