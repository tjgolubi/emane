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

#ifndef EMANEPHYLAYERIMPL_HEADER_
#define EMANEPHYLAYERIMPL_HEADER_

#include "emane/emanenemlayer.h"
#include "emane/emanecommonphyheader.h"

namespace EMANE
{
  /**
   * @class PHYLayerImplementor
   *
   * @brief PHY layer implementation entry point. 
   */
  class PHYLayerImplementor : public NEMLayer
  {
  public:
    virtual ~PHYLayerImplementor(){}

    virtual void processUpstreamPacket(const CommonPHYHeader & hdr,
                                       UpstreamPacket & pkt, 
                                       const ControlMessage & msg = EMPTY_CONTROL_MESSAGE) = 0;

    void sendDownstreamPacket(const CommonPHYHeader & hdr,
                              DownstreamPacket & pkt, 
                              const ControlMessage & msg = EMPTY_CONTROL_MESSAGE);
    
  protected:
    PHYLayerImplementor(NEMId id, PlatformServiceProvider *p):
      NEMLayer(id, p)
   {}

  private:
    // method meaningless for a PHY layer
    void processUpstreamControl(const ControlMessage &){}

    void processUpstreamPacket(UpstreamPacket & pkt, const ControlMessage & msg);

    void sendDownstreamPacket(DownstreamPacket & pkt, 
                              const ControlMessage & msg = EMPTY_CONTROL_MESSAGE);

  };
  
  typedef  PHYLayerImplementor * (*createPHYFunc)(NEMId id, PlatformServiceProvider *p); 
  typedef void (*destroyPHYFunc)( PHYLayerImplementor*); 
}

#define DECLARE_PHY_LAYER(X)                                                                           \
  extern "C"  EMANE::PHYLayerImplementor * create(EMANE::NEMId id, EMANE::PlatformServiceProvider *p)  \
  {return new X(id,p);}                                                                                \
  extern "C"  void destroy(EMANE::PHYLayerImplementor * p)                                             \
  {delete p;}

#include "emane/emanephylayerimpl.inl"

#endif //EMANEPHYLAYERIMPL_HEADER_
