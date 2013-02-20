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

#ifndef EMANESHIMLAYER_HEADER_
#define EMANESHIMLAYER_HEADER_

#include "emane/emaneshimlayerimpl.h"
#include "emane/emanestatisticunsignedinteger32.h"

#include "nemqueuedlayer.h"
#include "logserviceproxy.h"

#include <memory>

namespace EMANE
{
  /**
   * @class ShimLayer
   *
   * @brief Bridge for a shim NEM layer.  Decouples a ShimLayerImplementor
   * implementation from the NEM to allow for interface modification
   * and encapsulation.
   *
   * @note Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
   * Design Patterns: Elements of Reusable Object-Oriented Software.
   * Addison-Wesley, Reading MA, 1995
   * Bridge, p 152
   */
  class ShimLayer : public NEMQueuedLayer
  {
  public:
    ShimLayer(NEMId id, ShimLayerImplementor * pImplementor, PlatformServiceProvider *pPlatformService);

    ~ShimLayer();
    
    void initialize()
      throw(InitializeException);

    void configure(const ConfigurationItems & items)
      throw(ConfigureException);

    void start()
      throw(StartException);

    void stop()
      throw(StopException);

    void postStart();
    
    void destroy()
      throw();

    void setDownstreamTransport(DownstreamTransport *);

    void setUpstreamTransport(UpstreamTransport *);

  private:
    std::auto_ptr<ShimLayerImplementor> pImplementor_;

    EMANE::StatisticServiceProvider *pStatisticServiceProvider_;


    EMANE::StatisticUnsignedInteger32 processedDownstreamPackets_;
    EMANE::StatisticUnsignedInteger32 processedUpstreamPackets_;
    EMANE::StatisticUnsignedInteger32 processedDownstreamControl_;
    EMANE::StatisticUnsignedInteger32 processedUpstreamControl_;
    EMANE::StatisticUnsignedInteger32 processedEvents_;

    void doProcessUpstreamControl(const ControlMessage &);
    
    void doProcessUpstreamPacket(UpstreamPacket &,
                                 const ControlMessage &);

    void doProcessDownstreamControl(const ControlMessage &);
    
    void doProcessDownstreamPacket(DownstreamPacket &,
                                   const ControlMessage &);
    
    void doProcessEvent(const EventId & , const EventObjectState &);

    void doProcessTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);
  };
}

#endif //EMANESHIMLAYER_HEADER_
