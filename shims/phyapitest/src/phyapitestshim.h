/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "emane/emaneshimlayerimpl.h"
#include "emanecontrolmessages/universalphycontrolsendmessage.h"
#include "emanecontrolmessages/universalphycontrolrecvmessage.h"

#include <ace/Time_Value.h>
#include <ace/Basic_Types.h>


namespace EMANE
{
  /**
   * class PHYAPITestShimLayer
   *
   * @brief shim layer used to test the universal phy send/recv api
   *
   */

  class PHYAPITestShimLayer : public EMANE::ShimLayerImplementor
  {
  public:

    PHYAPITestShimLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService);

    ~PHYAPITestShimLayer();

    void initialize() 
      throw(EMANE::InitializeException);

    void configure(const EMANE::ConfigurationItems & items)
      throw(EMANE::ConfigureException);

    void start()
      throw(EMANE::StartException);

    void postStart();

    void stop()
      throw(EMANE::StopException);

    void destroy()
      throw();

    void processUpstreamControl(const EMANE::ControlMessage & msg);

    void processDownstreamControl(const EMANE::ControlMessage & msg);

    void processUpstreamPacket(EMANE::UpstreamPacket & pkt, const EMANE::ControlMessage & msg);

    void processDownstreamPacket(EMANE::DownstreamPacket & pkt, const EMANE::ControlMessage & msg);

    void processEvent(const EMANE::EventId &, const EMANE::EventObjectState &);

    void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);

  private:
    // the pkt payload size
    ACE_UINT16 u16PacketSize_;

    // the pkt tx interval (1 / pps)
    ACE_Time_Value tvTxInterval_;

    // the tx timed event id
    long txTimedEventId_;

    // the pkt destination
    EMANE::NEMId dst_;

    // additional transmitter info (ati)
    EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems ati_;

    // frequency info (fi)
    EMANE::PHYTxFrequencyInfoItems fi_;

    // per pkt tx control
    EMANE::UniversalPhyControlSendMessage txCtrl_;
  };
}
