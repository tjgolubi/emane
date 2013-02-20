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

#ifndef SAMPLEMACLAYER_HEADER_
#define SAMPLEMACLAYER_HEADER_

#include "emane/emanemaclayerimpl.h"
#include "emane/emanemactypes.h"

/**
 * @class BypassMACLayer
 *
 * @brief Implementation of the bypass MAC layer
 */
class BypassMACLayer : public EMANE::MACLayerImplementor
{
public:
  /**
   *
   * @brief constructor
   *
   */
  BypassMACLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider* pPlatformService);

  /**
   *
   * @brief destructor
   *
   */
  ~BypassMACLayer();

  /**
   *
   * @brief initialize
   *
   */
  void initialize()
    throw(EMANE::InitializeException);

  /**
   *
   * @brief configure
   *
   * @param items Reference to the ConfigurationItems list
   *
   */
  void configure(const EMANE::ConfigurationItems & items)
    throw(EMANE::ConfigureException);
 
  /**
   *
   * @brief start
   *
   */
  void start()
    throw(EMANE::StartException);
  
  /**
   *
   * @brief stop
   *
   */
  void stop()
    throw(EMANE::StopException);
  
  /**
   *
   * @brief destroy
   *
   */
  void destroy()
    throw();

  /**
  *
  * @brief process upstream control
  *
  * @param msg Reference to the ControlMessage
  *
  */
  void processUpstreamControl(const EMANE::ControlMessage &msg);

  /**
  *
  * @brief process upstream packet
  *
  * @param hdr Reference to the CommonMACHeader
  * @param pkt Reference to the UpstreamPacket
  * @param msg Reference to the ControlMessage
  *
  */
  void processUpstreamPacket(const EMANE::CommonMACHeader & hdr,
                             EMANE::UpstreamPacket & pkt,
                             const EMANE::ControlMessage & msg);  

  /**
  *
  * @brief process downstream control
  *
  * @param msg Reference to the ControlMessage
  *
  */
  void processDownstreamControl(const EMANE::ControlMessage & msg);
  
  /**
  *
  * @brief process downstream packet
  *
  * @param pkt Reference to the DownstreamPacket
  * @param msg Reference to the ControlMessage
  *
  */
  void processDownstreamPacket(EMANE::DownstreamPacket & pkt,
                               const EMANE::ControlMessage & msg);
  /**
  *
  * @brief process event
  *
  * @param id Reference to the EventId
  * @param state Reference to the EventObjectState
  *
  */
  void processEvent(const EMANE::EventId & id, const EMANE::EventObjectState & state);

  /**
  *
  * @brief processTimedEvent
  *
  * @param taskType  task type
  * @param eventId   event id
  * @param tv  Reference to the absolute scheduled timeout
  * @param arg opaque data
  *
  */
  void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);

private:
  static const EMANE::RegistrationId type_ = EMANE::REGISTERED_EMANE_MAC_BYPASS;
};

#endif //BYPASSMACLAYER_HEADER_
