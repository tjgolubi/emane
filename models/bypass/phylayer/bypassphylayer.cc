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

#include "bypassphylayer.h"
#include "emane/emanecommonphyheader.h"
#include "emane/emaneantennamode.h"

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      {0,0,0,0,0,0,0},
    };
}

BypassPHYLayer::BypassPHYLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider* pPlatformService):
  PHYLayerImplementor(id, pPlatformService)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

BypassPHYLayer::~BypassPHYLayer()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::~BypassPHYLayer",id_);
}

void BypassPHYLayer::processDownstreamControl(const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::processDownstreamControl",id_);
#endif

  sendDownstreamControl(msg);
}

void BypassPHYLayer::processDownstreamPacket(EMANE::DownstreamPacket & pkt,
                                             const EMANE::ControlMessage &)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::processDownstreamPacket",id_);
#endif

  EMANE::CommonPHYHeader hdr(type_,                            // phy registration id
                             0,                                // tx power dBm
                             0,                                // bandwidth Hz
                             0,                                // antenna gain dBi
                             EMANE::ANTENNA_MODE_FIXED,        // antenna mode
                             ACE_Time_Value(0),                // tx time
                             0);                               // sequence number
  
  
  
  EMANE::PHYTxFrequencyInfoItems frequencyInfo(1);
  
  frequencyInfo.push_back(EMANE::PHYTxFrequencyInfo(0,                       // freq Hz
                                                    ACE_Time_Value::zero,    // tx offset
                                                    ACE_Time_Value::zero));  // duration
  
  hdr.setFrequencyInfo(frequencyInfo);
  
  sendDownstreamPacket(hdr,pkt);
}

void BypassPHYLayer::processUpstreamPacket(const EMANE::CommonPHYHeader &,
                                           EMANE::UpstreamPacket & pkt,
                                           const EMANE::ControlMessage &)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::processUpstreamPacket",id_);
#endif

  sendUpstreamPacket(pkt);
}

void BypassPHYLayer::initialize()
  throw(EMANE::InitializeException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::initialize",id_);
}

void BypassPHYLayer::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::configure",id_);
  Component::configure(items);
}

void BypassPHYLayer::start()
  throw(EMANE::StartException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::start",id_);
}

void BypassPHYLayer::stop()
  throw(EMANE::StopException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::stop",id_);
}

void BypassPHYLayer::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03d BypassPHYLayer::destroy",id_);
};

void BypassPHYLayer::processEvent(const EMANE::EventId & ,
                                  const EMANE::EventObjectState & )
{
}

void BypassPHYLayer::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{
}

DECLARE_PHY_LAYER(BypassPHYLayer);
