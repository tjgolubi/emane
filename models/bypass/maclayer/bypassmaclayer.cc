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

#include "bypassmaclayer.h"

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      {0,0,0,0,0,0,0},
    };
}


BypassMACLayer::BypassMACLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider* pPlatformService):
  MACLayerImplementor(id, pPlatformService)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

BypassMACLayer::~BypassMACLayer()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"BypassMACLayer::~BypassMACLayer()");
}

void BypassMACLayer::processUpstreamControl(const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::processUpstreamControl",id_);
#endif

  sendUpstreamControl(msg);
}

void BypassMACLayer::processDownstreamControl(const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::processDownstreamControl",id_);
#endif

  sendDownstreamControl(msg);
}

void BypassMACLayer::processUpstreamPacket(const EMANE::CommonMACHeader & hdr,
                                           EMANE::UpstreamPacket & pkt, 
                                           const EMANE::ControlMessage &)
{
  if(hdr.getRegistrationId() != type_)
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: MAC Registration Id does not match, drop.",
                             id_, "BypassMACLayer", __func__);
      return;
    }

#ifdef VERBOSE_LOGGING
  const EMANE::PacketInfo & pinfo =  pkt.getPacketInfo();

  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::processUptreamPacket src %hu dst %hu dscp %hhu",
      id_,
      pinfo.source_,
      pinfo.destination_,
      pinfo.dscp_);
#endif

  sendUpstreamPacket(pkt);
}


void BypassMACLayer::processDownstreamPacket(EMANE::DownstreamPacket & pkt, const EMANE::ControlMessage &)
{
#ifdef VERBOSE_LOGGING
  const EMANE::PacketInfo  pinfo =  pkt.getPacketInfo();

  pPlatformService_->log(EMANE::DEBUG_LEVEL,
      "MACI %03d BypassMACLayer::processDownstreamPacket src %hu dst %hu dscp %hhu",
      id_,
      pinfo.source_,
      pinfo.destination_,
      pinfo.dscp_);
#endif

  // send pkt downstream
  sendDownstreamPacket(EMANE::CommonMACHeader(type_), pkt);
}

void BypassMACLayer::initialize()
  throw(EMANE::InitializeException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::initialize",id_);
}

void BypassMACLayer::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::configure",id_);
  Component::configure(items);
}

void BypassMACLayer::start()
  throw(EMANE::StartException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::start",id_);
}

void BypassMACLayer::stop()
  throw(EMANE::StopException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::stop",id_);
}

void BypassMACLayer::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::destroy",id_);
};

void BypassMACLayer::processEvent(const EMANE::EventId &, const EMANE::EventObjectState &)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::processEvent",id_);
#endif
}

void BypassMACLayer::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03d BypassMACLayer::processTimedEvent",id_);
#endif
}

DECLARE_MAC_LAYER(BypassMACLayer);
