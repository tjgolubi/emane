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

#include "maclayer.h"
#include "logservice.h"

EMANE::MACLayer::MACLayer(NEMId id, MACLayerImplementor * pImplementor, PlatformServiceProvider *pPlatformService):
  NEMQueuedLayer(id, pPlatformService),
  pImplementor_(pImplementor),
  pStatisticServiceProvider_(pPlatformService)
{
  pStatisticServiceProvider_->registerStatistic("processedDownstreamPackets", &processedDownstreamPackets_);
  pStatisticServiceProvider_->registerStatistic("processedUpstreamPackets", &processedUpstreamPackets_);
  pStatisticServiceProvider_->registerStatistic("processedDownstreamControl", &processedDownstreamControl_);
  pStatisticServiceProvider_->registerStatistic("processedUpstreamControl", &processedUpstreamControl_);
  pStatisticServiceProvider_->registerStatistic("processedEvents", &processedEvents_);
}

EMANE::MACLayer::~MACLayer(){}

void EMANE::MACLayer::initialize() throw(InitializeException)
{
  pImplementor_->initialize();
}

void EMANE::MACLayer::configure(const ConfigurationItems & items) throw(ConfigureException)
{
  pImplementor_->configure(items);
}

void EMANE::MACLayer::start() throw(StartException)
{
  // start the queue processing thread
  NEMQueuedLayer::start();

  pImplementor_->start();
}

void EMANE::MACLayer::postStart()
{
  pImplementor_->postStart();
}

void EMANE::MACLayer::stop() throw(StopException)
{
  pImplementor_->stop();

  // stop the queue processing thread
  NEMQueuedLayer::stop();
}

void EMANE::MACLayer::destroy() throw()
{
  pImplementor_->destroy();
}

void EMANE::MACLayer::doProcessUpstreamPacket(UpstreamPacket & pkt,
                                              const ControlMessage & ctrl)
{
   static_cast<UpstreamTransport *>(pImplementor_.get())->processUpstreamPacket(pkt,ctrl);

  // update statistics
  ++processedUpstreamPackets_;
}

void EMANE::MACLayer::doProcessDownstreamPacket(DownstreamPacket & pkt,
                                                const ControlMessage & ctrl)
{
  pImplementor_->processDownstreamPacket(pkt,ctrl);

  // update statistics
  ++processedDownstreamPackets_;
}

void EMANE::MACLayer::doProcessUpstreamControl(const ControlMessage & msg)
{
  pImplementor_->processUpstreamControl(msg);

  // update statistics
  ++processedUpstreamControl_;
}

void EMANE::MACLayer::doProcessDownstreamControl(const ControlMessage & msg)
{
  pImplementor_->processDownstreamControl(msg);

  // update statistics
  ++processedDownstreamControl_;
}

void EMANE::MACLayer::setUpstreamTransport(UpstreamTransport * pUpstreamTransport)
{
  pImplementor_->setUpstreamTransport(pUpstreamTransport);
}

void EMANE::MACLayer::setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
{
  pImplementor_->setDownstreamTransport(pDownstreamTransport);
}

void EMANE::MACLayer::doProcessEvent(const EventId & eventId, 
                                     const EventObjectState & state)
{
  
  pImplementor_->processEvent(eventId,state);
  
  // update statistics
  ++processedEvents_;
}


void EMANE::MACLayer::doProcessTimedEvent(ACE_UINT32 taskType, long eventId,  const ACE_Time_Value &tv, const void *arg)
{
  pImplementor_->processTimedEvent(taskType, eventId, tv, arg);
}
