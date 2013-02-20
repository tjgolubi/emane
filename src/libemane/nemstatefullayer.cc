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

#include "nemstatefullayer.h"
#include "nemlayerstate.h"
#include "nemlayerstateuninitialized.h"

EMANE::NEMStatefulLayer::NEMStatefulLayer(NEMId id, 
                                          NEMLayer * pLayer, 
                                          EMANE::PlatformServiceProvider *pPlatformService):
  NEMLayer(id, pPlatformService),
  pLayer_(pLayer),
  pState_(NEMLayerStateUninitializedSingleton::instance()){}

EMANE::NEMStatefulLayer::~NEMStatefulLayer(){}
    
void EMANE::NEMStatefulLayer::initialize()
  throw(InitializeException)
{
  pState_->handleInitialize(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::configure(const ConfigurationItems & items)
  throw(ConfigureException)
{
  pState_->handleConfigure(this,pLayer_.get(),items);
}

void EMANE::NEMStatefulLayer::start()
  throw(StartException)
{
  pState_->handleStart(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::postStart()
{
  pState_->handlePostStart(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::stop()
  throw(StopException)
{
  pState_->handleStop(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::destroy()
  throw()
{
  pState_->handleDestroy(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::processDownstreamControl(const ControlMessage & msg)
{
  pState_->processDownstreamControl(this,pLayer_.get(),msg);
}

void EMANE::NEMStatefulLayer::processDownstreamPacket(DownstreamPacket & pkt,
                                                      const ControlMessage & msg)
{
  pState_->processDownstreamPacket(this,pLayer_.get(),pkt,msg);
}

void EMANE::NEMStatefulLayer::processUpstreamPacket(UpstreamPacket & pkt,
                                                    const ControlMessage & msg)
{
  pState_->processUpstreamPacket(this,pLayer_.get(),pkt,msg);
}

void EMANE::NEMStatefulLayer::processUpstreamControl(const ControlMessage & msg)
{
  pState_->processUpstreamControl(this,pLayer_.get(),msg);
}

void EMANE::NEMStatefulLayer::processEvent(const EventId & id,
                                           const EventObjectState & state)
{
  pState_->processEvent(this,pLayer_.get(),id,state);
}

void EMANE::NEMStatefulLayer::setUpstreamTransport(UpstreamTransport * pUpstreamTransport)
{
  pLayer_->setUpstreamTransport(pUpstreamTransport);
}

void EMANE::NEMStatefulLayer::setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
{
  pLayer_->setDownstreamTransport(pDownstreamTransport);
}

void EMANE::NEMStatefulLayer::changeState(NEMLayerState * pState)
{
  pState_ = pState;
}

void EMANE::NEMStatefulLayer::processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg)
{
  pState_->processTimedEvent(this, pLayer_.get(), taskType, eventId, tv, arg);
}

