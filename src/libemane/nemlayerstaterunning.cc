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

#include "nemlayerstaterunning.h"
#include "nemlayerstatestopped.h"
#include "nemstatefullayer.h"

EMANE::NEMLayerStateRunning::NEMLayerStateRunning():
  NEMLayerState("RUNNING"){}

EMANE::NEMLayerStateRunning::~NEMLayerStateRunning(){}

void EMANE::NEMLayerStateRunning::handlePostStart(NEMStatefulLayer *, 
                                                  NEMLayer * pLayer)
{
  pLayer->postStart();
}

void EMANE::NEMLayerStateRunning::handleStop(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer)
      throw(StopException)
{
  pLayer->stop();
  changeState(pStatefulLayer,NEMLayerStateStoppedSingleton::instance());
}

void EMANE::NEMLayerStateRunning::processDownstreamControl(NEMStatefulLayer *, 
                                                           NEMLayer * pLayer, 
                                                           const ControlMessage & msg)
{
  pLayer->processDownstreamControl(msg);
}

void EMANE::NEMLayerStateRunning::processDownstreamPacket(NEMStatefulLayer *, 
                                                          NEMLayer * pLayer,  
                                                          DownstreamPacket & pkt,
                                                          const ControlMessage & msg)
{
  pLayer->processDownstreamPacket(pkt,msg);
}

void EMANE::NEMLayerStateRunning::processUpstreamPacket(NEMStatefulLayer *, 
                                                        NEMLayer * pLayer, 
                                                        UpstreamPacket & pkt,
                                                        const ControlMessage & msg)
{
  pLayer->processUpstreamPacket(pkt,msg);
}

void EMANE::NEMLayerStateRunning::processUpstreamControl(NEMStatefulLayer *, 
                                                         NEMLayer * pLayer, 
                                                         const ControlMessage & msg)
{
  pLayer->processUpstreamControl(msg);
}

void EMANE::NEMLayerStateRunning::processEvent(NEMStatefulLayer *,
                                               NEMLayer * pLayer,
                                               const EventId & id, 
                                               const EventObjectState & state)
{
  pLayer->processEvent(id,state);
}


void EMANE::NEMLayerStateRunning::processTimedEvent(NEMStatefulLayer *,
                                                    NEMLayer * pLayer,
                                                    ACE_UINT32 eventType,
                                                    long eventId,
                                                    const ACE_Time_Value &tv, 
                                                    const void *arg)
{
  pLayer->processTimedEvent(eventType, eventId, tv, arg);
}

