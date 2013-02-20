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

#include "nemlayerstate.h"
#include "nemstatefullayer.h"
#include "logservice.h"

EMANE::NEMLayerState::NEMLayerState(const char * pzStateName):
  pzStateName_(pzStateName){}

EMANE::NEMLayerState::~NEMLayerState(){}
    
void EMANE::NEMLayerState::handleInitialize(NEMStatefulLayer *, NEMLayer *)
      throw(InitializeException)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer invalid %s transition in %s state",
                                       "initialize",
                                       getStateName().c_str()); 
}
    
void EMANE::NEMLayerState::handleConfigure(NEMStatefulLayer *, NEMLayer *, const ConfigurationItems &)
      throw(ConfigureException)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer invalid %s transition in %s state",
                                       "configure",
                                       getStateName().c_str()); 
}

void EMANE::NEMLayerState:: handleStart(NEMStatefulLayer *, NEMLayer *)
      throw(StartException)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer invalid %s transition in %s state",
                                       "start",
                                       getStateName().c_str()); 
}

void EMANE::NEMLayerState:: handlePostStart(NEMStatefulLayer *, NEMLayer *)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer invalid %s transition in %s state",
                                       "postStart",
                                       getStateName().c_str()); 
}

void EMANE::NEMLayerState::handleStop(NEMStatefulLayer *, NEMLayer *)
      throw(StopException)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer invalid %s transition in %s state",
                                       "stop",
                                       getStateName().c_str()); 
}
    
void EMANE::NEMLayerState::handleDestroy(NEMStatefulLayer *, NEMLayer *)
      throw()
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer invalid %s transition in %s state",
                                       "destroy",
                                       getStateName().c_str()); 
}

void EMANE::NEMLayerState::processDownstreamControl(NEMStatefulLayer *, NEMLayer *, const ControlMessage &)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer %s not valid in %s state",
                                       "processDownstreamControl",
                                       getStateName().c_str());
}
   
void EMANE::NEMLayerState::processDownstreamPacket(NEMStatefulLayer *, 
                                                   NEMLayer *,  
                                                   DownstreamPacket &, 
                                                   const ControlMessage &)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer %s not valid in %s state",
                                       "processDownstreamPacket",
                                       getStateName().c_str());
}
    
void EMANE::NEMLayerState::processUpstreamPacket(NEMStatefulLayer *, 
                                                 NEMLayer *, 
                                                 UpstreamPacket &,
                                                 const ControlMessage &)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer %s not valid in %s state",
                                       "processUpstreamPacket",
                                       getStateName().c_str());
}

void EMANE::NEMLayerState::processUpstreamControl(NEMStatefulLayer *, NEMLayer *, const ControlMessage &)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer %s not valid in %s state",
                                       "processUpstreamControl",
                                       getStateName().c_str());
}

void EMANE::NEMLayerState::processEvent(NEMStatefulLayer *,
                                        NEMLayer *,
                                        const EventId &,
                                        const EventObjectState &)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer %s not valid in %s state",
                                       "processEvent",
                                       getStateName().c_str()); 
}


void EMANE::NEMLayerState::processTimedEvent(NEMStatefulLayer *,
                                             NEMLayer *,
                                             ACE_UINT32, 
                                             long, 
                                             const ACE_Time_Value &, 
                                             const void *)
{
  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                       "NEMLayer %s not valid in %s state",
                                       "processTimedEvent",
                                       getStateName().c_str()); 
}


void EMANE::NEMLayerState::changeState(NEMStatefulLayer * pStatefulLayer,NEMLayerState * pState)
{
  pStatefulLayer->changeState(pState);
}

std::string EMANE::NEMLayerState::getStateName() const
{
  return pzStateName_;
}
