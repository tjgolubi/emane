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

#include "nemlayerstack.h"
#include "logservice.h"

#include <sstream>

EMANE::NEMLayerStack::NEMLayerStack(){}
    
EMANE::NEMLayerStack::~NEMLayerStack()
{
  NEMLayerList::iterator iter = nemLayerList_.begin();
  
  for(; iter !=  nemLayerList_.end(); ++iter)
    {
      delete (*iter); 
    }
}
    
void EMANE::NEMLayerStack::connectLayers(UpstreamTransport * pUpstreamTransport,DownstreamTransport * pDownstreamTransport)
{
  NEMLayerList::iterator iter = nemLayerList_.begin();

  UpstreamTransport * pCurrentUpstreamTransport = pUpstreamTransport;

  for(; iter !=  nemLayerList_.end(); ++iter)
    {
      pCurrentUpstreamTransport->setDownstreamTransport(*iter);
      
      (*iter)->setUpstreamTransport(pCurrentUpstreamTransport);

      pCurrentUpstreamTransport = *iter;
    }

  pCurrentUpstreamTransport->setDownstreamTransport(pDownstreamTransport);
  
  pDownstreamTransport->setUpstreamTransport(pCurrentUpstreamTransport);
}


/*   
 *   NEMNetworkAdapter (connectLayers UpstreamTransport * param)
 *   ... --
 *   MAC   |
 *   ...   | - stack addLayer, bottom of stack pushed first.
 *   ...   |     where, ... can be 0 or more shims
 *   PHY   |
 *   ... --
 *   NEMOTAAdapter     (connectLayers DownstreamTransport * param)
 */
void EMANE::NEMLayerStack::addLayer(NEMLayer * pNEMLayer)
{
  nemLayerList_.push_back(pNEMLayer);
}


size_t EMANE::NEMLayerStack::layerCount() const
{
   return nemLayerList_.size();
}


void EMANE::NEMLayerStack::initialize()
  throw(InitializeException)
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"NEMLayerStack::initialize");

  // NEMLayers initialized individually by the builder
}
    
void EMANE::NEMLayerStack::configure(const ConfigurationItems & items)
  throw(ConfigureException)
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"NEMLayerStack::configure items=%d", 
                                       static_cast<int>(items.size()));

  // NEMLayers configured individually by the builder
}
    
void EMANE::NEMLayerStack::start()
  throw(StartException)
{
  NEMLayerList::iterator iter = nemLayerList_.begin();

  for(; iter !=  nemLayerList_.end(); ++iter)
    {
      (*iter)->start(); 
    }
}

void EMANE::NEMLayerStack::postStart()
{
  NEMLayerList::iterator iter = nemLayerList_.begin();

  for(; iter !=  nemLayerList_.end(); ++iter)
    {
      (*iter)->postStart(); 
    }
}
    
void EMANE::NEMLayerStack::stop()
  throw(StopException)
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"NEMLayerStack::stop");

  NEMLayerList::iterator iter = nemLayerList_.begin();

  for(; iter !=  nemLayerList_.end(); ++iter)
    {
      (*iter)->stop(); 
    }
}
    
void EMANE::NEMLayerStack::destroy()
  throw()
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"NEMLayerStack::destory");

  NEMLayerList::iterator iter = nemLayerList_.begin();

  for(; iter !=  nemLayerList_.end(); ++iter)
    {
      (*iter)->destroy(); 
    } 
}
