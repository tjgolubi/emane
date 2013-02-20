/*
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
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

#include "concretenem.h"
#include "logservice.h"
#include "emaneutils/parameterconvert.h"

#include <sstream>
#include <iomanip>

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      {false,false,1,"platformendpoint", 0,0,"Platform Server NEM endpoint address"},
      {false,false,1,"transportendpoint",0,0,"Transport endpoint address"},
      {false,false,1,"localaddress",     0,0,"NEM netadapter endpoint address (Deprecated)"},
      {false,false,1,"remoteaddress",    0,0,"NEM remote netadapter endpoint address (Deprecated)"},
      {0,0,0,0,0,0,0},
    };
}

EMANE::ConcreteNEM::ConcreteNEM(NEMId id, 
                                NEMLayerStack * pNEMLayerStack):
  pNEMLayerStack_(pNEMLayerStack),
  id_(id),
  NEMOTAAdapter_(id),
  NEMNetworkAdapter_(id)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);

  pNEMLayerStack_->connectLayers(&NEMNetworkAdapter_,&NEMOTAAdapter_);
}

EMANE::ConcreteNEM::~ConcreteNEM(){}
    
void EMANE::ConcreteNEM::initialize()
  throw(InitializeException)
{
  pNEMLayerStack_->initialize(); 
}
    
void EMANE::ConcreteNEM::configure(const ConfigurationItems & items)
  throw(ConfigureException)
{
  Component::configure(items);
}
    
void EMANE::ConcreteNEM::start()
  throw(StartException)
{
  bool bPlatformEndpointDefined = false;
  bool bTransportEndpointDefined = false;

  ConfigurationRequirements::iterator iter = configRequirements_.begin();

  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "localaddress"   ||
                 iter->first == "platformendpoint")
                {
                  bPlatformEndpointDefined = true;

                  if(iter->first == "localaddress")
                    {
                      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                                           "ConcreteNEM %03hu: Use of deprecated NEM parameter localaddress use platformendpoint",
                                                           id_);
                    }

                  platformEndpointAddr_ =
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINETAddr();
                }
              else if(iter->first == "remoteaddress" ||
                      iter->first == "transportendpoint")
                {
                  bTransportEndpointDefined = true;

                  if(iter->first == "remoteaddress")
                    {
                      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                                           "ConcreteNEM %03hu: Use of deprecated NEM parameter remoteaddress use transportendpoint",
                                                           id_);
                    }

                  transportEndpointAddr_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINETAddr();
                }
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
     
              ssDescription<<"ConcreteNEM "
                           <<std::setw(3)
                           <<std::setfill('0')
                           <<id_
                           <<": Missing configuration item "
                           <<iter->first
                           <<std::ends;

              throw StartException(ssDescription.str());
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;

      sstream<<"ConcreteNEM "
             <<std::setw(3)
             <<std::setfill('0')
             <<id_
             <<": Parameter "
             <<iter->first
             <<": "
             <<exp.what()
             <<std::ends;

      throw StartException(sstream.str());
    }

  if(!bPlatformEndpointDefined)
    {
      std::stringstream ssDescription;

      ssDescription<<"ConcreteNEM "
                   <<std::setw(3)
                   <<std::setfill('0')
                   <<id_
                   <<": Missing configuration item platformendpoint"
                   <<std::ends;

      throw StartException(ssDescription.str());
    }
  
  if(!bTransportEndpointDefined)
    {
      std::stringstream ssDescription;
      
      ssDescription<<"ConcreteNEM "
                   <<std::setw(3)
                   <<std::setfill('0')
                   <<id_
                   <<": Missing configuration item transportendpoint"
                   <<std::ends;
      
      throw StartException(ssDescription.str());
    }

   NEMOTAAdapter_.open();
   
   try
     {
       NEMNetworkAdapter_.open(platformEndpointAddr_,transportEndpointAddr_);
     }
   catch(NetworkAdapterException & exp)
     {
       throw StartException(exp.what());
     }

   pNEMLayerStack_->start();
}

void EMANE::ConcreteNEM::postStart()
{
  pNEMLayerStack_->postStart();
}
    
void EMANE::ConcreteNEM::stop()
  throw(StopException)
{
  NEMNetworkAdapter_.close();
  NEMOTAAdapter_.close();
  pNEMLayerStack_->stop(); 
}
    
void EMANE::ConcreteNEM::destroy()
  throw()
{
  pNEMLayerStack_->destroy();
}

EMANE::NEMId EMANE::ConcreteNEM::getNEMId() const
{
  return id_;
}

ACE_INET_Addr EMANE::ConcreteNEM::getPlatformEndpoint() const
{
  return platformEndpointAddr_;
}

ACE_INET_Addr EMANE::ConcreteNEM::getTransportEndpoint() const
{
  return transportEndpointAddr_;
}
