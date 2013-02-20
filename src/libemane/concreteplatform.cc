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

#include "concreteplatform.h"
#include "logservice.h"
#include "otamanager.h"
#include "timerservice.h"
#include "eventservice.h"
#include "componentmap.h"
#include "emaneutils/parameterconvert.h"
#include "eventserviceexception.h"

#include <sstream>
#include <iomanip>
#include <set>

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      {true,false, 1,"eventservicegroup",0,0,"Event service multicast endpoint"},
      {false,false,1,"eventservicedevice",0,0,"Event service multicast device"},
      {false,false, 1,"otamanagergroup"  ,0,0,"OTA Management mutlicast endpoint"},
      {false,false,1,"otamanagerdevice",0,0,"OTA Management multicast device"},
      {false,true,1,"otamanagerchannelenable","on",0,"OTA Management channel enable"},
      {false,true,1,"debugport","47000",0,"Port used for debug information"},
      {false,true,1,"debugportenable","off",0,"Debug Port enable"},
      {0,0,0,0,0,0,0},
    };
}

EMANE::ConcretePlatform::ConcretePlatform(EMANE::PlatformId id,
                                          EMANE::StatisticsManager* statsManager)
  : platformId_(id),
    statsManager_(statsManager)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

EMANE::ConcretePlatform::~ConcretePlatform()
{
  PlatformNEMMap::iterator iter =  platformNEMMap_.begin();

  for(;iter !=  platformNEMMap_.end(); ++iter)
    {
      delete iter->second;
    }  
}
    
void EMANE::ConcretePlatform::add(NEMId nemId, NEM * pNEM)
  throw(PlatformException)
{
  if(!platformNEMMap_.insert(std::make_pair(nemId,pNEM)).second)
    {
      std::stringstream sstream;
       
       sstream<<"ConcretePlatform "
              <<std::setw(3)
              <<std::setfill('0')
              <<platformId_
              <<": Multiple NEMs with id "
              <<nemId
              <<" detected."
              <<std::ends;
       
       throw PlatformException(sstream.str());
    }
}
    
void EMANE::ConcretePlatform::initialize()
  throw(InitializeException)
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::initialize");
}

void EMANE::ConcretePlatform::configure(const ConfigurationItems & items)
  throw(ConfigureException)
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::configure");
  Component::configure(items);
}

void EMANE::ConcretePlatform::start()
  throw(StartException)
{
  ACE_INET_Addr otaManagerGroupAddr;
  ACE_INET_Addr eventServiceGroupAddr;
  std::string otaManagerDevice;
  std::string eventServiceDevice;
  bool bOTAManagerChannelEnable = true;
  bool bHaveGroupAddr = false;
  bool bDebugPortEnable = true;
  ACE_UINT16 debugPort = 0;
  

  ConfigurationRequirements::iterator iter = configRequirements_.begin();

  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "eventservicegroup")
                {
                  eventServiceGroupAddr =
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINETAddr();
                    LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::start eventservicegroupaddr = %s",
                                                         eventServiceGroupAddr.get_host_addr());
                }
              else if(iter->first == "eventservicedevice")
                {
                  eventServiceDevice = iter->second.item_.getValue();
                  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::start eventservicedevice = %s",
                                                       eventServiceDevice.c_str());
                }
              else if(iter->first == "otamanagergroup")
                {
                  otaManagerGroupAddr =
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINETAddr();
                    LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::start otamanagergroupaddr = %s",
                                                         otaManagerGroupAddr.get_host_addr());
                    bHaveGroupAddr = true;
                  
                }
              else if(iter->first == "otamanagerdevice")
                {
                  otaManagerDevice = iter->second.item_.getValue();
                  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::start otamanagerdevice = %s",
                                                       otaManagerDevice.c_str());
                  
                }
              else if(iter->first == "otamanagerchannelenable")
                {
                  bOTAManagerChannelEnable = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();
                  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::start otamanagerchannelenable = %d",
                                                       bOTAManagerChannelEnable);
                }
              else if(iter->first == "debugportenable")
                {
                  bDebugPortEnable = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();
                  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::start debugportenable = %d",
                                                       bDebugPortEnable);
                }
              else if(iter->first == "debugport")
                {
                  debugPort =
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();
                    LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcretePlatform::start debugport = %u",
                                                         debugPort);
                  
                }
              
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream sstream;
              
              sstream<<"ConcretePlatform "
                     <<std::setw(3)
                     <<std::setfill('0')
                     <<platformId_
                     <<": Missing configuration item '"
                     <<iter->first
                     <<"'."
                     <<std::ends;
              
              throw StartException(sstream.str());
            }
        }
      if(bOTAManagerChannelEnable == true)
        {
          if(bHaveGroupAddr == true)
            {
              if(otaManagerDevice.empty())
                {
                  EMANE::OTAManagerSingleton::instance()->open(otaManagerGroupAddr, 0);
                }
              else
                {
                  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"OTA Manager Device set");
                  EMANE::OTAManagerSingleton::instance()->open(otaManagerGroupAddr, otaManagerDevice.c_str());
                }
            }
          else
            {
              std::stringstream sstream;
              
              sstream<<"ConcretePlatform "
                     <<std::setw(3)
                     <<std::setfill('0')
                     <<platformId_
                     <<": Missing configuration item 'otamanagergroup'.  "
                     <<"Required when 'otamanagerchannelenable' set to 'on'."
                     <<std::ends;
              
              throw StartException(sstream.str());
            }
        }
      else
        {
          LogServiceSingleton::instance()->log(DEBUG_LEVEL,"Not Starting Channel Device");
        }
      
      EMANE::ComponentMapSingleton::instance()->closeRegistration();

      try
        {
          EMANE::EventServiceSingleton::instance()->open(platformId_,
                                                         eventServiceGroupAddr, 
                                                         eventServiceDevice,
                                                         true);
        }
      catch(EventServiceException &e)
        {
          throw StartException(e.what());
        }

      if(bDebugPortEnable == true)
        {
          statServer_ = new EMANE::DebugPort(debugPort,statsManager_);
          statServer_->start();
        }
      
    }
  catch(StartException & exp)
    {
      throw; 
    }
   catch(std::exception & exp)
    {
       std::stringstream sstream;
       
       sstream<<"ConcretePlatform "
              <<std::setw(3)
              <<std::setfill('0')
              <<platformId_
              <<": "
              <<exp.what()
              <<std::ends;
       
       throw StartException(sstream.str());
    }

  try 
    {
      EMANE::TimerServiceSingleton::instance()->open();
    }
  catch(StartException & exp)
    {
      throw; 
    }

  PlatformNEMMap::iterator nemIter =  platformNEMMap_.begin();
  
  for(;nemIter !=  platformNEMMap_.end(); ++nemIter)
    {
      nemIter->second->start();
    }

  // check all platformendpoints and transportendpoints
  // associated with platform for duplicate endpoint
  // errors
  typedef std::set<ACE_INET_Addr> EndpointSet;
  EndpointSet endpointSet;
  EndpointSet duplicatedSet;
  
  nemIter =  platformNEMMap_.begin();
  
  for(;nemIter !=  platformNEMMap_.end(); ++nemIter)
    {
      ACE_INET_Addr platformEndpoint =
        nemIter->second->getPlatformEndpoint();

      std::pair<EndpointSet::iterator,bool> retval = 
        endpointSet.insert(platformEndpoint);
  
      if(!retval.second)
        {
          duplicatedSet.insert(platformEndpoint);
        }

      ACE_INET_Addr transportEndpoint =
        nemIter->second->getTransportEndpoint();
      
      retval = 
        endpointSet.insert(transportEndpoint);
  
      if(!retval.second)
        {
          duplicatedSet.insert(transportEndpoint);
        }
    }

  if(!duplicatedSet.empty())
    {
       std::stringstream sstream;
       
       sstream<<"ConcretePlatform "
              <<std::setw(3)
              <<std::setfill('0')
              <<platformId_
              <<": "
              <<"Duplicate endpoints detected.  "
              <<"All NEM 'platformendpoint' and 'transportendpoint' parameters must be unique."
              <<std::endl
              <<std::endl
              <<"Endpoints duplicated one or more times:"
              <<std::endl;

       EndpointSet::iterator iter = duplicatedSet.begin();

       for(;iter != duplicatedSet.end(); ++iter)
         {
           sstream<<" * "
                  <<iter->get_host_name()
                  <<":"
                  <<iter->get_port_number()
                  <<" ("
                  <<iter->get_host_addr()
                  <<":"
                  <<iter->get_port_number()
                  <<")"
                  <<std::endl;
         }

       sstream<<std::endl
              <<std::ends;
       
       throw StartException(sstream.str());
    }

}

void EMANE::ConcretePlatform::postStart()
{
  PlatformNEMMap::iterator nemIter =  platformNEMMap_.begin();
  
  for(;nemIter !=  platformNEMMap_.end(); ++nemIter)
    {
      nemIter->second->postStart();
    }
}

void EMANE::ConcretePlatform::stop()
  throw(StopException)
{
  statsManager_->dumpStatisticsToLog();

  EMANE::TimerServiceSingleton::instance()->close();

  PlatformNEMMap::iterator iter =  platformNEMMap_.begin();

  for(;iter !=  platformNEMMap_.end(); ++iter)
    {
      iter->second->stop();
    }
}

void EMANE::ConcretePlatform::destroy()
  throw()
{
  PlatformNEMMap::iterator iter =  platformNEMMap_.begin();

  for(;iter !=  platformNEMMap_.end(); ++iter)
    {
      iter->second->destroy();
    }
}
