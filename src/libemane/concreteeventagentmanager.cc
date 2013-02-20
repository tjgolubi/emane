/*
 * Copyright (c) 2011-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "concreteeventagentmanager.h"
#include "eventservicemessage.h"
#include "logservice.h"
#include "timerservice.h"
#include "eventservice.h"
#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/parameterconvert.h"
#include "emaneutils/recvcancelable.h"

#include <sstream>

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      {true,false, 1,"eventservicegroup",0,0,"Event service multicast endpoint"},
      {false,false,1,"eventservicedevice",0,0,"Event service multicast device"},
      {0,0,0,0,0,0,0},
    };
}

EMANE::ConcreteEventAgentManager::ConcreteEventAgentManager():
  receiveThread_(0),
  mcast_(ACE_SOCK_Dgram_Mcast::OPT_BINDADDR_NO)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

EMANE::ConcreteEventAgentManager::~ConcreteEventAgentManager()
{
  if(receiveThread_)
    {
      ACE_OS::thr_cancel(receiveThread_);

      ACE_OS::thr_join(receiveThread_,0,0);
    }

  EventAgentSet::iterator iter =  eventAgentSet_.begin();

  for(;iter !=  eventAgentSet_.end(); ++iter)
    {
      delete *iter;
    }
}

void EMANE::ConcreteEventAgentManager::initialize()
  throw(InitializeException)
{
  EventAgentSet::iterator iter =  eventAgentSet_.begin();

  for(;iter !=  eventAgentSet_.end(); ++iter)
    {
      (*iter)->initialize();
    } 
}

void EMANE::ConcreteEventAgentManager::configure(const ConfigurationItems & items)
  throw(ConfigureException)
{
  Component::configure(items);
}

void EMANE::ConcreteEventAgentManager::start()
  throw(StartException)
{
  ACE_INET_Addr eventServiceGroupAddr;
  std::string eventServiceDevice;

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
                }
              else if(iter->first == "eventservicedevice")
                {
                  eventServiceDevice = iter->second.item_.getValue();
                }
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"ConcreteEventAgentManager: Missing Configuration item "<<iter->first<<std::ends;
              throw StartException(ssDescription.str());
            }
        }
    }
    catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"ConcreteEventAgentManager: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }
  
  if(mcast_.open(eventServiceGroupAddr,eventServiceDevice.empty() ? 0 : eventServiceDevice.c_str()) == -1)
    {
      std::stringstream sstream;
      
      sstream<<"Event Agent Manager: Unable to open Event Service socket: '"
             <<eventServiceGroupAddr.get_host_addr()
             <<":"
             <<eventServiceGroupAddr.get_port_number()
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * No Multicast device specified and routing table nondeterministic"
             <<std::endl
             <<"   (no multicast route and no default route)."
             <<std::endl
             <<" * Multicast device "
             <<eventServiceDevice
             <<" does not exist or is not up."
             <<std::endl
             <<std::ends;
      
      throw StartException(sstream.str());
    }
  
  if(mcast_.join(eventServiceGroupAddr,1,eventServiceDevice.empty() ? 0 : eventServiceDevice.c_str()) == -1)
    {
      std::stringstream sstream;
      
      sstream<<"Event Agent Manager: Unable to join Event Service group: '"
             <<eventServiceGroupAddr.get_host_addr()
             <<":"
             <<eventServiceGroupAddr.get_port_number()
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<eventServiceGroupAddr.get_host_addr()
             <<" is not a multicast address."
             <<std::endl
             <<std::ends;
      
      throw StartException(sstream.str());
    }

  EventAgentSet::iterator iterGen =  eventAgentSet_.begin();

  try 
    {
      EMANE::TimerServiceSingleton::instance()->open();
      EMANE::EventServiceSingleton::instance()->open(0,
                                                     eventServiceGroupAddr, 
                                                     eventServiceDevice,
                                                     true);
    }
  catch(StartException & exp)
    {
      throw; 
    }

  for(;iterGen !=  eventAgentSet_.end(); ++iterGen)
    {
      (*iterGen)->start();
    }
  EMANEUtils::spawn(*this,&EMANE::ConcreteEventAgentManager::processIncomingEvents,&receiveThread_);
}

void EMANE::ConcreteEventAgentManager::postStart()
{
  EventAgentSet::iterator iter =  eventAgentSet_.begin();

  for(;iter !=  eventAgentSet_.end(); ++iter)
    {
      (*iter)->postStart();
    }
}

void EMANE::ConcreteEventAgentManager::stop()
  throw(StopException)
{
  EventAgentSet::iterator iter =  eventAgentSet_.begin();

  EMANE::TimerServiceSingleton::instance()->close();

  for(;iter !=  eventAgentSet_.end(); ++iter)
    {
      (*iter)->stop();
    }
}

void EMANE::ConcreteEventAgentManager::destroy()
  throw()
{
  EventAgentSet::iterator iter =  eventAgentSet_.begin();

  for(;iter !=  eventAgentSet_.end(); ++iter)
    {
      (*iter)->destroy();
    }
}

void EMANE::ConcreteEventAgentManager::add(EventAgent  * pAgent)
{
  EventRegistrationList regList = pAgent->getEventRegistrationList();
  EventRegistrationList::iterator regListIter = regList.begin();
  rwmutex_.acquire_write();
  eventAgentSet_.insert(pAgent);

  for(;regListIter != regList.end(); ++regListIter)
    {
      agentRegistrationMap_.insert(std::make_pair(*regListIter,pAgent));
    }
  
  rwmutex_.release();
}

size_t EMANE::ConcreteEventAgentManager::agentCount() const
{
  return eventAgentSet_.size();
}

ACE_THR_FUNC_RETURN EMANE::ConcreteEventAgentManager:: processIncomingEvents(void)
{
  ACE_UINT8 buf[65536];
  ssize_t len = 0;

  while(1)
    {
      if((len =  EMANEUtils::recvCancelable(mcast_,buf,sizeof(buf),addr_)) > 0)
        {
          LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcreteEventAgentManager Packet Received len: %zd",len);
      
          if(static_cast<size_t>(len) >= sizeof(EventServiceHeader))
            {
              EventServiceHeader * pHeader = reinterpret_cast<EventServiceHeader *>(buf);
          
              eventServiceHeaderToHost(pHeader);
                             
              switch(pHeader->u16Id_)
                {
                case EVENTSERVICE_SERVER_MSG:
                case EVENTSERVICE_CLIENT_MSG:

                  if(static_cast<size_t>(len) == pHeader->u16Length_)
                    {
                      EventServiceClientMessage * pMsg =  reinterpret_cast<EventServiceClientMessage *>(pHeader->data_);
             
                      eventServiceClientMessageToHost(pMsg);
     
                      if(static_cast<size_t>(len) - sizeof(EventServiceHeader) - sizeof(EventServiceClientMessage) ==  pMsg->u16Length_)
                        {
                          AgentRegistrationMap::iterator iter;
                          
                          std::pair<AgentRegistrationMap::iterator,AgentRegistrationMap::iterator> ret;

                          EventObjectState state(pMsg->data_, pMsg->u16Length_, pMsg->u16PlatformId_, pMsg->u16NEMId_, static_cast<ComponentType>(pMsg->u16LayerType_));

                          rwmutex_.acquire_read();
                      
                          ret = agentRegistrationMap_.equal_range(pMsg->u16EventId_);

                          for(iter = ret.first; iter !=ret.second; ++iter)
                            {
                               iter->second->processEvent(pMsg->u16EventId_,state);
                            }

                          rwmutex_.release();
                        }
                      else
                        {
                          LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventAgentManager EventServiceClientMessage length mismatch"); 
                        }
                    }
                  else
                    {
                      LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventAgentManager Packet Received length mismatch not EventServiceClientMessage");
                    }
                  break;
                default:
                  LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventAgentManager Packet Received unknown message: %hu",pHeader->u16Id_);
                  break;
                }
            }
          else
            {
              LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventAgentManager Packet Received length too small to be an EventService");
            }
        }
    }

  return 0;
}
