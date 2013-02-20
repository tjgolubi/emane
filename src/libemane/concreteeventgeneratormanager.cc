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

#include "concreteeventgeneratormanager.h"
#include "eventservicemessage.h" 
#include "eventserviceexception.h"
#include "emane/emaneeventreceiver.h"
#include "emaneutils/parameterconvert.h"
#include "logservice.h"
#include "eventservice.h"
#include "timerservice.h"

#include "emaneutils/spawnmemberfunc.h"
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

EMANE::ConcreteEventGeneratorManager::ConcreteEventGeneratorManager() :
  mcast_(ACE_SOCK_Dgram_Mcast::OPT_BINDADDR_NO),
  thread_(0)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

EMANE::ConcreteEventGeneratorManager::~ConcreteEventGeneratorManager()
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"EMANE::ConcreteEventServiceManager::~ConcreteEventServiceManager");
  
  EventGeneratorList::iterator iter =  eventGeneratorList_.begin();
  
  for(;iter !=  eventGeneratorList_.end(); ++iter)
    {
      delete *iter;
    }  
}


void EMANE::ConcreteEventGeneratorManager::add(EventGenerator * pGenerator)
{
  eventGeneratorList_.push_back(pGenerator);
}

   
size_t EMANE::ConcreteEventGeneratorManager::generatorCount() const
{
  return eventGeneratorList_.size();
}  


void EMANE::ConcreteEventGeneratorManager::initialize()
  throw(InitializeException)
{
  EventGeneratorList::iterator iter =  eventGeneratorList_.begin();

  for(;iter !=  eventGeneratorList_.end(); ++iter)
    {
      (*iter)->initialize();
    } 
}

void EMANE::ConcreteEventGeneratorManager::configure(const ConfigurationItems & items)
  throw(ConfigureException)
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcreteEventGeneratorManager::configure");
  Component::configure(items);
}

void EMANE::ConcreteEventGeneratorManager::start()
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
                    LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcreteEventGeneratorManager::start Event Service Group: %s",
                                                       eventServiceGroupAddr.get_host_addr());
                }
              else if(iter->first == "eventservicedevice")
                {
                  eventServiceDevice = iter->second.item_.getValue();
                  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcreteEventGeneratorManager::start Event Service Device: %s"
                                                   ,eventServiceDevice.c_str());
                }
                  
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"ConcreteEventGeneratorManager: Missing Configuration item "<<iter->first<<std::ends;
              throw StartException(ssDescription.str());
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"ConcreteEventGeneratorManager: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }

  if(mcast_.open(eventServiceGroupAddr,eventServiceDevice.empty() ? 0 : eventServiceDevice.c_str()) == -1)
    {
       std::stringstream sstream;
          
      sstream<<"Event Generator Manger: Unable to open Event Service socket: '"
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
      
      sstream<<"Event Generator Manger: Unable to join Event Service group: '"
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
  
  try
    {
      EMANE::EventServiceSingleton::instance()->open(
                  0,
                  eventServiceGroupAddr,
                  eventServiceDevice, 
                  true);
    }
  catch(EventServiceException &e)
    {
      throw StartException(e.what());
    }


  try 
    {
      EMANE::TimerServiceSingleton::instance()->open();
    }
  catch(StartException & exp)
    {
      throw; 
    }


  EventGeneratorList::iterator iterGen =  eventGeneratorList_.begin();

  for(;iterGen !=  eventGeneratorList_.end(); ++iterGen)
    {
      (*iterGen)->start();
    }

  EMANEUtils::spawn(*this,&ConcreteEventGeneratorManager::svc,&thread_);
  
}

void EMANE::ConcreteEventGeneratorManager::postStart()
{
  EventGeneratorList::iterator iter =  eventGeneratorList_.begin();

  for(;iter !=  eventGeneratorList_.end(); ++iter)
    {
      (*iter)->postStart();
    }
}

void EMANE::ConcreteEventGeneratorManager::stop()
  throw(StopException)
{

  if(thread_)
    {
      ACE_OS::thr_cancel(thread_);

      ACE_OS::thr_join(thread_,0,0);
    }

  EventGeneratorList::iterator iter =  eventGeneratorList_.begin();

  for(;iter !=  eventGeneratorList_.end(); ++iter)
    {
      (*iter)->stop();
    }
}

void EMANE::ConcreteEventGeneratorManager::destroy()
  throw()
{
  EventGeneratorList::iterator iter =  eventGeneratorList_.begin();

  for(;iter !=  eventGeneratorList_.end(); ++iter)
    {
      (*iter)->destroy();
    }
}


void EMANE::ConcreteEventGeneratorManager::sendEvent(PlatformId    platformId,
                                                     NEMId         nemId, 
                                                     ComponentType type,
                                                     const Event & event)
{
  sendEvent(platformId, nemId, type, event.getEventId(), event.getObjectState());
}


void EMANE::ConcreteEventGeneratorManager::sendEvent(PlatformId    platformId,
                                                     NEMId         nemId, 
                                                     ComponentType type,
                                                     EventId       eventId,
                                                     const EventObjectState & state)
{
  EventServiceHeader header;
  EventServiceClientMessage msg;

  iovec iov[3];
  
  iov[0].iov_base = reinterpret_cast<char *>(&header);
  iov[0].iov_len  = sizeof(header);
  
  iov[1].iov_base = reinterpret_cast<char *>(&msg);
  iov[1].iov_len  = sizeof(msg);
  
  iov[2].iov_base = const_cast<char *>(reinterpret_cast<const char *>(state.get()));
  iov[2].iov_len  = state.length();
      
  header.u16Id_     = EVENTSERVICE_CLIENT_MSG;
  header.u16Length_ = sizeof(header) + sizeof(msg) + state.length();
  
  msg.u16PlatformId_ = platformId;
  msg.u16NEMId_      = nemId;
  msg.u16LayerType_  = type;
  msg.u16EventId_    = eventId;
  msg.u16Length_     = state.length();
  
  eventServiceHeaderToNet(&header);
  eventServiceClientMessageToNet(&msg);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"Event %03hu EMANE::ConcreteEventServiceManager::sendEvent", eventId);

  if(mcast_.send(iov,3) == -1)
    {
      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                           "ConcreteEventGeneratorManager sendEvent unable to send event id:%hu for NEM:%hu\n",
                                           eventId,
                                           nemId);
    }
}


void EMANE::ConcreteEventGeneratorManager::registerEventHandler(EventId eventId, EventReceiver * pEventReceiver)
{
  rwmutex_.acquire_write();
  eventRegistrationMap_.insert(std::make_pair(eventId,pEventReceiver));
  rwmutex_.release();
}

 ACE_THR_FUNC_RETURN EMANE::ConcreteEventGeneratorManager::svc()
{
  ACE_UINT8 buf[65536];
  ssize_t len = 0;

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcreteEventGeneratorManager::svc");
  
  while(1)
    {
      if((len =  EMANEUtils::recvCancelable(mcast_,buf,sizeof(buf),addr_)) > 0)
        {
          LogServiceSingleton::instance()->log(DEBUG_LEVEL,"ConcreteEventGeneratorManager Packet Received len: %zd",len);
      
          if(static_cast<size_t>(len) >= sizeof(EventServiceHeader))
            {
              EventServiceHeader * pHeader = reinterpret_cast<EventServiceHeader *>(buf);
          
              eventServiceHeaderToHost(pHeader);
                             
              switch(pHeader->u16Id_)
                {
                case EVENTSERVICE_SERVER_MSG:

                  if(static_cast<size_t>(len) == pHeader->u16Length_)
                    {
                      EventServiceClientMessage * pMsg =  reinterpret_cast<EventServiceClientMessage *>(pHeader->data_);
             
                      eventServiceClientMessageToHost(pMsg);
     
                      if(static_cast<size_t>(len) - sizeof(EventServiceHeader) - sizeof(EventServiceClientMessage) ==  pMsg->u16Length_)
                        {
                          EventRegistrationMap::iterator iter;
                          std::pair<EventRegistrationMap::iterator, EventRegistrationMap::iterator> ret;

                          EventObjectState state(pMsg->data_, pMsg->u16Length_);

                          rwmutex_.acquire_read();
                      
                          ret = eventRegistrationMap_.equal_range(pMsg->u16EventId_);

                          for(iter = ret.first; iter !=ret.second; ++iter)
                            {
                              iter->second->handleEvent(pMsg->u16EventId_,state);
                            }

                          rwmutex_.release();
                        }
                      else
                        {
                          LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventServiceManager EventServiceClientMessage length mismatch"); 
                        }
                    }
                  else
                    {
                      LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventServiceManager Packet Received length mismatch not EventServiceClientMessage");
                    }
                  break;
                case EVENTSERVICE_CLIENT_MSG:
                  break;
                default:
                  LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventGeneratorManager Packet Received unknown message: %hu",pHeader->u16Id_);
                  break;
                }
            }
          else
            {
              LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventGeneratorManager Packet Received length too small to be an EventService");
            }
        }
      else
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"ConcreteEventGeneratorManager Packet Receive error");
          break;
        }
    }

  return 0;
}
