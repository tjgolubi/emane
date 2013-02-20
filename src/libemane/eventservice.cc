/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "eventservice.h"
#include "logservice.h"

#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/recvcancelable.h"

#include <sstream>
#include <iostream>

EMANE::EventService::EventService():
  mcast_(ACE_SOCK_Dgram_Mcast::OPT_BINDADDR_NO), 
  receiveThread_(0),
  processThread_(0),
  platformId_(0),
  bOpen_(false),
  ulCount_(0),
  eventGroupAddress_(),
  eventServiceDevice_(""),
  cond_(mutex_),
  bCancel_(false){}

EMANE::EventService::~EventService()
{
  if(bOpen_)
    {
      mutex_.acquire();

      if(!bCancel_ && processThread_)
      {
        bCancel_ = true;
        cond_.signal();
      }

      mutex_.release();

      ACE_OS::thr_cancel(receiveThread_);

      ACE_OS::thr_join(receiveThread_,0,0);

      ACE_OS::thr_cancel(processThread_);

      ACE_OS::thr_join(processThread_,0,0);
    }
}



void EMANE::EventService::sendEvent(PlatformId platformId, NEMId nemId, ComponentType type, const Event & event)
{
  sendEvent(platformId, nemId, type, event.getEventId(), event.getObjectState());
}


void EMANE::EventService::sendEvent(PlatformId platformId, NEMId nemId, ComponentType type, EventId eventId, const EventObjectState &state)
{
  if(bOpen_)
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

      LogServiceSingleton::instance()->log(DEBUG_LEVEL,"Event %03hu EMANE::EventService::sendEvent", eventId);

      if(mcast_.send(iov,3) == -1)
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                               "EventService sendEvent unable to send event id:%hu for NEM:%hu\n",
                                               eventId,
                                               nemId);
        }
    }
  else
    {
      LogServiceSingleton::instance()->log(DEBUG_LEVEL,"Event %03hu EMANE::EventService::sendEvent, not open, drop", eventId);
    }
}




bool EMANE::EventService::registerEventServiceHandler(NEMId id, ComponentType type, EventServiceHandler * pEventServiceHandler)
{
  NEMEventServiceHandlerMap::iterator iterNEM;

  rwmutex_.acquire_write();

  if((iterNEM = nemEventServiceHandlerMap_.find(id)) != nemEventServiceHandlerMap_.end())
    {
      EventServiceHandlerMap::iterator iterEventServiceHandler;

      if((iterEventServiceHandler = iterNEM->second.find(type)) == iterNEM->second.end())
        {
          iterNEM->second.insert(std::make_pair(type, pEventServiceHandler));
        }
      else
        {
          rwmutex_.release();
          return false;
        }
    }
  else
    {
      std::pair<NEMEventServiceHandlerMap::iterator,bool> ret =
        nemEventServiceHandlerMap_.insert(std::make_pair(id, EventServiceHandlerMap()));
      
      if(ret.second)
        {
          ret.first->second.insert(std::make_pair(type,pEventServiceHandler));
        }
      else
        {
          rwmutex_.release();
          return false;
        }
    }
  
  rwmutex_.release();
  return true;
}

void EMANE::EventService::unregisterEventServiceHandler(NEMId id, ComponentType type)
{
  NEMEventServiceHandlerMap::iterator iterNEM;

  rwmutex_.acquire_write();

  if((iterNEM = nemEventServiceHandlerMap_.find(id)) != nemEventServiceHandlerMap_.end())
    {
      iterNEM->second.erase(type);
    }
  
  rwmutex_.release();
}

void EMANE::EventService::open(PlatformId platformId, 
                               const ACE_INET_Addr & eventGroupAddress, 
                               const std::string & eventServiceDevice, 
                               bool loopbackEnable)
  throw(EventServiceException)
{
  platformId_      =  platformId; 
  ACE_hthread_t threadHandle;
  int priority = 0;
  int policy = 0;
  
  if(bOpen_)
    { 
      if(eventGroupAddress_.get_port_number() != 
         eventGroupAddress.get_port_number())
        {
          std::stringstream sstream;
          sstream
            <<"eventservicegroup port mismatch. First open request port: "
            <<eventGroupAddress_.get_port_number()
            <<"'. "
            <<"Now requesting: "
            <<eventGroupAddress.get_port_number()
            <<"'."
            <<std::endl
            <<std::ends;
      
          throw EventServiceException(sstream.str());
        }

      // can't use strcmp here because get_host_addr is "non-reentrant",
      // see ace notes
      std::string ega1(eventGroupAddress_.get_host_addr());
      std::string ega2(eventGroupAddress.get_host_addr());
      if(ega1.compare(ega2) != 0)
        {
          std::stringstream sstream;
          sstream
            <<"eventservicegroup port mismatch. First open request address: "
            <<ega1
            <<"'. "
            <<"Now requesting: "
            <<ega2
            <<"'."
            <<std::endl
            <<std::ends;
      
          throw EventServiceException(sstream.str());
        }

      if(eventServiceDevice_.compare(eventServiceDevice) != 0)
        {
          std::stringstream sstream;
          sstream
            <<"eventservicedevice mismatch. First open request: "
            <<eventServiceDevice_
            <<"'. "
            <<"Now requesting: "
            <<eventServiceDevice
            <<"'."
            <<std::endl
            <<std::ends;
      
          throw EventServiceException(sstream.str());
        }      
    }
  else
    {
      bOpen_ = true;

      // Save params for consistency check on subsequent calls
      eventGroupAddress_.set(eventGroupAddress);
      eventServiceDevice_ = eventServiceDevice;
      const ACE_TCHAR* device = 
        eventServiceDevice_.empty() ? 0 : eventServiceDevice_.c_str();

      if(mcast_.open(eventGroupAddress_, device) == -1)
        {
          std::stringstream sstream;
          sstream
            <<"Platform Event Service: Unable to open Event Service socket: '"
            <<eventGroupAddress_.get_host_addr()
            <<":"
            <<eventGroupAddress_.get_port_number()
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
            <<eventServiceDevice_
            <<" does not exist or is not up."
            <<std::endl
            <<std::ends;
      
          throw EventServiceException(sstream.str());
        }
  
      if(mcast_.join(eventGroupAddress_,1,device) == -1)
        {
          std::stringstream sstream;
          sstream
            <<"Platform OTA Manager: Unable to join Event Service group: '"
            <<eventGroupAddress_.get_host_addr()
            <<":"
            <<eventGroupAddress_.get_port_number()
            <<"'."
            <<std::endl
            <<std::endl
            <<"Possible reason(s):"
            <<std::endl
            <<" * "
            <<eventGroupAddress_.get_host_addr()
            <<" is not a multicast address."
            <<std::endl
            <<std::ends;
      
          throw EventServiceException(sstream.str());
        }

      if(loopbackEnable == false)
        {
          if(eventGroupAddress_.get_type() == AF_INET) {
            if(mcast_.set_option(IP_MULTICAST_LOOP,0) == -1)
              {
                std::stringstream ssDescription;
                ssDescription
                  <<"unable to unset EventService group IP_MULTICAST_LOOP"
                  <<std::ends;
                throw EventServiceException(ssDescription.str()); 
              }
          }
          else if(eventGroupAddress_.get_type() == AF_INET6) {
            int loop = 0;
            if(mcast_.ACE_SOCK::set_option(IPPROTO_IPV6,IPV6_MULTICAST_LOOP,
                                           &loop,
                                           sizeof(loop)) == -1)
              {
                std::stringstream ssDescription;
                ssDescription
                  <<"unable to unset EventService group IPV6_MULTICAST_LOOP"
                  <<std::ends;
                throw EventServiceException(ssDescription.str()); 
              }
          }
        }

      EMANEUtils::spawn(*this,
                        &EMANE::EventService::receiveEventMessage,
                        &receiveThread_,
                        &threadHandle);

      EMANEUtils::spawn(*this,
                        &EMANE::EventService::processEventMessage,
                        &processThread_);

      ACE_OS::thr_getprio(threadHandle,priority,policy);

      if(policy == ACE_SCHED_RR)
      { 
        int retval = ACE_OS::thr_setprio(threadHandle,ACE_THR_PRI_FIFO_DEF + 1,ACE_SCHED_RR);

        if(retval != 0)
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"EventManager::open: Unable to set Real Time Priority");
        }
      }
    }
}

ACE_THR_FUNC_RETURN EMANE::EventService::receiveEventMessage()
{
  ACE_UINT8 buf[65536];
  ssize_t len = 0;
  struct QueueEntry entry;


  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"EventService::receiveEventMessage");
  
  while(1)
    {
      if((len =  EMANEUtils::recvCancelable(mcast_,buf,sizeof(buf),addr_)) > 0)
        {
          LogServiceSingleton::instance()->log(DEBUG_LEVEL,"EventService Packet Received len: %zd",len);

          entry.data = new ACE_UINT8[len];
          memcpy(entry.data,buf,len);
          entry.len = len;
          queueMutex_.acquire();
          eventServiceQueue_.push(entry);
          queueMutex_.release();
          // signal entry added
          cond_.signal();
        }
      else
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"EventService Packet Receive error");
          break;
        }
    }

  return 0;
}

ACE_THR_FUNC_RETURN EMANE::EventService::processEventMessage()
{

  struct QueueEntry entry;
  EventServiceHeader *pHeader;
  
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"EventService::processEventMessage");

  while(1)
  {
     // wait while queue is empty and not canceled
     while(eventServiceQueue_.empty() && !bCancel_) 
     {
       cond_.wait();
     }
     // woke up canceled
     if (bCancel_)
     {
       return 0;
     }

     queueMutex_.acquire();
     entry = eventServiceQueue_.front();
     eventServiceQueue_.pop();
     queueMutex_.release();

     if(static_cast<size_t>(entry.len) >= sizeof(EventServiceHeader))
     {
       pHeader = reinterpret_cast<EventServiceHeader *>(entry.data);              
       eventServiceHeaderToHost(pHeader);

     switch(pHeader->u16Id_)
     {
       case EVENTSERVICE_CLIENT_MSG:
       ++ulCount_;
       if(static_cast<size_t>(entry.len) == pHeader->u16Length_)
       {
         EventServiceClientMessage * pMsg =  reinterpret_cast<EventServiceClientMessage *>(pHeader->data_);
                     
         eventServiceClientMessageToHost(pMsg);
                      
         if(static_cast<size_t>(entry.len) - sizeof(EventServiceHeader) - sizeof(EventServiceClientMessage) ==  pMsg->u16Length_)
         {                        
            EventObjectState state(pMsg->data_, 
                                   pMsg->u16Length_,
                                   pMsg->u16PlatformId_, 
                                   pMsg->u16NEMId_, 
                                   static_cast<ComponentType>(pMsg->u16LayerType_));
                          
            rwmutex_.acquire_read();
                          
            if(!pMsg->u16PlatformId_ || pMsg->u16PlatformId_ == platformId_) 
            {
              if(pMsg->u16NEMId_)
              {
                NEMEventServiceHandlerMap::iterator iterNEM;
                                  
                if((iterNEM = nemEventServiceHandlerMap_.find(pMsg->u16NEMId_)) != nemEventServiceHandlerMap_.end())
                {
                  EventServiceHandlerMap::iterator iterEventServiceHandler =  iterNEM->second.begin();
                                      
                  for(;iterEventServiceHandler != iterNEM->second.end(); ++iterEventServiceHandler)
                  {
                     // if pMsg->u16LayerType_ is 0, send event to all NEM layers, otherwise
                     // only send the event to the NEM layers with matching layer type
                     //
                     // note: mathing type test is ~bit~ test to allow targeting multiple
                     // layers at once
                     if(!pMsg->u16LayerType_ || (iterEventServiceHandler->first & pMsg->u16LayerType_))
                     {
                          iterEventServiceHandler->second->processEvent(pMsg->u16EventId_,state);
                     }
                  }
                }
              }
              else
              {
                 // iterate through all NEMs and send event to all matching layers
                 NEMEventServiceHandlerMap::iterator iterNEM =  nemEventServiceHandlerMap_.begin();
                                  
                 for(; iterNEM != nemEventServiceHandlerMap_.end(); ++iterNEM)
                 {
                    EventServiceHandlerMap::iterator iterEventServiceHandler = iterNEM->second.begin();
                                      
                    for (; iterEventServiceHandler != iterNEM->second.end(); ++iterEventServiceHandler)
                    {
                      // if pMsg->u16LayerType_ is 0, send event to all NEM layers, otherwise
                      // only send the event to the NEM layers with matching layer type
                      //
                      // note: mathing type test is ~bit~ test to allow targeting multiple
                      // layers at once
                      if(!pMsg->u16LayerType_ || (iterEventServiceHandler->first & pMsg->u16LayerType_))
                      {
                        iterEventServiceHandler->second->processEvent(pMsg->u16EventId_,state);
                      }
                    }
                  }
                }
              }
                          
              rwmutex_.release(); 
              }
              else
              {
                LogServiceSingleton::instance()->log(ERROR_LEVEL,"EventService EventServiceStandardMsg length mismatch"); 
              }
            }
            else
            {
              LogServiceSingleton::instance()->log(ERROR_LEVEL,"EventService Packet Received length mismatch not EVENTSERVICE_STANDARDMSG");
            }
            break;
            default:
               LogServiceSingleton::instance()->log(ERROR_LEVEL,"EventService Packet Received unknown message: %hu",pHeader->u16Id_);
               break;
          }
         delete[] entry.data;
     }
     else
     {
       LogServiceSingleton::instance()->log(ERROR_LEVEL,"EventService Packet Received length too small to be an EventService");
     }
  }
  return 0;
}
