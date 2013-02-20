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

#ifndef EMANEEVENTSERVICE_HEADER_
#define EMANEEVENTSERVICE_HEADER_

#include "eventserviceexception.h"

#include "eventservicemessage.h"
#include "emane/emaneeventserviceprovider.h"
#include "emane/emaneplatformserviceuser.h"

#include "emaneutils/componenttypes.h"

#include <map>
#include <string>
#include <queue>

#include <ace/SOCK_Dgram_Mcast.h>
#include <ace/RW_Thread_Mutex.h>
#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>
#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>
#include <ace/Thread.h>


namespace EMANE
{
  struct QueueEntry
  {
    ACE_UINT8 *data;
    ssize_t len;
  };

  typedef std::queue<EMANE::QueueEntry> EventServiceQueue;  


  /**
   * @class EventService
   *
   * @brief Platform event service
   *
   * @details Realization of the EventService interface.  This class
   * handles the EventReceiver registration and event routing of both
   * platform component sourced and generated events.
   *
   */
  class EventService : public EventServiceProvider
  {
  public:
    EventService();
    ~EventService();

    /**
     * Send an event
     *
     * @param platformId Id of destination platform @c 0 for all platforms
     * @param nemId Id of destination NEM @c 0 for all NEMs in a platform
     * @param type Component target of destination
     * @param event Reference to the event object
     */
    void sendEvent(PlatformId    platformId,
                   NEMId         nemId, 
                   ComponentType type, 
                   const Event & event);

    /**
     * Send an event
     *
     * @param platformId Id of destination platform @c 0 for all platforms
     * @param nemId Id of destination NEM @c 0 for all NEMs in a platform
     * @param type Component target of destination
     * @param eventId The event id
     * @param state Reference to the event object state
     */
    void sendEvent(PlatformId    platformId,
                   NEMId         nemId, 
                   ComponentType type,
                   EventId       eventId,
                   const EventObjectState & state);


    bool registerEventServiceHandler(NEMId id, ComponentType type, EventServiceHandler * pEventServiceHandler);

    void unregisterEventServiceHandler(NEMId id, ComponentType type);

    /**
     * Open the event server channel
     *
     * @param platformId Platform id
     * @param eventGroupAddress Multicast group address of event service channel
     * @param eventServiceDevice the event service device name
     * @param loopbackEnable flag to enable multicast loopback
     */
    void open(PlatformId platformId, 
              const ACE_INET_Addr & eventGroupAddress, 
              const std::string & eventServiceDevice, 
              bool loopbackEnable)
      throw(EventServiceException);

  private:
    typedef std::map<ComponentType, EventServiceHandler *> EventServiceHandlerMap;   
                                                                                    
    typedef std::map<NEMId, EventServiceHandlerMap> NEMEventServiceHandlerMap;

    NEMEventServiceHandlerMap nemEventServiceHandlerMap_;
    ACE_SOCK_Dgram_Mcast mcast_;
    ACE_thread_t receiveThread_;
    ACE_thread_t processThread_;
    ACE_RW_Thread_Mutex rwmutex_;
    PlatformId platformId_;
    bool bOpen_;
    unsigned long ulCount_;
    ACE_THR_FUNC_RETURN receiveEventMessage();
    ACE_THR_FUNC_RETURN processEventMessage();
    ACE_INET_Addr eventGroupAddress_;
    std::string eventServiceDevice_;
    EventServiceQueue eventServiceQueue_;
 
    ACE_Thread_Mutex mutex_;
    ACE_Thread_Mutex queueMutex_;
    ACE_Condition<ACE_Thread_Mutex> cond_;
    bool bCancel_;
    ACE_INET_Addr addr_;
  };
  
  typedef ACE_Unmanaged_Singleton<EventService,ACE_Null_Mutex> EventServiceSingleton;
}

#endif //EMANEEVENTSERVICE_HEADER_
