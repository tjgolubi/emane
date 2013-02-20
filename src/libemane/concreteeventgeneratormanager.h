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

#ifndef CONCRETEEVENTGENERATORMANAGER_HEADER_
#define CONCRETEEVENTGENERATORMANAGER_HEADER_

#include "emane/emaneeventgeneratormanager.h"
#include "emane/emanecomponent.h"
#include "emane/emaneeventgenerator.h"

#include <map>
#include <list>

#include <ace/SOCK_Dgram_Mcast.h>
#include <ace/RW_Thread_Mutex.h>
#include <ace/Task.h>

namespace EMANE
{
  /**
   * @class ConcreteEventGeneratorManager
   *
   * @brief Deployment event server 
   *
   * @details Realization of the EventGeneratorManager interface.  Allows for the 
   * registration and state management of event generators and event receivers.
   *
   */
  class ConcreteEventGeneratorManager : public EMANE::EventGeneratorManager
  {
  public:
    ConcreteEventGeneratorManager();

    ~ConcreteEventGeneratorManager();
    
    void sendEvent(PlatformId platformId, 
                   NEMId nemId, 
                   ComponentType type, 
                   const Event & event);

    void sendEvent(PlatformId platformId, 
                   NEMId nemId, 
                   ComponentType type, 
                   EventId eventId, 
                   const EventObjectState & state);

    void registerEventHandler(EventId eventId, EventReceiver * pEventReceiver);

    void initialize()
      throw(InitializeException);
    
    void configure(const ConfigurationItems & items)
      throw(ConfigureException);
    
    void start()
      throw(StartException);

    void postStart();
    
    void stop()
      throw(StopException);
    
    void destroy()
      throw();

    /**
     * Add and event generator
     * 
     * @param pEventGenerator EventGenerator to add
     */
    void add(EventGenerator * pEventGenerator);

    size_t generatorCount() const;
    
  private:
    typedef std::multimap<EventId, EventReceiver *> EventRegistrationMap;
    typedef std::list<EventGenerator *> EventGeneratorList;
    
    ACE_SOCK_Dgram_Mcast mcast_;
    ACE_RW_Thread_Mutex rwmutex_;
    ACE_INET_Addr eventServiceAddr_;
    EventRegistrationMap eventRegistrationMap_;
    EventGeneratorList eventGeneratorList_;
    
    ACE_thread_t thread_;
    ACE_INET_Addr addr_;

    ACE_THR_FUNC_RETURN svc();
  };
}

#endif //CONCRETEEMANEEVENTGENERATOR_HEADER_
