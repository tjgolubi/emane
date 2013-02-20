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

#ifndef CONCRETEEVENTAGENTMANAGER_HEADER_
#define CONCRETEEVENTAGENTMANAGER_HEADER_

#include "emane/emaneeventagentmanager.h"

#include <map>
#include <set>

#include <ace/SOCK_Dgram_Mcast.h>
#include <ace/RW_Thread_Mutex.h>
#include <ace/Thread.h>

namespace EMANE
{
  /**
   * @class ConcreteEventAgentManager
   *
   * @brief Implementation of EventAgentManager - manage all event agents
   */
  class ConcreteEventAgentManager : public EMANE::EventAgentManager
  {
  public:
    ConcreteEventAgentManager();

    ~ConcreteEventAgentManager();
    
    void add(EventAgent  * pAgent);

    size_t agentCount() const;

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

  private:
    typedef std::multimap<EventId, EventAgent *> AgentRegistrationMap;
    typedef std::set<EventAgent *>  EventAgentSet;

    ACE_thread_t receiveThread_;    
    ACE_SOCK_Dgram_Mcast mcast_;
    ACE_RW_Thread_Mutex rwmutex_;
    ACE_INET_Addr eventServiceAddr_;
    AgentRegistrationMap  agentRegistrationMap_;
    EventAgentSet eventAgentSet_;
    ACE_INET_Addr addr_;
    
    /** 
     * Process incoming events.  Listening to the event multicast 
     * channel process all incoming events that are currently registered
     * for my one or more agents.
     *
     */
    ACE_THR_FUNC_RETURN processIncomingEvents(void);
  };
}

#endif // CONCRETEEVENTAGENTMANAGER_HEADER_
