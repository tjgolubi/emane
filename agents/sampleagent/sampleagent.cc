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

#include "sampleagent.h"
#include "emaneevents/locationevent.h"
#include "emaneevents/pathlossevent.h"
#include "emaneutils/spawnmemberfunc.h"

#include <ace/OS_NS_sys_time.h>

namespace
{
  const EMANE::ConfigurationDefinition defs[] =
    {
      {0,0,0,0,0,0,0},
    };
}

EMANE::sampleAgent::sampleAgent(EMANE::NEMId nemId, EMANE::PlatformServiceProvider *pPlatformService):
  EventAgent(nemId, pPlatformService),
  cond_(mutex_),
  bCancel_(false)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

EMANE::sampleAgent::~sampleAgent(){}

void EMANE::sampleAgent::initialize()
  throw(EMANE::InitializeException){}

void EMANE::sampleAgent::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  Component::configure(items);
}

void EMANE::sampleAgent::start()
  throw(EMANE::StartException)
{
  EMANEUtils::spawn(*this,&sampleAgent::svc,&thread_);
}

void EMANE::sampleAgent::postStart()
  throw(EMANE::StartException){}

void EMANE::sampleAgent::stop()
  throw(EMANE::StopException)
{
  if(thread_)
    {
      mutex_.acquire();
      bCancel_ = true;
      cond_.signal();
      mutex_.release();
      ACE_OS::thr_join(thread_,0,0);
    }
}

void EMANE::sampleAgent::destroy()
  throw(){}

void EMANE::sampleAgent::processEvent(const EMANE::EventId & eventId,
                                      const EMANE::EventObjectState & state)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"sampleAgent::processEvent Called");

  if(eventId == LocationEvent::EVENT_ID)
    {
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"LocationEvent Received");

      pPlatformService_->log(EMANE::DEBUG_LEVEL,"Platform: %u, NEM: %u, Component: %u",
                             state.getPlatformId(),state.getNEMId(),state.getComponentType());
      updatePathlossStatistics(state.getNEMId());
    }
  if(eventId == PathlossEvent::EVENT_ID)
    {
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"PathlossEvent Received");
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"Platform: %u, NEM: %u, Component: %u",
                             state.getPlatformId(),state.getNEMId(),state.getComponentType());
      updateLocationStatistics(state.getNEMId());
    }

  if(eventId == SampleAgentEvent::EVENT_ID)
    {
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"sampleAgent Received");
    }
}

void EMANE::sampleAgent::processTimedEvent(ACE_UINT32,
                                    long,
                                    const ACE_Time_Value &,
                                    const void *){}

EMANE::EventRegistrationList EMANE::sampleAgent::getEventRegistrationList()
{
  EMANE::EventId LocationId = LocationEvent::EVENT_ID;
  EMANE::EventId PathlossId = PathlossEvent::EVENT_ID;
  EMANE::EventRegistrationList regList;
  regList.push_back(LocationId);
  regList.push_back(PathlossId);
  return regList;
}

ACE_THR_FUNC_RETURN EMANE::sampleAgent::svc()
{
  ACE_Time_Value tvWait = ACE_OS::gettimeofday();
  const ACE_Time_Value tvDelta(5,0);
  int iRet = 0;

  while(1)
    {
      mutex_.acquire();
      iRet = 0;

      tvWait += tvDelta;

      while(!bCancel_ && iRet != -1)
      {
        iRet = cond_.wait(&tvWait);
      }

      if(bCancel_)
      {
          mutex_.release();
          break;
      }
      mutex_.release();
      int size = eventsReceivedHistory_.size();
      if(size > 0)
      {
        eventHistory::iterator histPos = eventsReceivedHistory_.begin();
        SampleAgentEvent::SampleAgentEntry Events[size];

        for(int i=0; i < size; i++,histPos++)
        {
          Events[i] = histPos->second;           
        }
        SampleAgentEvent SampleEvent(&Events[0],size);

        pPlatformService_->sendEvent(0,
                                     0,
                                     EMANE::COMPONENT_ALL,
                                     SampleEvent);
      }

    }
  return 0;
}

void EMANE::sampleAgent::updatePathlossStatistics(EMANE::NEMId nemId)
{
  eventHistory::iterator histPos;

  // Look to see if we already have values for this NEM
  histPos = eventsReceivedHistory_.find(nemId);

  if(histPos != eventsReceivedHistory_.end())
    {
      // NEM found, update pathloss
      histPos->second.pathlossEventsReceived_++;
    }
  else
    {
      SampleAgentEvent::SampleAgentEntry newEntry;

      newEntry.u16Node_ = nemId;
      newEntry.pathlossEventsReceived_ = 1;
      newEntry.locationEventsReceived_ = 0;

      eventsReceivedHistory_.insert(std::make_pair(nemId,newEntry));
    }
}

void EMANE::sampleAgent::updateLocationStatistics(EMANE::NEMId nemId)
{
  eventHistory::iterator histPos;

  // Look to see if we already have values for this NEM
  histPos = eventsReceivedHistory_.find(nemId);

  if(histPos != eventsReceivedHistory_.end())
    {
      // NEM found, update pathloss
      histPos->second.locationEventsReceived_++;
    }
  else
    {
      SampleAgentEvent::SampleAgentEntry newEntry;

      newEntry.u16Node_ = nemId;
      newEntry.pathlossEventsReceived_ = 0;
      newEntry.locationEventsReceived_ = 1;

      eventsReceivedHistory_.insert(std::make_pair(nemId,newEntry));
    }
}

DECLARE_EVENT_AGENT(EMANE::sampleAgent);
