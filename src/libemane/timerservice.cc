/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "timerservice.h"
#include "logservice.h"


EMANE::TimerService::TimerService() : bOpen_(false)
{ }


EMANE::TimerService::~TimerService()
{ }


void EMANE::TimerService::open()
      throw(TimerServiceException)
{
  if(!bOpen_)
    {
      bOpen_ = true;
      activeTimer_.activate ();
    }
}


void EMANE::TimerService::close()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  // cleanup outstanding tasks
  for(ActiveTimerEventHandlerMapIter iter = activeTimerEventHandlerMap_.begin();
      iter !=  activeTimerEventHandlerMap_.end(); ++iter)
    {
      // cancel event
      activeTimer_.cancel(iter->first);


      // cleaup
      delete iter->second;
    }

  // clear all entries
  activeTimerEventHandlerMap_.clear();

  // deactivate
  activeTimer_.deactivate ();

  // wait
  activeTimer_.wait ();
}


void EMANE::TimerService::cancelTimedEvent (long eventId)
{
  // cancel event
  activeTimer_.cancel (eventId);

  // remove event info
  removeTimedEventHandler(eventId);
}



long EMANE::TimerService::scheduleTimedEvent (ACE_UINT32 eventType, 
                                              const void * arg, 
                                              const ACE_Time_Value & tvTimeOut, 
                                              const ACE_Time_Value & tvInterval,
                                              EMANE::PlatformServiceUser * pPlatformServiceUser)
{
  // create active timer event handler
  EMANE::TimerService::ActiveTimerEventHandler * e = 
    new EMANE::TimerService::ActiveTimerEventHandler(this, 
                                                     pPlatformServiceUser, 
                                                     eventType, 
                                                     tvInterval == ACE_Time_Value::zero);

  // schedule the event
  const long eventId = activeTimer_.schedule (e, arg, tvTimeOut, tvInterval);

  // success
  if(eventId >= 0)
   {
     // add event info
     addTimedEventHandler(eventId, e);
   }
  // failure
  else
   {
     // cleaup
     delete e;
   }

  return eventId;
}



void EMANE::TimerService::removeTimedEventHandler (long eventId)
{
  // guard
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  // search for event id
  ActiveTimerEventHandlerMapIter iter = activeTimerEventHandlerMap_.find(eventId);

  if(iter != activeTimerEventHandlerMap_.end())
    {
      // cleaup
      delete iter->second;

      // remove from map
      activeTimerEventHandlerMap_.erase(iter);
    }
}


void EMANE::TimerService::addTimedEventHandler (long eventId, EMANE::TimerService::ActiveTimerEventHandler * e)
{
  // guard
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  // set the event id
  e->setEventId(eventId);

  // add to map
  activeTimerEventHandlerMap_.insert(std::make_pair(eventId, e));
}

