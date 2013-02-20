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


#include "platformservice.h"
#include "emane/emaneplatformserviceuser.h"

#include <ace/OS_NS_sys_time.h>
#include <ace/Guard_T.h>


EMANE::PlatformService::PlatformService(EMANE::LogServiceProvider * pLogServiceProvider,
                                        EMANE::EventServiceProvider * pEventServiceProvider,
                                        EMANE::TimerService * pTimerService,
                                        EMANE::StatisticsManager *statsManager,                                        EMANE::BuildId bid)
 :
 pPlatformServiceUser_(0),
 pLogServiceProvider_(pLogServiceProvider),
 pEventServiceProvider_(pEventServiceProvider),
 pTimerService_(pTimerService),
 statsManager_(statsManager),
 buildId_(bid),
 randomSeed_(ACE_OS::gettimeofday().usec())
{ }



EMANE::PlatformService::~PlatformService()
{ }


void EMANE::PlatformService::setPlatformServiceUser(PlatformServiceUser * pPlatformServiceUser)
{
   pPlatformServiceUser_ = pPlatformServiceUser;
}


/* timer service */
long EMANE::PlatformService::scheduleTimedEvent (ACE_UINT32 taskType, const void *arg, 
                                                 const ACE_Time_Value & tvTimeOut, const ACE_Time_Value & tvInterval)
{
   return pTimerService_->scheduleTimedEvent(taskType, arg, tvTimeOut, tvInterval, pPlatformServiceUser_);
}



void EMANE::PlatformService::cancelTimedEvent (long eventId)
{
   pTimerService_->cancelTimedEvent(eventId);
}


/* random numbers */
int EMANE::PlatformService::getRandomNumber ()
{
#ifdef HAVE_RAND_R_SEED_PASS_BY_PTR
  return ACE_OS::rand_r(&randomSeed_);
#else
  return ACE_OS::rand_r(randomSeed_);
#endif
}


float EMANE::PlatformService::getRandomProbability ()
{
  return (getRandomNumber() / static_cast < float >(RAND_MAX));
}


int EMANE::PlatformService::getRandomNumber (int iMinLimit, int iMaxLimit)
{
  return (getRandomNumber() % (iMaxLimit - iMinLimit + 1)) + iMinLimit;
}


void EMANE::PlatformService::setRandomSeed(unsigned int seed)
{
  randomSeed_ = seed;
}



/* logging */
void EMANE::PlatformService::vlog(LogLevel level, const char *fmt, va_list ap)
{
   pLogServiceProvider_->vlog(level,fmt,ap);   
}


void EMANE::PlatformService::log(LogLevel level, const char *fmt, ...)
{
   va_list ap;
    
   va_start(ap, fmt);
      
   vlog(level,fmt,ap);   
     
   va_end(ap);
}



/* events */
void EMANE::PlatformService::sendEvent(PlatformId platformId, NEMId nemId, ComponentType type, const Event & event)
{
   pEventServiceProvider_->sendEvent(platformId, nemId, type, event);
}


void EMANE::PlatformService::sendEvent(PlatformId platformId, NEMId nemId, ComponentType type, EventId eventId, const EventObjectState & state)
{
   pEventServiceProvider_->sendEvent(platformId, nemId, type, eventId, state);
}

void EMANE::PlatformService::registerStatistic(std::string name,EMANE::Statistic *value)
{
  statsManager_->registerStatistic(buildId_, name, value);
}
     
void EMANE::PlatformService::unregisterStatistic(std::string name)
{
  statsManager_->unregisterStatistic(buildId_, name);
}
