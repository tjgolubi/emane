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

#ifndef EMNAEPLATFORMSERVICE_HEADER_
#define EMNAEPLATFORMSERVICE_HEADER_

#include "emane/emanecomponenttypes.h"
#include "emane/emaneplatformserviceprovider.h"
#include "timerservice.h"
#include "statisticsmanager.h"

namespace EMANE
{

  class PlatformServiceUser;

  /**
   * @class PlatformService
   *
   * @brief platform service 
   */
  class PlatformService : public PlatformServiceProvider
  {
   public:
     PlatformService(LogServiceProvider * pLogServicePrvider,
                     EventServiceProvider * pEventServiceProvider,
                     TimerService * pTimerService,
                     StatisticsManager* statsManager,
                     BuildId bid);

     ~PlatformService();

     /**
      *
      * set the platform service user
      *
      * @param  pPlatformServiceUser pointer to the PlatformServiceUser 
      *
      */

     void setPlatformServiceUser(PlatformServiceUser * pPlatformServiceUser);

    /**
     * cancel the timed event
     *
     * @param eventId  event id
     *
     */
     void cancelTimedEvent (long eventId);

    /**
     * schedule the timed event
     *
     * @param taskType   task id type
     * @param arg        task data
     * @param tvTimeOut  task schedule time
     * @param tvInterval optional task reschedule interval
     *
     * @return task id or -1 on failure
     *
     */
     long scheduleTimedEvent (ACE_UINT32 taskType, const void *arg, 
                              const ACE_Time_Value & tvTimeOut, 
                              const ACE_Time_Value & tvInterval = ACE_Time_Value::zero);

     /* random numbers */
     int getRandomNumber();

     int getRandomNumber (int iMinLimit, int iMaxLimit);

     float getRandomProbability ();

     void setRandomSeed(unsigned int seed);
  
     /* logging */ 
     void vlog(LogLevel level, const char *fmt, va_list ap);
    
     void log(LogLevel level, const char *fmt, ...);

     /* events */
     void sendEvent(PlatformId platformId, NEMId nemId, ComponentType type, const Event & event);
    
     void sendEvent(PlatformId platformId, NEMId nemId, ComponentType type, EventId eventId, const EventObjectState & state);

     /* statistics */
     void registerStatistic(std::string name,EMANE::Statistic *value);

     void unregisterStatistic(std::string name);
   private:
     PlatformServiceUser * pPlatformServiceUser_;

     LogServiceProvider * pLogServiceProvider_;

     EventServiceProvider * pEventServiceProvider_;

     TimerService * pTimerService_;

     StatisticsManager* statsManager_;

     BuildId buildId_;
  
     unsigned int randomSeed_;
   };
}


#endif //EMNAEPLATFORMSERVICE_HEADER_
