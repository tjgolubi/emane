/*
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANETIMERSERVICE_HEADER_
#define EMANETIMERSERVICE_HEADER_

#include "timerserviceexception.h"
#include "emane/emaneplatformserviceuser.h"

#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>
#include <ace/Thread_Mutex.h>
#include <ace/Timer_Queue_Adapters.h>
#include <ace/Timer_Heap.h>

#include <map>

namespace EMANE
{
  
  /**
   * @class TimerService
   *
   * @brief Platform timer service
   *
   * @details Realization of the TimerService interface.
   *
   */
  class TimerService
  {
  public:
    /**
     * @brief constructor
     *
     */
    TimerService();

    /**
     * @brief destructor
     *
     */
    ~TimerService();

    /**
     * @brief open the timer service
     *
     */
    void open()
      throw(TimerServiceException);

    /**
     * @brief close the timer service
     *
     */
    void close();

     /**
     * @brief cancel a timed event
     *
     * @param eventId  event id to cancel
     */
    void cancelTimedEvent (long eventId);


    /**
     * @brief schedule a timed event.
     *
     * @param taskType              identifier corresponding to the task type, used by handler
     *                              to determine what action to take.
     * @param arg                   opaque data.
     * @param tvTimeOut             the absolute time the event is scheduled to occur.
     * @param tvInterval            optional the interval time the event shall re-occur.
     * @param pPlatformServiceUser  reference to the PlatformServiceUser.
     *
     * @return identifier corresponding to the event id, or -1 on failure
     */
    long scheduleTimedEvent (ACE_UINT32 taskType, 
                             const void *arg, 
                             const ACE_Time_Value & tvTimeOut, 
                             const ACE_Time_Value & tvInterval,
                             EMANE::PlatformServiceUser *pPlatformServiceUser);

    protected:

    /**
     * @class ActiveTimerEventHandler
     *
     * @brief the timer service event handler
     */
    class ActiveTimerEventHandler : public ACE_Event_Handler
      {
        public:
        /**
         *
         * @param pMgr                  reference to the TimerService.
         * @param pPlatformServiceUser  reference to the PlatformServiceUser to be called back on.
         * @param taskType              identifier corresponding to the task type.
         * @param bOneShot              timer is either one-shot or re-occuring.
         */
          ActiveTimerEventHandler (TimerService *pMgr, 
                                   PlatformServiceUser * pPlatformServiceUser, 
                                   ACE_UINT32 taskType, 
                                   bool bOneShot) :
           pMgr_(pMgr),
           pPlatformServiceUser_(pPlatformServiceUser),
           taskType_(taskType),
           bOneShot_(bOneShot),
           eventId_(-1)
          { }

          ~ActiveTimerEventHandler ()
          { }


        /**
         *
         * @param eventId   set the event id for this event.
         *
         */
         void setEventId(long eventId) { eventId_ = eventId; };

        /**
         * @brief handle timeout
         *
         * @param tv     the absolute time the event was scheduled to occur.
         * @param arg    opaque data passed to the platofrm service event handler
         *
         * @return       returns 0 
         */
         virtual int handle_timeout (const ACE_Time_Value &tv, const void *arg)
          {
            // call back 
            pPlatformServiceUser_->processTimedEvent(taskType_, eventId_, tv, arg);

            // cleanup one shot timer 
            if(bOneShot_ == true)
             {
               // cleanup
               pMgr_->cancelTimedEvent(eventId_);
             }
     
            return 0;
          }

        private:
          ActiveTimerEventHandler () {};

          ActiveTimerEventHandler (const ActiveTimerEventHandler &);

          TimerService * pMgr_;

          PlatformServiceUser * pPlatformServiceUser_;

          ACE_UINT32 taskType_;

          bool bOneShot_;

          long eventId_;
      };

   private:
    bool bOpen_;

    typedef std::map<long, ActiveTimerEventHandler *> ActiveTimerEventHandlerMap;

    typedef ActiveTimerEventHandlerMap::iterator ActiveTimerEventHandlerMapIter;

    typedef ACE_Thread_Timer_Queue_Adapter < ACE_Timer_Heap > ActiveTimer;
  
    ActiveTimer activeTimer_;

    ActiveTimerEventHandlerMap activeTimerEventHandlerMap_;

    ACE_Thread_Mutex mutex_;

    /* helpers */
    void removeTimedEventHandler (long eventId);

    void addTimedEventHandler (long eventId, EMANE::TimerService::ActiveTimerEventHandler * e);
  };
  typedef ACE_Unmanaged_Singleton<TimerService,ACE_Null_Mutex> TimerServiceSingleton;
}

#endif //EMANETIMERSERVICE_HEADER_
