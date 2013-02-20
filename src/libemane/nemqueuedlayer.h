/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANENEMQUEUEDLAYER_HEADER_
#define EMANENEMQUEUEDLAYER_HEADER_

#include "emane/emanenemlayer.h"

#include "emaneutils/functor.h"

#include <queue>

#include <ace/Basic_Types.h>
#include <ace/Time_Value.h>
#include <ace/Thread.h>
#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>

namespace EMANE
{
  /**
   * @class NEMQueuedLayer
   *
   * @brief A layer stack with a porcessing queue between
   * each layer to decouple to intra queue processing
   *
   * @note Transport processing is deferred using function objects.  A
   * processing thread is then used to work the function object queue
   * processing packets, control, and events in a thread safe sequential
   * manner.
   */
  class NEMQueuedLayer :  public NEMLayer
  {
  public:
    ~NEMQueuedLayer();

    void start()
      throw(StartException);
    
    void stop()
      throw(StopException);
   
   /**
    *
    * @brief processDownstreamControl
    *
    * @param msg reference to the ControlMessage
    *
    */
    void processDownstreamControl(const ControlMessage & msg);
    
   /**
    *
    * @brief processDownstreamPacket
    *
    * @param pkt reference to the DownstreamPacket
    * @param msg reference to the ControlMessage
    *
    */
    void processDownstreamPacket(DownstreamPacket &pkt, const ControlMessage & msg);
    
   /**
    *
    * @brief processUpstreamPacket
    *
    * @param pkt reference to the UpstreamPacket
    * @param msg reference to the ControlMessage
    *
    */
    void processUpstreamPacket(UpstreamPacket &pkt, const ControlMessage & msg);
    
   /**
    *
    * @brief processUpstreamControl
    *
    * @param msg reference to the ControlMessage
    *
    */
    void processUpstreamControl(const ControlMessage & msg);

   /**
    *
    * @brief processEvent
    *
    * @param eventId reference to the EventId
    * @param state reference to the EventObjectState
    *
    */
    void processEvent(const EventId &eventId, const EventObjectState &state);
    
   /**
    *
    * @brief processtimedEvent
    *
    * @param taskType  task type identifier
    * @param eventId   the scheduled eventId
    * @param tv        the scheduled absolute event time
    * @param arg       opaque data
    *
    */
    void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);

  protected:
    NEMQueuedLayer(NEMId id, PlatformServiceProvider * pPlatformService);
    
    virtual void doProcessDownstreamControl(const ControlMessage &) = 0;
    
    virtual void doProcessDownstreamPacket(DownstreamPacket &, const ControlMessage &) = 0;
    
    virtual void doProcessUpstreamPacket(UpstreamPacket &, const ControlMessage &) = 0;
    
    virtual void doProcessUpstreamControl(const ControlMessage &) = 0;

    virtual void doProcessEvent(const EventId &, const EventObjectState &) = 0;
    
    virtual void doProcessTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg) = 0;

  private:
    typedef std::queue<EMANEUtils::Functor<void> *> MessageProcessingQueue;
    ACE_thread_t thread_;
    MessageProcessingQueue queue_;
    ACE_Thread_Mutex mutex_;
    ACE_Condition<ACE_Thread_Mutex> cond_;
    bool bCancel_;
  
    NEMQueuedLayer(const NEMQueuedLayer &);
    
    ACE_THR_FUNC_RETURN processWorkQueue();

    void handleProcessDownstreamControl(const ControlMessage);
    
    void handleProcessDownstreamPacket(DownstreamPacket,
                                       const ControlMessage);
    
    void handleProcessUpstreamPacket(UpstreamPacket,
                                     const ControlMessage);
    
    void handleProcessUpstreamControl(const ControlMessage);

    void handleProcessEvent(const EventId, const EventObjectState);

    void handleProcessTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value tv, const void *arg);

  };
}

#endif //EMANENEMQUEUEDLAYER_HEADER_
