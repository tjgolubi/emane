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

#include "nemqueuedlayer.h"
#include "logservice.h"

#include "emaneutils/spawnmemberfunc.h"
#include "emane/emanecommonmacheaderexception.h"
#include "emane/emanecommonphyheaderexception.h"

#include <exception>

#include <ace/Guard_T.h>

EMANE::NEMQueuedLayer::NEMQueuedLayer(NEMId id, PlatformServiceProvider *pPlatformService):
  NEMLayer(id, pPlatformService),
  thread_(0),
  cond_(mutex_),
  bCancel_(false)
{}

EMANE::NEMQueuedLayer::~NEMQueuedLayer()
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"NEMQueuedLayer::~NEMQueuedLayer %03hu",id_);
  
  mutex_.acquire();

  if(!bCancel_ && thread_)
    {
      bCancel_ = true;
      cond_.signal();
    }

  mutex_.release();
}

void EMANE::NEMQueuedLayer::start() throw(StartException)
{
  EMANEUtils::spawn(*this,&EMANE::NEMQueuedLayer::processWorkQueue,&thread_);
}

void EMANE::NEMQueuedLayer::stop() throw(StopException)
{
  mutex_.acquire();
  bCancel_ = true;
  cond_.signal();
  mutex_.release();
  ACE_OS::thr_join(thread_,0,0);
  bCancel_ = false;
  thread_ = 0;
}

void EMANE::NEMQueuedLayer::processDownstreamControl(const ControlMessage & ctrl)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  
  queue_.push(EMANEUtils::makeFunctor(*this,&NEMQueuedLayer::handleProcessDownstreamControl,ctrl));
  cond_.signal();
}
    
void EMANE::NEMQueuedLayer::processDownstreamPacket(DownstreamPacket & pkt,const ControlMessage & ctrl)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  queue_.push(EMANEUtils::makeFunctor(*this,&NEMQueuedLayer::handleProcessDownstreamPacket,pkt,ctrl));
  
  cond_.signal();
}
    
void EMANE::NEMQueuedLayer::processUpstreamPacket(UpstreamPacket & pkt,const ControlMessage & ctrl)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  queue_.push(EMANEUtils::makeFunctor(*this,&NEMQueuedLayer::handleProcessUpstreamPacket,pkt,ctrl));
  cond_.signal();
}

void EMANE::NEMQueuedLayer::processUpstreamControl(const ControlMessage & ctrl)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  queue_.push(EMANEUtils::makeFunctor(*this,&NEMQueuedLayer::handleProcessUpstreamControl,ctrl));
  cond_.signal();
}

void EMANE::NEMQueuedLayer::processEvent(const EventId & eventId,
                                         const EventObjectState & state)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  queue_.push(EMANEUtils::makeFunctor(*this,&NEMQueuedLayer::handleProcessEvent,eventId,EventObjectState(state.get(),state.length(),state.getPlatformId(),state.getNEMId(),state.getComponentType())));
  cond_.signal();
}

void EMANE::NEMQueuedLayer::processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  queue_.push(EMANEUtils::makeFunctor(*this,&NEMQueuedLayer::handleProcessTimedEvent, taskType, eventId, tv, arg));
  cond_.signal();
}


ACE_THR_FUNC_RETURN EMANE::NEMQueuedLayer::processWorkQueue()
{
  while(1)
    {
      mutex_.acquire();
      
      while(queue_.empty() && !bCancel_)
        {
          cond_.wait();
        }

      if(bCancel_)
        {
          mutex_.release();
          break;
        }

      // retrieve the next funtion to execute
      EMANEUtils::Functor<void> * f = queue_.front();

      // remove the functor from the top of the queue
      queue_.pop();

      // release the queue syncronization object
      mutex_.release();

      
      try
        {
          // execute the funtor
          (*f)();
        }
      catch(std::exception & exp)
        {
          // cannot really do too much at this point, so we'll log it 
          // good candidate spot to generate and error event as well
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"%03hu NEMQueuedLayer::processWorkQueue: Exception caught %s",id_,exp.what());
        }
      catch(...)
        {
          // cannot really do too much at this point, so we'll log it 
          // good candidate spot to generate and error event as well
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"%03hu NEMQueuedLayer::processWorkQueue: Exception caught",id_);
        }

      // clean up
      delete f;
    }

  return 0;
}

void EMANE::NEMQueuedLayer::handleProcessDownstreamControl(const ControlMessage ctrl)
{
  doProcessDownstreamControl(ctrl);
}

void EMANE::NEMQueuedLayer::handleProcessDownstreamPacket(DownstreamPacket pkt,
                                                          const ControlMessage ctrl)
{
  doProcessDownstreamPacket(pkt,ctrl);
}

void EMANE::NEMQueuedLayer::handleProcessUpstreamPacket(UpstreamPacket pkt,
                                                        const ControlMessage ctrl)
{
  doProcessUpstreamPacket(pkt,ctrl);
}

void EMANE::NEMQueuedLayer::handleProcessUpstreamControl(const ControlMessage ctrl)
{
  doProcessUpstreamControl(ctrl);
}

void EMANE::NEMQueuedLayer::handleProcessEvent(const EventId id, const EventObjectState state)
{
  doProcessEvent(id,state);
}

void EMANE::NEMQueuedLayer::handleProcessTimedEvent(ACE_UINT32 taskType, long eventId, 
                                                    const ACE_Time_Value tv, const void *arg)
{
  doProcessTimedEvent(taskType, eventId, tv, arg);
}
