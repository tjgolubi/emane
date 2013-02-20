/*
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef MOBILITYMODELEVENTGENERATOR_HEADER_
#define MOBILITYMODELEVENTGENERATOR_HEADER_

#include "emane/emaneeventgenerator.h"
#include "mobilitymodelloader.h"

#include <ace/Thread.h>
#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>

/**
 * @class MobilityModelEventGenerator
 *
 * @brief MobilityModelEvent generator is the the entry point
 * for MobilityModelEvents into the EventServer.
 */
class MobilityModelEventGenerator : public EMANE::EventGenerator
{
public:
  MobilityModelEventGenerator(EMANE::PlatformServiceProvider *pPlatformService);
  ~MobilityModelEventGenerator();
  
  void initialize()
    throw(EMANE::InitializeException);
  
  void configure(const EMANE::ConfigurationItems & items)
    throw(EMANE::ConfigureException);
  
  void start()
    throw(EMANE::StartException);
  
  void stop()
    throw(EMANE::StopException);
  
  void destroy()
    throw();

  void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);

  void processEvent(const EMANE::EventId&, const EMANE::EventObjectState&);

private:
  MobilityModelLoader loader_;
  ACE_UINT16 u16TotalNodes_;
  ACE_UINT16 u16MaxNEMPresent_;
  bool bPublishLocationEvents_;
  bool bPublishPathlossEvents_;
  ACE_thread_t thread_;
  ACE_Thread_Mutex mutex_;
  ACE_Condition<ACE_Thread_Mutex> cond_;
  bool bCancel_;
  ACE_THR_FUNC_RETURN svc();
};

#endif // MOBILITYMODELEVENTGENERATOR_HEADER_
