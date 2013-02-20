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

#ifndef SAMPLEAGENTEVENT_HEADER_
#define SAMPLEAGENTEVENT_HEADER_

#include "emane/emaneevent.h"


 /**
   *
   * @class  sampleAgentEvent
   * 
   *  @brief Defines the Sample Agent Event.
   * An event used by the sample agent to transmit statistics on what events the sample agent heard
   * containing statistics on what it heard
   *        
   *        
   */

class SampleAgentEvent: public EMANE::Event
{
 public:
  static const EMANE::EventId EVENT_ID = 40000;

  
 /**
   *
   * @struct  SampleAgentEvent::SampleAgentEntry
   * 
   *  @brief Defines the Sample Agent Entry
   *        
   *        
   */
  struct SampleAgentEntry
  {
    ACE_UINT16 u16Node_;
    ACE_UINT32 locationEventsReceived_;
    ACE_UINT32 pathlossEventsReceived_;
  }__attribute__((packed));

  SampleAgentEvent(SampleAgentEntry *pEntries, ACE_UINT16 u16NumberOfEntries);

  SampleAgentEvent(const EMANE::EventObjectState)
    throw(EMANE::EventObjectStateException);

  ~SampleAgentEvent();

  EMANE::EventObjectState getObjectState() const;

  const SampleAgentEntry *getEntries() const;

  bool findEntry(EMANE::NEMId id, SampleAgentEntry &entry) const;

  ACE_UINT16 getNumberOfEntries() const;

 private:
  SampleAgentEntry * pEntries_;
  ACE_UINT16         u16NumberOfEntries_;
};

#include "sampleagentevent.inl"

#endif // SAMPLEAGENTEVENT_HEADER_
