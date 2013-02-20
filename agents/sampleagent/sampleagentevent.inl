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


/**
 * @struct SampleAgentEntryInternal
 *
 * @brief internal Sample Entry update definition
 *
 */
struct SampleAgentEntryInternal
{
   ACE_UINT16 u16Node_;
   ACE_UINT32 locationEventsReceived_;
   ACE_UINT32 pathlossEventsReceived_;
}__attribute__((packed));

/**
 *
 * @struct SampleAgentEventState
 *
 * @brief Defines the sample agent event state
 *
 */
struct SampleAgentEventState
{
  ACE_UINT16 u16NumberEntries_;
  SampleAgentEntryInternal entries_[0];
} __attribute__((packed));


/**
 *
 * @class SampleAgentEvent
 *
 * @brief Defines the sample agent event 
 *
 */
inline
SampleAgentEvent::SampleAgentEvent(SampleAgentEntry * pEntries, ACE_UINT16 u16NumberOfEntries):
  EMANE::Event(EVENT_ID,"SampleAgent Event"),
  pEntries_(new SampleAgentEntry[u16NumberOfEntries]),
  u16NumberOfEntries_(u16NumberOfEntries)
{
  // copy the entries in public format in host byte order
  memcpy(pEntries_, pEntries, u16NumberOfEntries * sizeof(SampleAgentEntry));
}

inline
SampleAgentEvent::SampleAgentEvent(const EMANE::EventObjectState state)
  throw(EMANE::EventObjectStateException):
  EMANE::Event(EVENT_ID,"SampleAgent Event"),
  pEntries_(0)
{
  const SampleAgentEventState * pState = reinterpret_cast<const SampleAgentEventState *>(state.get());

  // save number of entries
  u16NumberOfEntries_ = ACE_NTOHS(pState->u16NumberEntries_);

  // public entries
  pEntries_ = new SampleAgentEntry[u16NumberOfEntries_];

  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
  {
    // convert to host byte order
    pEntries_[i].u16Node_                 = ACE_NTOHS(pState->entries_[i].u16Node_);
    pEntries_[i].locationEventsReceived_  = ACE_NTOHL(pState->entries_[i].locationEventsReceived_);
    pEntries_[i].pathlossEventsReceived_  = ACE_NTOHL(pState->entries_[i].pathlossEventsReceived_);
  }
}

inline
SampleAgentEvent::~SampleAgentEvent()
{
  delete [] pEntries_;
}

inline
bool SampleAgentEvent::findEntry(EMANE::NEMId id, SampleAgentEntry & entry) const
{
  int idx = -1;

  // check all entries
  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
  {
      // exact match
      if(pEntries_[i].u16Node_ == id)
      {
        entry.u16Node_       = pEntries_[i].u16Node_;
        entry.locationEventsReceived_  = pEntries_[i].locationEventsReceived_;
        entry.pathlossEventsReceived_  = pEntries_[i].pathlossEventsReceived_;

        // success
        return true;
      }

      // check for all nodes
      if(pEntries_[i].u16Node_ == 0xFFFF)
      {
        // remember index
        idx = i;
      }

    }
    
    // was all nodes found
    if(idx != -1)
    {
       // set values
        entry.u16Node_       = pEntries_[idx].u16Node_;
        entry.locationEventsReceived_  = pEntries_[idx].locationEventsReceived_;
        entry.pathlossEventsReceived_  = pEntries_[idx].pathlossEventsReceived_;

        // success
        return true;
     }

     // no match
     return false;
}

inline
EMANE::EventObjectState SampleAgentEvent::getObjectState() const
{
  // allocate buffer of internal entries
  unsigned char buf[sizeof(SampleAgentEventState) + u16NumberOfEntries_ * sizeof(SampleAgentEntryInternal)];

  SampleAgentEventState * pState = reinterpret_cast<SampleAgentEventState *>(buf);

  pState->u16NumberEntries_ = ACE_HTONS(u16NumberOfEntries_);

  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
  {
    pState->entries_[i].u16Node_       = ACE_HTONS(pEntries_[i].u16Node_);
    pState->entries_[i].locationEventsReceived_ = ACE_HTONL(pEntries_[i].locationEventsReceived_);
    pState->entries_[i].pathlossEventsReceived_ = ACE_HTONL(pEntries_[i].pathlossEventsReceived_);
  }

  // return state in internal format
  return EMANE::EventObjectState(buf, sizeof(buf));
}

inline
const SampleAgentEvent::SampleAgentEntry * SampleAgentEvent::getEntries() const
{
  // return entries in public format
  return pEntries_;
}

inline
ACE_UINT16 SampleAgentEvent::getNumberOfEntries() const
{
  return u16NumberOfEntries_;
}
