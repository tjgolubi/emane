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

/**
 *
 * @brief Defines the path loss event state
 *
 */
struct PathlossEventState
{ 
  ACE_UINT16 u16NumberEntries_;
  PathlossEvent::PathlossEntry entries_[0];
} __attribute__((packed));
 
inline
PathlossEvent::PathlossEvent(PathlossEntry * pEntries, ACE_UINT16 u16NumberOfEntries):
  EMANE::Event(EVENT_ID,"Pathloss Event"),
  pEntries_(new PathlossEntry[u16NumberOfEntries]),
  u16NumberOfEntries_(u16NumberOfEntries)
{
  memcpy(pEntries_,pEntries,u16NumberOfEntries * sizeof(PathlossEntry));
}

inline
PathlossEvent::PathlossEvent(const EMANE::EventObjectState state)
  throw(EMANE::EventObjectStateException):
  EMANE::Event(EVENT_ID,"Pathloss Event"),
  pEntries_(0) 
{
  const PathlossEventState * p = reinterpret_cast<const PathlossEventState *>(state.get());

  u16NumberOfEntries_ =  ACE_NTOHS(p->u16NumberEntries_);
  
  pEntries_        = new PathlossEntry[u16NumberOfEntries_];

  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
    {
      pEntries_[i].u16TxNode_              = ACE_NTOHS(p->entries_[i].u16TxNode_);
      pEntries_[i].i32PathLossDBScaled_    = ACE_NTOHL(p->entries_[i].i32PathLossDBScaled_);
      pEntries_[i].i32RevPathLossDBScaled_ = ACE_NTOHL(p->entries_[i].i32RevPathLossDBScaled_);
    }
}

inline
PathlossEvent::~PathlossEvent()
{
  delete [] pEntries_;
}

inline
EMANE::EventObjectState PathlossEvent::getObjectState() const
{
  unsigned char buf[sizeof(PathlossEventState) + u16NumberOfEntries_ * sizeof(PathlossEntry)];

  PathlossEventState * p = reinterpret_cast<PathlossEventState *>(buf);
  
  p->u16NumberEntries_ = ACE_HTONS(u16NumberOfEntries_);

  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
    {
      p->entries_[i].u16TxNode_              = ACE_HTONS(pEntries_[i].u16TxNode_);
      p->entries_[i].i32PathLossDBScaled_    = ACE_HTONL(pEntries_[i].i32PathLossDBScaled_);
      p->entries_[i].i32RevPathLossDBScaled_ = ACE_HTONL(pEntries_[i].i32RevPathLossDBScaled_);
    }

  return EMANE::EventObjectState(buf,sizeof(buf));
}

inline
const PathlossEvent::PathlossEntry * PathlossEvent::getEntries() const
{
  return pEntries_;
}

inline
ACE_UINT16 PathlossEvent::getNumberOfEntries() const
{
  return u16NumberOfEntries_;
}



