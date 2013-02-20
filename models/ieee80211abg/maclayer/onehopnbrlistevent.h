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

#ifndef IEEE80211ABG_ONEHOPNBRLISTEVENT_HEADER_
#define IEEE80211ABG_ONEHOPNBRLISTEVENT_HEADER_

#include "emane/emaneevent.h"
#include "emaneevents/events.h"

#include <ace/Basic_Types.h>

/**
 *
 * @class OneHopNbrListEvent 
 *
 * @brief Defines the one hop nbr list event
 *
 */
class OneHopNbrListEvent : public EMANE::Event
{
public:
  static const EMANE::EventId EVENT_ID = EMANE::REGISTERED_EMANE_EVENT_IEEE80211ABG_ONE_HOP_NBR_LIST;

  /**
   *
   * @brief one hop nbr list update definition
   *
   */ 
  struct OneHopNbrEntry
    { 
      ACE_UINT16 u16NEMId_;
    } __attribute__((packed));

  OneHopNbrListEvent(OneHopNbrEntry * pEntries, ACE_UINT16 u16NumberOfEntries);
  
  OneHopNbrListEvent(const EMANE::EventObjectState)
    throw(EMANE::EventObjectStateException);
  
  ~OneHopNbrListEvent();
  
  EMANE::EventObjectState getObjectState() const;
  
  const OneHopNbrEntry * getEntries() const;

  ACE_UINT16 getNumberOfEntries() const;
  
private:
  OneHopNbrEntry   * pEntries_;
  ACE_UINT16       u16NumberOfEntries_;
};

#include "onehopnbrlistevent.inl"

#endif // IEEE80211ABG_ONEHOPNBRLISTEVENT_HEADER_
