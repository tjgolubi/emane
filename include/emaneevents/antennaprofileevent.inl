/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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
 * @brief internal Antenna Profile update definition
 *
 */ 


#include "emane/emaneconstants.h"

struct AntennaProfileEntryInternal
{ 
    ACE_UINT16 u16Node_;
    ACE_UINT16 u16ProfileId_;
    ACE_INT32  i32AzimuthScaled_;
    ACE_INT32  i32ElevationScaled_;
} __attribute__((packed));


/**
 *
 * @brief internal Antenna Profile Event State
 *
 */ 
struct AntennaProfileEventState
{
  ACE_UINT16 u16NumberEntries_;

  AntennaProfileEntryInternal entries_[0];
} __attribute__((packed));
 

inline
AntennaProfileEvent::AntennaProfileEvent(AntennaProfileEntry * pEntries, ACE_UINT16 u16NumberOfEntries):
  EMANE::Event(EVENT_ID,"AntennaProfile Event"),
  pEntries_(new AntennaProfileEntry[u16NumberOfEntries]),
  u16NumberOfEntries_(u16NumberOfEntries)
{
  // copy the entries in public format in host byte order
  memcpy(pEntries_, pEntries, u16NumberOfEntries * sizeof(AntennaProfileEntry));
}


inline
AntennaProfileEvent::AntennaProfileEvent(const EMANE::EventObjectState state)
  throw(EMANE::EventObjectStateException):
  EMANE::Event(EVENT_ID,"AntenneProfile Event"),
  pEntries_(0) 
{
  const AntennaProfileEventState * pState = reinterpret_cast<const AntennaProfileEventState *>(state.get());
 
  // save number of entries 
  u16NumberOfEntries_ = ACE_NTOHS(pState->u16NumberEntries_);
 
  // public entries
  pEntries_ = new AntennaProfileEntry[u16NumberOfEntries_];

  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
    {
      // short
      pEntries_[i].u16ProfileId_       = ACE_NTOHS(pState->entries_[i].u16ProfileId_);
      pEntries_[i].u16Node_            = ACE_NTOHS(pState->entries_[i].u16Node_);

      // long
      pEntries_[i].fElevationDegrees_  = static_cast<float>(ACE_NTOHL(pState->entries_[i].i32ElevationScaled_)) / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
      pEntries_[i].fAzimuthDegrees_    = static_cast<float>(ACE_NTOHL(pState->entries_[i].i32AzimuthScaled_))   / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
    }
}



inline
AntennaProfileEvent::~AntennaProfileEvent()
{
  delete [] pEntries_;
}


inline
bool AntennaProfileEvent::findEntry(EMANE::NEMId id, AntennaProfileEntry & entry) const
{
  int idx = -1;

  // check all entries
  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
    {
      // exact match
      if(pEntries_[i].u16Node_ == id)
       {
         // copy
         entry = pEntries_[i];
     
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

   // was all nodes id found
   if(idx != -1)
    {
       // copy
       entry = pEntries_[idx];
      
       // success 
       return true; 
    }

   // no match
   return false;
}




inline
EMANE::EventObjectState AntennaProfileEvent::getObjectState() const
{
  // allocate buffer of internal entries
  unsigned char buf[sizeof(AntennaProfileEventState) + u16NumberOfEntries_ * sizeof(AntennaProfileEntryInternal)];

  AntennaProfileEventState * pState = reinterpret_cast<AntennaProfileEventState *>(buf);

  pState->u16NumberEntries_ = ACE_HTONS(u16NumberOfEntries_);

  for(ACE_UINT16 i = 0; i < u16NumberOfEntries_; ++i)
    {
      // short
      pState->entries_[i].u16Node_            = ACE_HTONS(pEntries_[i].u16Node_);
      pState->entries_[i].u16ProfileId_       = ACE_HTONS(pEntries_[i].u16ProfileId_);

      // long
      pState->entries_[i].i32ElevationScaled_ = ACE_HTONL(static_cast<ACE_INT32> (pEntries_[i].fElevationDegrees_ * EMANE::MILLI_ARC_SECONDS_PER_DEGREE));
      pState->entries_[i].i32AzimuthScaled_   = ACE_HTONL(static_cast<ACE_INT32> (pEntries_[i].fAzimuthDegrees_   * EMANE::MILLI_ARC_SECONDS_PER_DEGREE));
    }

  // return state in internal format
  return EMANE::EventObjectState(buf, sizeof(buf));
}



inline
const AntennaProfileEvent::AntennaProfileEntry * AntennaProfileEvent::getEntries() const
{
  // return entries in public format
  return pEntries_;
}



inline
ACE_UINT16 AntennaProfileEvent::getNumberOfEntries() const
{
  return u16NumberOfEntries_;
}
