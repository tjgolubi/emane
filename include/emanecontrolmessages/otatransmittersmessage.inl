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

#include "emaneutils/netutils.h"

EMANE::OTATransmittersMessage::OTATransmittersMessage()
{ }


EMANE::OTATransmittersMessage::OTATransmittersMessage(const EMANE::ControlMessage & msg)
  throw(OTATransmittersMessageException)
{ 
  unpackControlMessage(msg);
}


EMANE::OTATransmittersMessage::OTATransmittersMessage(const EMANE::NEMIdSet & set)
{
   set_ = set;
}

inline  
void EMANE::OTATransmittersMessage::set (const EMANE::NEMIdSet & set)
 {
   set_ = set;
 }


inline  
void EMANE::OTATransmittersMessage::get (EMANE::NEMIdSet & set)
 {
   set = set_;
 }


inline  
EMANE::ControlMessage EMANE::OTATransmittersMessage::buildControlMessage() const
 {
   ACE_UINT16 buff[set_.size()];

   ACE_UINT16 * p = buff;

   for(EMANE::NEMIdSetConstIter iter = set_.begin(); iter != set_.end(); ++iter, ++p)
    {
      *p = ACE_HTONS(*iter);
    }

   return EMANE::ControlMessage(MAJOR_ID, MINOR_ID, buff, set_.size() * sizeof(ACE_UINT16));
 }


inline  
void EMANE::OTATransmittersMessage::unpackControlMessage(const EMANE::ControlMessage & msg)
        throw(OTATransmittersMessageException)
 {
   if ((msg.getMajorIdentifier() != MAJOR_ID) || (msg.getMinorIdentifier() != MINOR_ID))
    {
      throw(OTATransmittersMessageException("Invalid OTATransmittersMessage major/minor id"));
    }
   else
    {
      ACE_UINT16 * p = (ACE_UINT16 *) msg.get();

      const size_t len = msg.length() / sizeof(ACE_UINT16);
 
      for(ACE_UINT16 idx = 0; idx < len; ++idx, ++p)
       {
         set_.insert(ACE_NTOHS(*p));
       }
    }
 }
