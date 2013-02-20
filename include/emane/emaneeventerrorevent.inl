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

#include <memory>
#include <cstring>

inline
EMANE::EventErrorEvent::EventErrorEvent(EventId id, const EventObjectState state,const std::string & sDescription):
  EMANE::Event(EventErrorEvent::EVENT_ID,"Event Error"),
  id_(id),
  state_(state),
  sDescription_(sDescription){}

inline
EMANE::EventErrorEvent::EventErrorEvent(const EMANE::EventObjectState state)
  throw(EMANE::EventObjectStateException):
  EMANE::Event(EventErrorEvent::EVENT_ID,"Event Error"),
  state_(0,0)
{
  if(state_.length() >= 6)
    {
      const unsigned char * c = reinterpret_cast<const unsigned char *>(state.get());
      
      id_ = (static_cast<unsigned short>(c[0])<<8) + c[1];
      
      size_t stateLength = (static_cast<unsigned short>(c[2])<<8) + c[3];
      
      size_t descriptLength = (static_cast<unsigned short>(c[4])<<8) + c[5];
      
      if(state_.length() == 6 + stateLength +  descriptLength)
        {
          state_ = EventObjectState(&c[6],stateLength);
      
          sDescription_  = std::string(reinterpret_cast<const char *>(&c[6] + stateLength),descriptLength);
        }
      else
        {
          throw EMANE::EventObjectStateException();
        }
    }
  else
    {
      throw EMANE::EventObjectStateException();
    }
}
 
inline     
EMANE::EventErrorEvent::~EventErrorEvent(){}

inline      
EMANE::EventObjectState EMANE::EventErrorEvent::getObjectState() const
{
  unsigned char buf[sDescription_.length() + state_.length() + sizeof(id_) + 2 + 2];

  buf[0] = (id_ & 0xff00) >> 8;
  buf[1] = id_ & 0xff;

  buf[2] = (state_.length() & 0xff00) >> 8;
  buf[3] = state_.length() & 0xff;

  buf[4] = (sDescription_.length() & 0xff00) >> 8;
  buf[5] = sDescription_.length() & 0xff;

  memcpy(&buf[6],state_.get(),state_.length());

  memcpy(&buf[6] + state_.length(),sDescription_.c_str(), sDescription_.length());

  return EMANE::EventObjectState(buf,sizeof(buf));
}

