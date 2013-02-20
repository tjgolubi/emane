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


inline
EMANE::EventObjectState::EventObjectState(const void * buf, size_t len) :
 platformId_(0),
 nemId_(0),
 componentType_(static_cast<ComponentType>(0))
{
  const unsigned char * c = static_cast<const unsigned char *>(buf);
  stateBuffer_.reserve(len);
  stateBuffer_.insert(stateBuffer_.begin(),&c[0],&c[len]);
}

inline
EMANE::EventObjectState::EventObjectState(const EMANE::EventObjectState& e)
{
  stateBuffer_ = e.stateBuffer_;
  platformId_ = e.platformId_;
  nemId_ = e.nemId_;
  componentType_ = e.componentType_;  
}

inline
EMANE::EventObjectState::EventObjectState(const void * buf, size_t len, PlatformId platformId, NEMId nemId, ComponentType componentType) :
 platformId_(0),
 nemId_(0),
 componentType_(static_cast<ComponentType>(0))
{
  const unsigned char * c = static_cast<const unsigned char *>(buf);
  stateBuffer_.reserve(len);
  stateBuffer_.insert(stateBuffer_.begin(),&c[0],&c[len]);

  platformId_ = platformId;
  nemId_ = nemId;
  componentType_ = componentType;
}

inline
const void * EMANE::EventObjectState::get() const
{
  return (stateBuffer_.empty() ? 0 : &stateBuffer_[0]);
}

inline
size_t EMANE::EventObjectState::length() const
{
  return stateBuffer_.size();
}

inline
EMANE::PlatformId EMANE::EventObjectState::getPlatformId() const
{
  return platformId_;
}

inline
EMANE::NEMId EMANE::EventObjectState::getNEMId() const
{
  return nemId_;
}

inline
EMANE::ComponentType EMANE::EventObjectState::getComponentType() const
{
  return componentType_;
}
