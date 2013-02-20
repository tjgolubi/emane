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

#ifndef EMANEEVENTOBJECTSTATE_HEADER_
#define EMANEEVENTOBJECTSTATE_HEADER_

#include <vector>
#include <cstddef>

#include "emanetypes.h"
#include "emanecomponenttypes.h"

namespace EMANE
{
   /**
   * @class EventObjectStateException
   *
   * @brief Exception thrown when EventObjectState errors occur
   * on received message creation.
   *
   */
  class EventObjectStateException{};

  /**
   * @class EventObjectState
   *
   * @brief Opaque object state data.  Object state is used to
   * transfer event data in order to rebuild transmitted event objects.
   *
   * @note opaque object data should always be in network byte order
   */
  class EventObjectState
  {
  public:
    EventObjectState(const void * buf, size_t len);

    EventObjectState(const void * buf, size_t len, PlatformId platformId, NEMId nemId, ComponentType type);

    EventObjectState(const EMANE::EventObjectState& e);
    
    ~EventObjectState(){};

    /**
     * Get a pointer to the intenal state buffer
     *
     * @return pointer to the internal state buffer
     */
    const void * get() const;

    /**
     * Get the length of the internal state buffer
     * 
     * @return size Length in bytes
     */
    size_t length() const;

    /**
     * Get the PlatformId of the event
     * 
     * @return PlatformId
     */
    PlatformId getPlatformId() const;

    /**
     * Get the NEMId of the event
     * 
     * @return size Length in bytes
     */
    NEMId getNEMId() const;

    /**
     * Get the ComponentType of the event
     * 
     * @return size Length in bytes
     */
    ComponentType getComponentType() const;
    
  private:
    typedef std::vector<unsigned char> StateBuffer;
    StateBuffer stateBuffer_;
    PlatformId platformId_; 
    NEMId nemId_; 
    ComponentType componentType_;
  };
}

#include "emane/emaneeventobjectstate.inl"

#endif // EMANEEVENTOBJECTSTATE_HEADER_
