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

#ifndef EMANEEVENTSERVICEPROVIDER_HEADER_
#define EMANEEVENTSERVICEPROVIDER_HEADER_

#include "emane/emaneevent.h"
#include "emane/emanecomponenttypes.h"

namespace EMANE
{
  
  /**
   * @class EventServiceProvider
   *
   * @brief Event service provider interface
   */
  class EventServiceProvider
  {
  public:
    virtual ~EventServiceProvider(){}
    
    /**
     * Send an event
     *
     * @param platformId Id of destination platform @c 0 for all platforms
     * @param nemId Id of destination NEM @c 0 for all NEMs in a platform
     * @param type Component target of destination
     * @param event Reference to the event object
     */
    virtual void sendEvent(PlatformId    platformId,
                           NEMId         nemId, 
                           ComponentType type, 
                           const Event & event) = 0;

    /**
     * Send an event
     *
     * @param platformId Id of destination platform @c 0 for all platforms
     * @param nemId Id of destination NEM @c 0 for all NEMs in a platform
     * @param type Component target of destination
     * @param eventId The event id
     * @param state Reference to the event object state
     */
    virtual void sendEvent(PlatformId    platformId,
                           NEMId         nemId, 
                           ComponentType type,
                           EventId       eventId,
                           const EventObjectState & state) = 0;


  protected:
    EventServiceProvider(){}
  };
}

#endif //EMANEEVENTSERVICEPROVIDER_HEADER_
