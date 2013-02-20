/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANENEMLAYERSTATERUNNING_HEADER_
#define EMANENEMLAYERSTATERUNNING_HEADER_

#include "nemlayerstate.h"

#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>

namespace EMANE
{
  /**
   * @class NEMLayerStateRunning
   *
   * @brief Component start complete all transport processing and
   * event actions are now allowed.  Allowable transitions are 
   * Component::stop().
   */
  class NEMLayerStateRunning : public NEMLayerState
  {
  public:
    NEMLayerStateRunning();

    ~NEMLayerStateRunning();

    /**
     * Handle post start
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * Layer post start hook
     */
    void handlePostStart(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer);
    
    /**
     * Handle stop
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @exception StopException
     *
     * @note A successful stop will result in a transition
     * to the NEMLayerStateStopped state.
     */
    void handleStop(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer)
      throw(StopException);

    /**
     *  Process downstream control
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param msg Control message
     *
     * Layer processing of downstream control
     */
    void processDownstreamControl(NEMStatefulLayer * pStatefulLayer, 
                                  NEMLayer * pLayer, 
                                  const ControlMessage & msg);
    
    /**
     *  Process downstream packet
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param pkt Reference to the Downstream packet
     * @param msg Reference to the ControlMessage
     *
     * Layer processing of downstream packet
     */
    void processDownstreamPacket(NEMStatefulLayer * pStatefulLayer, 
                                 NEMLayer * pLayer,  
                                 DownstreamPacket & pkt,
                                 const ControlMessage & msg);

    /**
     *  Process downstream packet
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param pkt Reference to the Upstream packet
     * @param msg Reference to the ControlMessage
     *
     * Layer processing of upstream packet
     */
    void processUpstreamPacket(NEMStatefulLayer * pStatefulLayer, 
                               NEMLayer * pLayer, 
                               UpstreamPacket & pkt,
                               const ControlMessage & msg);

    /**
     *  Process upstream control
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param msg Reference to the Control message
     *
     * Layer processing of upstream control
     */
    void processUpstreamControl(NEMStatefulLayer * pStatefulLayer, 
                                NEMLayer * pLayer, 
                                const ControlMessage & msg);
    
    /**
     *  Process event
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param id Event Id
     * @param state Opaque event state
     *
     * Layer processing of event
     */
    void processEvent(NEMStatefulLayer * pStatefulLayer, 
                      NEMLayer * pLayer, 
                      const EventId & id, 
                      const EventObjectState & state);

    /**
     *  Process timed event
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param eventType  event type id
     * @param eventId    event id
     * @param tv         scheduled run time
     * @param arg        event data
     *
     * Layer processing of timed event
     */
    void processTimedEvent(NEMStatefulLayer * pStatefulLayer, 
                           NEMLayer * pLayer, 
                           ACE_UINT32 eventType,
                           long eventId,
                           const ACE_Time_Value &tv, 
                           const void *arg);
 
  };
  
  typedef ACE_Unmanaged_Singleton<NEMLayerStateRunning,ACE_Null_Mutex> NEMLayerStateRunningSingleton;
}

#endif //EMANENEMLAYERSTATERUNNING_HEADER_
