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

#ifndef EMANENEMLAYERSTATE_HEADER_
#define EMANENEMLAYERSTATE_HEADER_

#include "emane/emanenemlayer.h"

namespace EMANE
{
  class NEMStatefulLayer;
  class NEMLayer;
  
  /**
   * @class NEMLayerState
   *
   * @brief Encapsulated behavior associated with an NEMLayer depending
   * on the layer's current context
   *
   * @note Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
   * Design Patterns: Elements of Reusable Object-Oriented Software.
   * Addison-Wesley, Reading MA, 1995
   * Bridge, p 305
   */
  class NEMLayerState
  {
  public:
    virtual ~NEMLayerState() = 0;

    /**
     * Handle initialize
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @exception InitializeException
     *
     * @note Default implementation generates a log error
     */ 
    virtual void handleInitialize(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer)
      throw(InitializeException);

    /**
     * Handle configuration
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param items Reference to theConfiguration item list
     * 
     * @exception ConfigureException
     *
     * @note Default implementation generates a log error
     *
     */
    virtual void handleConfigure(NEMStatefulLayer * pStatefulLayer, 
                                 NEMLayer * pLayer, 
                                 const ConfigurationItems & items)
      throw(ConfigureException);

    /**
     * Handle start
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @exception StartException
     *
     * @note Default implementation generates a log error
     */
    virtual void handleStart(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer)
      throw(StartException);
    
    /**
     * Handle post start
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @note Default implementation generates a log error
     */
    virtual void handlePostStart(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer);

     /**
     * Handle stop
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @exception StopException
     *
     * @note Default implementation generates a log error
     */
    virtual void handleStop(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer)
      throw(StopException);

    /**
     * Handle destroy
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @note Default implementation generates a log error
     */
    virtual void handleDestroy(NEMStatefulLayer * pStatefulLayer, NEMLayer * pLayer)
      throw();

    /**
     *  Process downstream control
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param msg Reference to the Control message
     *
     * @note Default implementation generates a log error
     */
    virtual void processDownstreamControl(NEMStatefulLayer * pStatefulLayer, 
                                          NEMLayer * pLayer, 
                                          const ControlMessage & msg);

    /**
     *  Process downstream packet
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param pkt Reference to the Downstream packet
     * @param msg reference to the ControlMessage
     *
     * @note Default implementation generates a log error
     */ 
    virtual void processDownstreamPacket(NEMStatefulLayer * pStatefulLayer, 
                                         NEMLayer * pLayer,  
                                         DownstreamPacket & pkt,
                                         const ControlMessage & msg);
    
    /**
     *  Process downstream packet
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param pkt Reference to the Upstream packet
     * @param msg reference to the ControlMessage
     *
     * @note Default implementation generates a log error
     */
    virtual void processUpstreamPacket(NEMStatefulLayer * pStatefulLayer, 
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
     * @note Default implementation generates a log error
     */
    virtual void processUpstreamControl(NEMStatefulLayer * pStatefulLayer, 
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
     * @note Default implementation generates a log error
     */
    virtual void processEvent(NEMStatefulLayer * pStatefulLayer,
                              NEMLayer * pLayer,
                              const EventId & id, 
                              const EventObjectState &state);


    /**
     *  Process timed event
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param eventType  event type id
     * @param eventId    event id
     * @param tv         Reference to the scheduled run time
     * @param arg        event opaque data
     *
     * @note Default implementation generates a log error
     */
    virtual void processTimedEvent(NEMStatefulLayer * pStatefulLayer,
                                   NEMLayer * pLayer,
                                   ACE_UINT32 eventType,
                                   long eventId,
                                   const ACE_Time_Value &tv, 
                                   const void *arg);
   
    /**
     * Get state name
     *
     * @return state Name of current state
     */
    std::string getStateName() const;

  protected:
    const char * pzStateName_;

    NEMLayerState(const char * pzStateName);
    
    void changeState(NEMStatefulLayer * pStatefulLayer,NEMLayerState * pState);
  };
}

#endif //EMANENEMLAYERSTATE_HEADER_
