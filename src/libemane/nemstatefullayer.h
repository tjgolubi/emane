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

#ifndef EMANENEMSTATEFULLAYER_HEADER_
#define EMANENEMSTATEFULLAYER_HEADER_

#include "emane/emanenemlayer.h"
#include "nemlayerstateuninitialized.h"

#include <memory>

namespace EMANE
{
  class NEMLayerState;
  
  /**
   * @class NEMStatefulLayer
   *
   * @brief A layer stack that enforces component state transition
   * rules.  The stateful layer is not a fully functional layer it 
   * wraps a real NEM layer allowing only correct transitions and 
   * state actions.
   */
  class NEMStatefulLayer : public NEMLayer
  {
  public:
    /**
     * Constructor
     *
     * @param id NEM id
     * @param pLayer Layer to statefully wrap 
     * @param pPlatformService pointer to the PlatformServiceProvider 
     *
     * @note Initial state is NEMLayerStateUninitialized
     */
    NEMStatefulLayer(NEMId id, 
                     NEMLayer * pLayer, 
                     PlatformServiceProvider *pPlatformService);
    
    ~NEMStatefulLayer();
   /**
    *
    * @brief initialize
    *
    */
  void initialize()
    throw(InitializeException);
  
   /**
    *
    * @brief configure
    *
    * @param items  reference to configuation items
    *
    */
  void configure(const ConfigurationItems & items)
    throw(ConfigureException);
  
   /**
    *
    * @brief start
    *
    */
  void start()
    throw(StartException);
  
   /**
    *
    * @brief postStart
    *
    */
  void postStart();

   /**
    *
    * @brief stop
    *
    */
  void stop()
    throw(StopException);
  
   /**
    *
    * @brief destroy
    *
    */
  void destroy()
    throw();

 /**
  *
  * @brief process an upstream control message.
  * 
  * @param msg  reference to the the control message.
  *
  */
  void processUpstreamControl(const ControlMessage &msg);

 /**
  *
  * @brief process an upstream packet.
  * 
  * @param pkt    reference to the upstream packet. 
  * @param msg    reference to the control message.
  *
  */

  void processUpstreamPacket(UpstreamPacket & pkt,
                             const ControlMessage &msg);  
  
 /**
  *
  * @brief process a downstream control message.
  * 
  * @param msg control message.
  *
  */
  void processDownstreamControl(const ControlMessage &msg);
 

  /**
   *
   * @brief process a downstream packet.
   * 
   * @param pkt  reference to the downstream packet.
   * @param msg  reference to the control message.
   *
   */
  void processDownstreamPacket(DownstreamPacket &pkt,
                               const ControlMessage &msg);


  /**
   *
   * @brief process an event
   * 
   * @param eventId  the event id
   * @param state    reference to the event object state
   *
   */
  void processEvent(const EventId &eventId,
                    const EventObjectState &state);

  /**
  *
  * @brief processTimedEvent
  *
  * @param taskType  task type
  * @param eventId   event id
  * @param tv        timeout
  * @param arg       data
  *
  */
  void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);

    
  /**
  *
  * @brief setUpstreamTransport
  *
  * @param pUpstreamTransport pointer to the  UpstreamTransport 
  *
  */
  void setUpstreamTransport(UpstreamTransport * pUpstreamTransport);
 
   
  /**
  *
  * @brief setDownstreamTransport
  *
  * @param pDownstreamTransport pointer to the  DownstreamTransport 
  *
  */
  void setDownstreamTransport(DownstreamTransport * pDownstreamTransport);


  /**
   * Change state
   * 
   * @param pState New state to transition to
   */
  void changeState(NEMLayerState * pState);
   
  private:
    std::auto_ptr<NEMLayer> pLayer_;
    NEMLayerState * pState_;
  };
}

#endif // EMANENEMSTATEFULLAYER_HEADER_
