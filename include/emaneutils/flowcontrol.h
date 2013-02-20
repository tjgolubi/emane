/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSFLOWCONTROL_HEADER_
#define EMANEUTILSFLOWCONTROL_HEADER_

#include "emane/emaneupstreamtransport.h"
#include "emane/emanetypes.h"
#include "emane/emanenet.h"

#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>

namespace EMANEUtils
{
 /**
  *  
  * @class FlowControl
  *
  * @brief Flow Control (token consumer) side.
  *
  */
  class FlowControl
  { 
  
  public:
  /**
   *  
   * @brief status of a flow control request
   *
   */
   struct FlowControlStatus 
    {
      bool bStatus_;                        // request status success or failure

      EMANE::UINT16 ui16TokensAvailable_;   // number of tokens available after request
    
 
     /**
      *  
      * Initialized FlowControlStatus 
      *
      * @param bStatus request status success of failure
      * @param ui16TokensAvailable number of tokens available after request
      *
      */
      FlowControlStatus (bool bStatus, EMANE::UINT16 ui16TokensAvailable) :
       bStatus_(bStatus),
       ui16TokensAvailable_(ui16TokensAvailable)
       { }
    };

   /**
    *  
    * constructor
    *
    * @param transport reference to the UpstremTransport
    *
    */
    FlowControl(EMANE::UpstreamTransport & transport);
 
   /**
    *  
    * destructor
    *
    */
    ~FlowControl();

   /**
    *  
    * start starts flow control processing
    *
    */
    void start();

   /**
    *  
    * stop stops flow control processing
    *
    */
    void stop();

   /**
    *  
    * remove a flow control token, blocks until a token is available when flow control is enabled
    *
    * @return returns FlowControlStatus
    *
    */
    FlowControlStatus removeToken();

   /**
    *  
    * gets the number of flow control tokens available
    *
    * @return returns number of flow control tokens
    *
    */
    EMANE::UINT16 getAvailableTokens();

   /**
    *  
    * handles a flow control update message
    *
    * @param ctrl a flow control message containig the number of tokens avaialble
    *
    */
    void processFlowControlMessage(const EMANE::ControlMessage & ctrl);

   /**
    *  
    * sends a flow control message ack in response to a flow control update message
    *
    */
    void sendFlowControlMessageAck();
    
  private:
    EMANE::UpstreamTransport & rTransport_;

    bool bCancel_;

    ACE_Thread_Mutex mutex_;

    ACE_Condition<ACE_Thread_Mutex> cond_;

    EMANE::UINT16 ui16TokensAvailable_;
  };
}

#include "flowcontrol.inl"

#endif // EMANEUTILSFLOWCONTROL_HEADER_
