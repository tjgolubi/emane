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

#ifndef EMANEUTILSFLOWCONTROLMANAGER_HEADER_
#define EMANEUTILSFLOWCONTROLMANAGER_HEADER_

#include "emane/emanedownstreamtransport.h"
#include "emane/emanetypes.h"
#include "emane/emanenet.h"

#include <ace/Thread_Mutex.h>

namespace EMANEUtils
{
 /**
  *
  * @class FlowControlManager
  *
  * @brief Flow Control Manager (token producer) side.
  *
  */
  class FlowControlManager
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

      EMANE::UINT16 ui16TokensAvailable_;   // number of tokens available 
      EMANE::UINT16 ui16ShadowTokenCount_;  // number of peer flow control tokens (shadow count)
     
     /**
      *  
      * @param bStatus request status success of failure
      * @param ui16TokensAvailable number of tokens available after request
      * @param ui16ShadowTokenCount number of peer tokens available (shadow count)
      *
      */
      FlowControlStatus (bool bStatus, EMANE::UINT16 ui16TokensAvailable, EMANE::UINT16 ui16ShadowTokenCount) :
       bStatus_(bStatus),
       ui16TokensAvailable_(ui16TokensAvailable),
       ui16ShadowTokenCount_(ui16ShadowTokenCount)
       { }
    };

   /**
    *  
    * constructor
    *
    * @param transport reference to the DownstremTransport
    *
    */
    FlowControlManager(EMANE::DownstreamTransport & transport);
    
   /**
    *  
    * destructor
    *
    */
    ~FlowControlManager();

   /**
    *  
    * starts flow control processing
    *
    * @param  ui16TotalTokensAvailable initial number of flow control tokens available
    *
    */
    void start(EMANE::UINT32 ui16TotalTokensAvailable);

   /**
    *  
    * stops flow control processing
    *
    */
    void stop();

   /**
    *  
    * remove a flow control token
    *
    * @return returns FlowControlStatus
    *
    */
    FlowControlStatus removeToken();

   /**
    *  
    * add one or more flow control token(s)
    *
    * @param ui16Tokens number of tokens with default value of 1
    *
    * @return returns FlowControlStatus
    *
    */
    FlowControlStatus addToken(EMANE::UINT16 ui16Tokens = 1);

   /**
    *  
    * get number of flow control token
    *
    * @return number of flow control tokens
    *
    */
    EMANE::UINT16 getAvailableTokens();

   /**
    *  
    * get number of shadow count flow control token
    *
    * @return number of shadow count flow control tokens
    *
    */
    EMANE::UINT16 getShadowCount();

   /**
    *  
    * handles a flow control update message
    *
    * @param ctrl flow control message
    *
    */
    void processFlowControlMessage(const EMANE::ControlMessage & ctrl);
    
  private:
    EMANE::DownstreamTransport & rTransport_;

    bool bCancel_;

    ACE_Thread_Mutex mutex_;

    EMANE::UINT16 ui16TokensAvailable_;

    EMANE::UINT16 ui16TotalTokensAvailable_;

    EMANE::UINT16 ui16ShadowTokenCount_;

    bool bAckPending_;

    void sendFlowControlResponseMessage();
  };
}

#include "flowcontrolmanager.inl"

#endif // EMANEUTILSFLOWCONTROLMANAGER_HEADER_
