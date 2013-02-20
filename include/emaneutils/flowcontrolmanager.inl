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

#include "emanecontrolmessages/controlmessages.h"
#include "emanecontrolmessages/flowcontroltokenrequest.h"
#include "emanecontrolmessages/flowcontroltokenresponse.h"
#include "emanecontrolmessages/flowcontroltokenresponseack.h"

#include <ace/Guard_T.h>

inline
EMANEUtils::FlowControlManager::FlowControlManager(EMANE::DownstreamTransport & transport):
  rTransport_(transport),
  ui16TokensAvailable_(0),
  ui16TotalTokensAvailable_(0),
  ui16ShadowTokenCount_(0),
  bAckPending_(true)
{}


inline
EMANEUtils::FlowControlManager::~FlowControlManager(){}


inline
void EMANEUtils::FlowControlManager::start(EMANE::UINT32 ui16TotalTokensAvailable)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  
  ui16TotalTokensAvailable_ = ui16TotalTokensAvailable;
  ui16TokensAvailable_      = ui16TotalTokensAvailable_;

  sendFlowControlResponseMessage();
}


inline
void EMANEUtils::FlowControlManager::stop()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  ui16TotalTokensAvailable_ = 0;
  ui16TokensAvailable_      = 0;
  ui16ShadowTokenCount_     = 0;
}


inline
EMANEUtils::FlowControlManager::FlowControlStatus 
EMANEUtils::FlowControlManager::addToken(EMANE::UINT16 ui16Tokens)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  // get add tokens status
  const bool bStatus = ((ui16TokensAvailable_ + ui16Tokens) <= ui16TotalTokensAvailable_);

  // have room to add all requested tokens
  if(bStatus == true)
   { 
     // update tokens available
     ui16TokensAvailable_ += ui16Tokens;
   }

  // check shadow count, send update if tokens are available
  if((ui16ShadowTokenCount_ == 0) && (ui16TokensAvailable_ > 0))
   {
     sendFlowControlResponseMessage();
   }

  return EMANEUtils::FlowControlManager::FlowControlStatus (bStatus, ui16TokensAvailable_, ui16ShadowTokenCount_);
}


inline
EMANEUtils::FlowControlManager::FlowControlStatus 
EMANEUtils::FlowControlManager::removeToken()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  bool bStatus;
 
  // ack pending, no action
  if(bAckPending_ == true)
   {
     bStatus = false;
   }
  else 
   {
     --ui16ShadowTokenCount_;

     // no tokens available
     if(ui16TokensAvailable_ == 0)
       {
         bStatus = false;
       }
     else
       {
         // decrement tokens available
         --ui16TokensAvailable_;

         bStatus = true;
       }
    }

  return EMANEUtils::FlowControlManager::FlowControlStatus (bStatus, ui16TokensAvailable_, ui16ShadowTokenCount_);
}


inline
EMANE::UINT16 EMANEUtils::FlowControlManager::getAvailableTokens()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  return ui16TokensAvailable_;
}


inline
EMANE::UINT16 EMANEUtils::FlowControlManager::getShadowCount()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  return ui16ShadowTokenCount_;
}



inline
void EMANEUtils::FlowControlManager::processFlowControlMessage(const EMANE::ControlMessage & ctrl)
{
  if(ctrl.getMajorIdentifier() == EMANE::FlowControlTokenRequest::MAJOR_ID &&
     ctrl.getMinorIdentifier() == EMANE::FlowControlTokenRequest::MINOR_ID)
    {
      ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  
      sendFlowControlResponseMessage();

    }
  else if(ctrl.getMajorIdentifier() == EMANE::FlowControlTokenResponseAck::MAJOR_ID &&
          ctrl.getMinorIdentifier() == EMANE::FlowControlTokenResponseAck::MINOR_ID)
    {
      ACE_Guard<ACE_Thread_Mutex> m(mutex_);

      bAckPending_ = false;
    }
}


inline
void EMANEUtils::FlowControlManager::sendFlowControlResponseMessage()
{
  EMANE::FlowControlTokenResponse flowControlResponse(ui16TokensAvailable_);
  
  flowControlResponse.toNetworkByteOrder();
  
  rTransport_.sendUpstreamControl(EMANE::ControlMessage(EMANE::FlowControlTokenResponse::MAJOR_ID,
                                                        EMANE::FlowControlTokenResponse::MINOR_ID,
                                                        &flowControlResponse, 
                                                        sizeof(flowControlResponse)));
  
  ui16ShadowTokenCount_ = ui16TokensAvailable_;

  bAckPending_ = true;
}
