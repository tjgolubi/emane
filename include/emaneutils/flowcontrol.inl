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

inline
EMANEUtils::FlowControl::FlowControl(EMANE::UpstreamTransport & transport):
  rTransport_(transport),
  bCancel_(false),
  cond_(mutex_),
  ui16TokensAvailable_(0)
{}


inline
EMANEUtils::FlowControl::~FlowControl()
{}


inline
void EMANEUtils::FlowControl::start()
{
  // create request message
  EMANE::FlowControlTokenRequest flowControlRequest;
 
  // conver to network byte order 
  flowControlRequest.toNetworkByteOrder();
 
  // send message 
  rTransport_.sendDownstreamControl(EMANE::ControlMessage(EMANE::FlowControlTokenRequest::MAJOR_ID,
                                                          EMANE::FlowControlTokenRequest::MINOR_ID,
                                                          &flowControlRequest, 
                                                          sizeof(flowControlRequest)));
}


inline
void EMANEUtils::FlowControl::stop()
{
  mutex_.acquire();

  // set cancel flag
  bCancel_ = true;

  // unblock pending request if any
  cond_.signal();

  mutex_.release();
}


inline
EMANEUtils::FlowControl::FlowControlStatus
EMANEUtils::FlowControl::removeToken()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  // request status 
  bool bStatus;

  // wait unitil tokens are available, and not canceled 
  while(ui16TokensAvailable_ == 0 && !bCancel_)
    {
      cond_.wait();
    }
 
  // check canceled flag
  if(bCancel_)
    {
      // failure
      bStatus = false;
    }
  else
    {
      // success
      bStatus = true;
 
      // remove token 
      --ui16TokensAvailable_;
    }
  
  return EMANEUtils::FlowControl::FlowControlStatus (bStatus, ui16TokensAvailable_);
}


inline
EMANE::UINT16 EMANEUtils::FlowControl::getAvailableTokens()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  return ui16TokensAvailable_;
}


inline
void EMANEUtils::FlowControl::processFlowControlMessage(const EMANE::ControlMessage & ctrl)
{
  // check message type
  if(ctrl.getMajorIdentifier() == EMANE::FlowControlTokenResponse::MAJOR_ID &&
     ctrl.getMinorIdentifier() == EMANE::FlowControlTokenResponse::MINOR_ID)
    {
      EMANE::FlowControlTokenResponse p(*reinterpret_cast<const EMANE::FlowControlTokenResponse *>(ctrl.get()));

      // convert to host byte order      
      p.toHostByteOrder();
      
      ACE_Guard<ACE_Thread_Mutex> m(mutex_);

      // update count
      ui16TokensAvailable_ = p.ui16TokensAvailable_;

      // send ack
      sendFlowControlMessageAck();

      // unblock pending request if any
      cond_.signal();  
    }
}
 

inline
void EMANEUtils::FlowControl::sendFlowControlMessageAck()
{
  // create ack message with current tokens available
  EMANE::FlowControlTokenResponseAck flowControlResponseAck(ui16TokensAvailable_);
 
  // conver to network byte order 
  flowControlResponseAck.toNetworkByteOrder();
 
  // send message 
  rTransport_.sendDownstreamControl(EMANE::ControlMessage(EMANE::FlowControlTokenResponseAck::MAJOR_ID,
                                                          EMANE::FlowControlTokenResponseAck::MINOR_ID,
                                                          &flowControlResponseAck, 
                                                          sizeof(flowControlResponseAck)));
}
