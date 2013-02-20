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

#include "unicastrtsctstxstate.h"
#include "unicastrtsctsposttxstate.h"
#include "modetimingparameters.h"
#include "ieee80211abgmaclayer.h"
#include "ieee80211abgmacheader.h"

IEEE80211ABG::UnicastRtsCtsTxState::UnicastRtsCtsTxState ()
{ }

IEEE80211ABG::UnicastRtsCtsTxState::~UnicastRtsCtsTxState ()
{ }


bool
IEEE80211ABG::UnicastRtsCtsTxState::process (IEEE80211ABG::IEEE80211abgMACLayer * pMgr, 
                                       IEEE80211ABG::DownstreamQueueEntry & entry)
{
  // duration interval unicast rts cts
  entry.tvDurationInterval_ = pMgr->getModeTiming ().getMessageDuration (IEEE80211ABG::MSG_TYPE_UNICAST_RTS_CTS_DATA, entry.pkt_.length ());

  // set tx time
  entry.tvTxTime_ = ACE_OS::gettimeofday ();

  // send packet
  pMgr->sendDownstreamUnicastData (entry);

  // change state to unicast rts cts post tx
  changeState (pMgr, IEEE80211ABG::UnicastRtsCtsPostTxStateSingleton::instance ());

  // continue to process
  return true;
}


ACE_Time_Value 
IEEE80211ABG::UnicastRtsCtsTxState::getWaitTime (IEEE80211ABG::DownstreamQueueEntry &)
{
  // no wait time
  return ACE_Time_Value::zero;
}


const char *
IEEE80211ABG::UnicastRtsCtsTxState::statename ()
{
  return "UnicastRtsCtsTxState";
}
