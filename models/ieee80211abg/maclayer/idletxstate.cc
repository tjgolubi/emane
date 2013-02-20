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

#include "idletxstate.h"

#include "multicastpretxstate.h"
#include "unicastpretxstate.h"
#include "unicastrtsctspretxstate.h"

#include "modetimingparameters.h"
#include "ieee80211abgmaclayer.h"
#include "macstatistics.h"

IEEE80211ABG::IdleTxState::IdleTxState ()
{ }

IEEE80211ABG::IdleTxState::~IdleTxState ()
{ }


bool
IEEE80211ABG::IdleTxState::process (IEEE80211ABG::IEEE80211abgMACLayer * pMgr,
                                    IEEE80211ABG::DownstreamQueueEntry & entry)
{
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();

  // multicast/broadcast 
  if (entry.pkt_.getPacketInfo ().destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS)
    {
      // packet txop timed out
      if ((pMgr->getConfig ().getTxOpTime (entry.qidx_) != ACE_Time_Value::zero) && 
           (pMgr->getConfig ().getTxOpTime (entry.qidx_) + entry.tvAcquireTime_) < tvCurrentTime)
        {
          // set discard due to txop timeout
          pMgr->getStatistics ().incrementDownstreamMulticastDataDiscardDueToTxop ();

          // get next packet
          return false;
        }
      else
        {
          // set max retries 
          entry.maxRetries_ = 0;

          // set pre/post tx delay time
          pMgr->setDelayTime(entry);

          // change state to multicast pre tx state
          changeState (pMgr, IEEE80211ABG::MulticastPreTxStateSingleton::instance ());

          // continue to process
          return true;
        }
    }
  // unicast 
  else
    {
      // packet txop timed out
      if ((pMgr->getConfig ().getTxOpTime (entry.qidx_) != ACE_Time_Value::zero) && 
           (pMgr->getConfig ().getTxOpTime (entry.qidx_) + entry.tvAcquireTime_) < tvCurrentTime)
        {
          // set discard due to txop timeout
          pMgr->getStatistics ().incrementDownstreamUnicastDataDiscardDueToTxop ();

          // get next packet
          return false;
        }
      else
        {
          // set max retries
          entry.maxRetries_ = pMgr->getConfig ().getRetryLimit (entry.qidx_);

          // set pre/post tx delay time
          pMgr->setDelayTime(entry);

          // check rts cts enable
          if(entry.bRtsCtsEnable_ == true)
           {
             // change state to unicast rts cts pre tx state
             changeState (pMgr, IEEE80211ABG::UnicastRtsCtsPreTxStateSingleton::instance ());
           }
          else
           {
             // change state to unicast pre tx state
             changeState (pMgr, IEEE80211ABG::UnicastPreTxStateSingleton::instance ());
           }

          // continue to process
          return true;
        }
    }
}



ACE_Time_Value
IEEE80211ABG::IdleTxState::getWaitTime (IEEE80211ABG::DownstreamQueueEntry &)
{
  // no wait time
  return ACE_Time_Value::zero;
}


const char *
IEEE80211ABG::IdleTxState::statename ()
{
  return "IdleTxState";
}
