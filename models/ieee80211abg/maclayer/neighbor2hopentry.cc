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


#include "neighbor2hopentry.h"

IEEE80211ABG::Neighbor2HopEntry::Neighbor2HopEntry ():
 tvLastActivityTime_(ACE_Time_Value::zero)
{ }



ACE_Time_Value IEEE80211ABG::Neighbor2HopEntry::getLastActivityTime () const
{
  return tvLastActivityTime_;
}



void 
IEEE80211ABG::Neighbor2HopEntry::resetUtilization ()
{
   utilization_.reset();
}


void
IEEE80211ABG::Neighbor2HopEntry::updateChannelActivity (const ACE_Time_Value & tvBandWidth, 
                                                        const ACE_Time_Value & tvTime, 
                                                        size_t numPackets)
{
   utilization_.update(numPackets, tvBandWidth);

   tvLastActivityTime_ = tvTime;
}



size_t 
IEEE80211ABG::Neighbor2HopEntry::getNumberOfPackets () const
{
  return  utilization_.totalNumPackets_;
}


ACE_Time_Value IEEE80211ABG::Neighbor2HopEntry::getBandwidtUtilization () const
{
   return utilization_.tvTotalBWUtilization_;
}



