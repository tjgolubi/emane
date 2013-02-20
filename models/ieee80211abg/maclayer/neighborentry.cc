/*
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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


#include "neighborentry.h"

IEEE80211ABG::NeighborEntry::NeighborEntry ():
 tvLastActivityTime_(ACE_Time_Value::zero),
 fEstimatedNumCommonNeighbors_(0),
 fHiddenChannelActivity_(0.0),
 dAverageHiddenRxPowerMilliWatts_(0.0),
 dAverageCommonRxPowerMilliWatts_(0.0)
{
  utilizationTypeMap_.insert(std::make_pair(IEEE80211ABG::MSG_TYPE_BROADCAST_DATA,        IEEE80211ABG::NeighborEntry::Utilization()));
  utilizationTypeMap_.insert(std::make_pair(IEEE80211ABG::MSG_TYPE_UNICAST_DATA,          IEEE80211ABG::NeighborEntry::Utilization()));
  utilizationTypeMap_.insert(std::make_pair(IEEE80211ABG::MSG_TYPE_UNICAST_RTS_CTS_DATA,  IEEE80211ABG::NeighborEntry::Utilization()));
  utilizationTypeMap_.insert(std::make_pair(IEEE80211ABG::MSG_TYPE_UNICAST_CTS_CTRL,      IEEE80211ABG::NeighborEntry::Utilization()));
}



ACE_Time_Value 
IEEE80211ABG::NeighborEntry::getLastActivityTime () const
{
  return tvLastActivityTime_;
}



ACE_Time_Value IEEE80211ABG::NeighborEntry::getBandwidtUtilization (ACE_UINT32 msgTypeMask) const
{
  ACE_Time_Value result = ACE_Time_Value::zero;

  // all utilization types
  for(UtilizationTypeMapConstIter iter = utilizationTypeMap_.begin(); iter != utilizationTypeMap_.end(); ++iter)
   {
     // check mask
     if((1 << (iter->first - 1)) & msgTypeMask)
      {
        result += iter->second.tvTotalBWUtilization_;
      }
   }

  return result;
}



float 
IEEE80211ABG::NeighborEntry::getHiddenChannelActivity () const
{
  return fHiddenChannelActivity_;
}



void
IEEE80211ABG::NeighborEntry::setHiddenChannelActivity (float fActivity)
{
  fHiddenChannelActivity_ = fActivity;
}



size_t 
IEEE80211ABG::NeighborEntry::getNumberOfPackets (ACE_UINT32 msgTypeMask) const
{
  size_t result = 0;

  // all utilization types
  for(UtilizationTypeMapConstIter iter = utilizationTypeMap_.begin(); iter != utilizationTypeMap_.end(); ++iter)
   {
     // check mask
     if((1 << (iter->first - 1)) & msgTypeMask)
      {
        result += iter->second.totalNumPackets_;
      }
   }
 
  return result;
}



float 
IEEE80211ABG::NeighborEntry::getEstimatedNumCommonNeighbors () const
{
  return fEstimatedNumCommonNeighbors_;
}




void 
IEEE80211ABG::NeighborEntry::setEstimatedNumCommonNeighbors (float num)
{
  fEstimatedNumCommonNeighbors_ = num;
}



double
IEEE80211ABG::NeighborEntry::getAverageHiddenRxPowerMilliWatts () const
{
  return dAverageHiddenRxPowerMilliWatts_;
}



void
IEEE80211ABG::NeighborEntry::setAverageHiddenRxPowerMilliWatts (double dAverageHiddenRxPowerMilliWatts)
{
  dAverageHiddenRxPowerMilliWatts_ = dAverageHiddenRxPowerMilliWatts;
}


double
IEEE80211ABG::NeighborEntry::getAverageCommonRxPowerMilliWatts () const
{
  return dAverageCommonRxPowerMilliWatts_;
}


void
IEEE80211ABG::NeighborEntry::setAverageCommonRxPowerMilliWatts (double dAverageCommonRxPowerMilliWatts)
{
  dAverageCommonRxPowerMilliWatts_ = dAverageCommonRxPowerMilliWatts;
}


double
IEEE80211ABG::NeighborEntry::getRxPowerMilliWatts (ACE_UINT32 msgTypeMask) const
{
  double result = 0.0;

  // all utilization types
  for(UtilizationTypeMapConstIter iter = utilizationTypeMap_.begin(); iter != utilizationTypeMap_.end(); ++iter)
   {
     // check mask
     if((1 << (iter->first - 1)) & msgTypeMask)
      {
        result += iter->second.dTotalRxPowerMilliWatts_;
      }
   }
 
  return result;
}


void 
IEEE80211ABG::NeighborEntry::resetUtilization (ACE_UINT32 msgTypeMask)
{
  // all utilization types
  for(UtilizationTypeMapIter iter = utilizationTypeMap_.begin(); iter != utilizationTypeMap_.end(); ++iter)
   {
     // check mask
     if((1 << (iter->first - 1)) & msgTypeMask)
      {
        // reset entry
        iter->second.reset();
      }
   }
}


void
IEEE80211ABG::NeighborEntry::updateChannelActivity (const ACE_Time_Value & tvBandWidth, 
                                                    ACE_UINT8 type,
                                                    const ACE_Time_Value & tvTime, 
                                                    double dRxPowerMilliWatts, 
                                                    size_t numPackets)
{
  const UtilizationTypeMapIter iter = utilizationTypeMap_.find(type);

  if(iter != utilizationTypeMap_.end())
   {
     iter->second.update(numPackets, tvBandWidth, dRxPowerMilliWatts);
   }

  tvLastActivityTime_ = tvTime;
}



void 
IEEE80211ABG::NeighborEntry::setOneHopNeighbors(const IEEE80211ABG::NbrSet & nbrs)
{
  oneHopNbrSet_ = nbrs;
}



void 
IEEE80211ABG::NeighborEntry::setHiddenNeighbors(const IEEE80211ABG::NbrSet & nbrs)
{
  hiddenNbrSet_ = nbrs;
}



void 
IEEE80211ABG::NeighborEntry::setCommonNeighbors(const IEEE80211ABG::NbrSet & nbrs)
{
  commonNbrSet_ = nbrs;
}



const IEEE80211ABG::NbrSet & 
IEEE80211ABG::NeighborEntry::getOneHopNeighbors() const
{
  return oneHopNbrSet_;
}


const IEEE80211ABG::NbrSet & 
IEEE80211ABG::NeighborEntry::getCommonNeighbors() const
{
  return commonNbrSet_;
}


const IEEE80211ABG::NbrSet & 
IEEE80211ABG::NeighborEntry::getHiddenNeighbors() const
{
  return hiddenNbrSet_;
}


bool 
IEEE80211ABG::NeighborEntry::isOneHopNbr(EMANE::NEMId id) const
{
   return oneHopNbrSet_.find(id) != oneHopNbrSet_.end();
}


bool 
IEEE80211ABG::NeighborEntry::isCommonNbr(EMANE::NEMId id) const
{
   return commonNbrSet_.find(id) != commonNbrSet_.end();
}


bool 
IEEE80211ABG::NeighborEntry::isHiddenNbr(EMANE::NEMId id) const
{
   return hiddenNbrSet_.find(id) != hiddenNbrSet_.end();
}
