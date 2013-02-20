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

#ifndef IEEE80211ABG_NEIGHBORENTRY_HEADER_
#define IEEE80211ABG_NEIGHBORENTRY_HEADER_

#include "emane/emanetypes.h"
#include "neighbortype.h"
#include "msgtypes.h"

#include <ace/Time_Value.h>

#include <map>
#include <set>

namespace IEEE80211ABG
{
  /**
   *
   * @class  NeighborEntry
   *
   * @brief Defines a 1 hop nbr and its bandwidth utilization
   *
   */
  class NeighborEntry
  {
  public:
   /**
    *
    * constructor
    *
    */
    NeighborEntry ();

   /**
    *
    * updates the bandwidth utilization parameters
    *
    * @param tvBandWidth       the bandwidth (duration) of channel activity
    * @param type              the msg type (unicast, unicast rts/cts, broadcast)
    * @param tvTime            the absolute time that the activity
    * @param dRxPowerMilliWatts the rx power in milliwatts
    * @param numPackets        the number of packets associated with the activity default is 1
    *
    */
    void updateChannelActivity (const ACE_Time_Value & tvBandWidth, 
                                ACE_UINT8 type,
                                const ACE_Time_Value & tvTime, 
                                double dRxPowerMilliWatts, 
                                size_t numPackets = 1);

   /**
    *
    * resets the bandwidth utilization parameter(s)
    *
    * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast) default is all msg types
    *
    */
    void resetUtilization (ACE_UINT32 msgTypeMask = MSG_TYPE_MASK_ALL);

    ACE_Time_Value getLastActivityTime () const;

   /**
    *
    * gets the bandiwdth utilzation (duration) time
    *
    * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast)
    *
    * @return the bandiwdth utilzation
    *
    */
    ACE_Time_Value getBandwidtUtilization (ACE_UINT32 msgTypeMask) const;

   /**
    *
    * gets the number of utilization packets
    *
    * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast)
    *
    * @return the number of utilization packets
    *
    */
    size_t getNumberOfPackets (ACE_UINT32 msgTypeMask) const;


   /**
    *
    * gets the utilization rx power
    *
    * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast)
    *
    * @return the utilization rx power sum
    *
    */
    double getRxPowerMilliWatts (ACE_UINT32 msgTypeMask) const;


    float getEstimatedNumCommonNeighbors () const;


    void setEstimatedNumCommonNeighbors (float num);


    float getHiddenChannelActivity () const;


    void setHiddenChannelActivity (float fActivity);


    void setAverageHiddenRxPowerMilliWatts (double dAverageHiddenRxPowerMilliWatts);


    double getAverageHiddenRxPowerMilliWatts () const;


    void setAverageCommonRxPowerMilliWatts (double dAverageCommonRxPowerMilliWatts);


    double getAverageCommonRxPowerMilliWatts () const;


    void setOneHopNeighbors(const NbrSet & nbrs);


    void setHiddenNeighbors(const NbrSet & nbrs);


    void setCommonNeighbors(const NbrSet & nbrs);


    const NbrSet & getOneHopNeighbors() const;


    const NbrSet & getCommonNeighbors() const;


    const NbrSet & getHiddenNeighbors() const;


    bool isOneHopNbr(EMANE::NEMId id) const;


    bool isCommonNbr(EMANE::NEMId id) const;


    bool isHiddenNbr(EMANE::NEMId id) const;

  private:
  /**
   *
   * @brief Defines a 1 hop nbr bandwidth utilization
   *
   */

    struct Utilization
      {
         ACE_Time_Value tvTotalBWUtilization_;

         size_t         totalNumPackets_;

         double         dTotalRxPowerMilliWatts_;

         Utilization() :
          tvTotalBWUtilization_(ACE_Time_Value::zero),
          totalNumPackets_(0),
          dTotalRxPowerMilliWatts_(0)
         { }

         void reset()
         {
           tvTotalBWUtilization_    = ACE_Time_Value::zero;
           totalNumPackets_         = 0;
           dTotalRxPowerMilliWatts_ = 0;
         }

         void update(size_t numPackets, const ACE_Time_Value & tvBandWidth, double dRxPowerMilliWatts)
          {
             totalNumPackets_ += numPackets;

             tvTotalBWUtilization_ += tvBandWidth;

             dTotalRxPowerMilliWatts_ += dRxPowerMilliWatts;
          }
      };

    typedef std::map<ACE_UINT8, Utilization> UtilizationTypeMap;
    typedef UtilizationTypeMap::iterator UtilizationTypeMapIter;
    typedef UtilizationTypeMap::const_iterator UtilizationTypeMapConstIter;

    UtilizationTypeMap utilizationTypeMap_;

    ACE_Time_Value tvLastActivityTime_;

    NbrSet oneHopNbrSet_;

    NbrSet commonNbrSet_;

    NbrSet hiddenNbrSet_;

    float fEstimatedNumCommonNeighbors_;

    float fHiddenChannelActivity_;

    double dAverageHiddenRxPowerMilliWatts_;

    double dAverageCommonRxPowerMilliWatts_;
  };
}
#endif                          //IEEE80211ABG_NEIGHBORENTRY_HEADER_
