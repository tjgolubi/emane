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

#ifndef IEEE80211ABG_NEIGHBOR2HOPENTRY_HEADER_
#define IEEE80211ABG_NEIGHBOR2HOPENTRY_HEADER_

#include <ace/Time_Value.h>

namespace IEEE80211ABG
{
  /**
   *
   * @class  Neighbor2HopEntry
   *
   * @brief Defines a 2 hop nbr and its bandwidth utilization
   *
   */
  class Neighbor2HopEntry
  {
   public:
   /**
    *
    * constructor
    *
    */

     Neighbor2HopEntry ();

   /**
    *
    * gets the last channel activity absolute time
    *
    * @return the  last channel activity absolute time
    *
    */
    ACE_Time_Value getLastActivityTime () const;


   /**
    *
    * updates the bandwidth utilization parameters
    *
    * @param tvBandWidth the bandwidth (duration) of channel activity
    * @param tvTime      the absolute time that the activity
    * @param numPackets  the number of packets associated with the activity default is 1
    *
    */
    void updateChannelActivity (const ACE_Time_Value & tvBandWidth, 
                                const ACE_Time_Value & tvTime, 
                                size_t numPackets = 1);


   /**
    *
    * resets the bandwidth utilization parameters
    *
    */
    void resetUtilization ();

   /**
    *
    * gets the bandwidth utilization time
    *
    * @return the bandwidth utilization time
    *
    */
    ACE_Time_Value getBandwidtUtilization () const;

   /**
    *
    * gets the number of utilization packets
    *
    * @return the number of utilization packets
    *
    */
    size_t getNumberOfPackets () const;

   private:
  /**
   *
   * @brief Defines a 2 hop nbr bandwidth utilization
   *
   */
    struct Utilization
      {
         ACE_Time_Value tvTotalBWUtilization_;

         size_t         totalNumPackets_;

         Utilization() :
          tvTotalBWUtilization_(ACE_Time_Value::zero),
          totalNumPackets_(0)
         { }

         void reset()
         {
           tvTotalBWUtilization_ = ACE_Time_Value::zero;
           totalNumPackets_      = 0;
         }

         void update(size_t numPackets, const ACE_Time_Value & tvBandWidth)
         {
            totalNumPackets_ += numPackets;

            tvTotalBWUtilization_ += tvBandWidth;
         }
      };

     ACE_Time_Value tvLastActivityTime_;

     Utilization utilization_;
  };
}
#endif                          //IEEE80211ABG_NEIGHBOR2HOPENTRY_HEADER_
