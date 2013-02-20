/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANE_UNIVERSALPHYCONTROLRECVMESSAGE_HEADER_
#define EMANE_UNIVERSALPHYCONTROLRECVMESSAGE_HEADER_

#include "emanecontrolmessages/controlmessages.h"
#include "emane/emanecontrolmessage.h"
#include "emanecontrolmessages/universalphycontrolrecvmessageexception.h"
#include "emane/emanecommonphyheader.h"
#include "emane/emanecommonphyfreqinfo.h"
#include "emane/emanenet.h"

#include <ace/Time_Value.h>
#include <ace/Basic_Types.h>

#include <vector>

namespace EMANE
{
  /**
   * @class UniversalPhyControlRecvMessage
   *
   * @brief universal phy control recv message
   *
   */
  class UniversalPhyControlRecvMessage
  { 
     public:
      static const ACE_INT32 MAJOR_ID = EMANE_UNIVERSALPHYCONTROL_MAJOR_ID;
      static const ACE_INT32 MINOR_ID = EMANE_UNIVERSALPHYCONTROL_RECV_MINOR_ID;

      /**
       * @class TransmitterInfo
       *
       * @brief info about a transmitting (received from) NEM
       *
       */
       class TransmitterInfo
        {
          public:
          /**
           *
           * @brief  default constructor
           *
           */
           TransmitterInfo() :
             u16Src_(0),
             i16RxPowerdBmScaled_(0),
             u32PropagationDelaySec_(0),
             u32PropagationDelayUsec_(0)
            { }

          /**
           *
           * @brief  parameter constructor
           *
           * @param src the transmitting NEM
           * @param fRxPowerdBm the receive power in dBm
           * @param tv the absolute receive time
           *
           */
           TransmitterInfo(const ACE_UINT16 src, const float fRxPowerdBm, const ACE_Time_Value & tv) :
             u16Src_(src),
             i16RxPowerdBmScaled_(fRxPowerdBm * DB_SCALE_FACTOR),
             u32PropagationDelaySec_(tv.sec()),
             u32PropagationDelayUsec_(tv.usec())
            { }
 
           /**
            *
            * @return returns the src NEM id
            *
            */
            ACE_UINT16 getSrc() const 
              { 
                return u16Src_;
              }

           /**
            *
            * @return returns the received power in dBm
            *
            */
            float getRxPowerdBm() const 
              { 
                return static_cast<float> (i16RxPowerdBmScaled_) / DB_SCALE_FACTOR; 
              }

           /**
            *
            * @return returns the propagation delay if available via location info
            *
            */
            ACE_Time_Value getPropagationDelay() const
              { 
                 return ACE_Time_Value(u32PropagationDelaySec_, u32PropagationDelayUsec_);
              }

           /**
            *
            * @brief  returns a formated string of paramters and values for logging
            *
            */
            std::string format() const
              {
                char fmtBuff[256];

                snprintf(fmtBuff, sizeof(fmtBuff), "transmitter: %hu, rx pwr %5.4f dBm, prop delay %ld:%06ld", 
                         getSrc(),
                         getRxPowerdBm(),
                         getPropagationDelay().sec(),
                         getPropagationDelay().usec());

                return fmtBuff;
              }

           /**
            *
            * @brief  convert values to host byte order
            *
            */
           void toHostByteOrder()
            {
              // short
              u16Src_                  = ACE_NTOHS(u16Src_);
              i16RxPowerdBmScaled_     = ACE_NTOHS(i16RxPowerdBmScaled_);

              // long
              u32PropagationDelaySec_  = ACE_NTOHL(u32PropagationDelaySec_);
              u32PropagationDelayUsec_ = ACE_NTOHL(u32PropagationDelayUsec_);
            }

           /**
            *
            * @brief  convert values to network byte order
            *
            */
           void toNetworkByteOrder()
            {
              // short
              u16Src_                  = ACE_HTONS(u16Src_);
              i16RxPowerdBmScaled_     = ACE_HTONS(i16RxPowerdBmScaled_);

              // long
              u32PropagationDelaySec_  = ACE_HTONL(u32PropagationDelaySec_);
              u32PropagationDelayUsec_ = ACE_HTONL(u32PropagationDelayUsec_);
            }

           private:         
            ACE_UINT16    u16Src_;                   // the src NEM (transmitter)
            ACE_INT16     i16RxPowerdBmScaled_;      // rx power in dBm
            ACE_UINT32    u32PropagationDelaySec_;   // propagation delay in seconds
            ACE_UINT32    u32PropagationDelayUsec_;  // propagation delay in useconds
       }__attribute__((packed));

       // transmitter info container and iterators
       typedef std::vector<TransmitterInfo>         TransmitterInfoItems;
       typedef TransmitterInfoItems::iterator       TransmitterInfoItemsIter;
       typedef TransmitterInfoItems::const_iterator TransmitterInfoItemsConstIter;


      /**
       *
       * @brief default constructor
       *
       */
      UniversalPhyControlRecvMessage();

      /**
       *
       * @brief parmater constructor
       *
       * @param msg EMANE control message
       * @exception throws  UniversalPHYControlRecvMessageException
       *
       */
      UniversalPhyControlRecvMessage(const EMANE::ControlMessage & msg)
       throw(UniversalPHYControlRecvMessageException);

      /**
       *
       * @return returns the bandwidth in Hz
       *
       */
      ACE_UINT64 getBandWidthHz() const;

      /**
       *
       * @param u64BandWidthHz the bandwidth in Hz
       *
       */
      void setBandWidthHz(ACE_UINT64 u64BandWidthHz);

      /**
       *
       * @param tv the absoulute tx time
       *
       */
      void setTxTime(const ACE_Time_Value & tv);

      /**
       *
       * @return returns the absoulute tx time
       *
       */
      ACE_Time_Value getTxTime() const;

      /**
       * set frequency info vector
       *
       * @param vec vector of frequency info data
       * @exception throws  UniversalPHYControlRecvMessageException
       *
       */
      void setFrequencyInfo (const EMANE::PHYRxFrequencyInfoItems & vec)
         throw(UniversalPHYControlRecvMessageException);

      /**
       * get frequency info vector
       *
       * @return vec vector of frequency info data
       *
       */
      const EMANE::PHYRxFrequencyInfoItems & getFrequencyInfo () const;


      /**
       * set transmitter info vector
       *
       * @param vec vector of transmitter info data
       * @exception throws  UniversalPHYControlRecvMessageException
       *
       */
      void setTransmitterInfo (const TransmitterInfoItems & vec)
         throw(UniversalPHYControlRecvMessageException);

      /**
       * get transmitter info vector
       *
       * @return vec vector of transmitter info data
       *
       */
      const TransmitterInfoItems & getTransmitterInfo () const;

      /**
       *
       * @brief  returns a formated string of paramters and values for logging
       *
       */
      std::string format(const ACE_Time_Value & tv) const;

      /**
       *
       * @return returns a complete EMANE ControlMessage 
       *
       */
      EMANE::ControlMessage buildControlMessage();

      /**
       *
       * @param msg unpacks a complete EMANE ControlMessage
       * @exception throws  UniversalPHYControlRecvMessageException
       *
       */
      void unpackControlMessage(const EMANE::ControlMessage & msg)
       throw(UniversalPHYControlRecvMessageException);

    private:
      /**
       * @struct  Data 
       *
       * @brief  private class data 
       *
       */
      struct Data {
        ACE_UINT64  u64BandWidthHz_;                   // frequency bandwidth in Hz
        ACE_UINT32  u32TxTimeSec_;                     // pkt abs tx time in sec (the transmitters tx time)
        ACE_UINT32  u32TxTimeUsec_;                    // pkt abs tx time in usec(the transmitters tx time)
        ACE_UINT8   u8NumberOfFrequencies_;            // number of optional frequency info entries
        ACE_UINT8   u8NumberOfTransmitters_;           // number of transmitters (total)

        /**
         *
         * @brief default constructor
         *
         */
        Data () :
           u64BandWidthHz_(0),
           u32TxTimeSec_(0),
           u32TxTimeUsec_(0),
           u8NumberOfFrequencies_(0),
           u8NumberOfTransmitters_(0)
        { } 

        /**
         *
         * @brief parameter constructor
         *
         * @param  u64BandWidthHz the bandwidth in Hz
         * @param tvTxTime the absolute tx time
         *
         */

        Data (const ACE_UINT64 u64BandWidthHz, const ACE_Time_Value & tvTxTime) :
           u64BandWidthHz_(u64BandWidthHz),
           u32TxTimeSec_(tvTxTime.sec()),
           u32TxTimeUsec_(tvTxTime.usec()),
           u8NumberOfFrequencies_(0),
           u8NumberOfTransmitters_(0)
        { }
      } __attribute__((packed)) data_;


      // scale factor for rx power
      static const float DB_SCALE_FACTOR = 100.0;

      // option limits
      const static ACE_UINT8 MAX_NUM_FREQUENCY_INFO_ENTRIES = 255; 
      const static ACE_UINT8 MAX_NUM_TRANSMITTER_ENTRIES    = 255; 

      void toNetworkByteOrder(Data & data);

      void toHostByteOrder(Data & data);

      EMANE::PHYRxFrequencyInfoManager frequencyInfoManager_;

      TransmitterInfoItems vec_;
   };
}

#include "universalphycontrolrecvmessage.inl"

#endif //EMANE_UNIVERSALPHYCONTROLRECVMESSAGE_HEADER_
