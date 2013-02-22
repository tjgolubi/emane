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

#ifndef EMANE_UNIVERSALPHYCONTROLSENDMESSAGE_HEADER_
#define EMANE_UNIVERSALPHYCONTROLSENDMESSAGE_HEADER_

#include "emanecontrolmessages/controlmessages.h"
#include "emane/emanecontrolmessage.h"
#include "emanecontrolmessages/universalphycontrolsendmessageexception.h"
#include "emane/emanecommonphyheader.h"
#include "emane/emanecommonphyfreqinfo.h"
#include "emane/emanenet.h"

#include <ace/Time_Value.h>
#include <ace/Basic_Types.h>


namespace EMANE
{
  /**
   * @class UniversalPhyControlSendMessage
   *
   * @brief universal phy control send message
   *
   */
  class UniversalPhyControlSendMessage
  { 

    public:
      static const ACE_INT32 MAJOR_ID = EMANE_UNIVERSALPHYCONTROL_MAJOR_ID;
      static const ACE_INT32 MINOR_ID = EMANE_UNIVERSALPHYCONTROL_SEND_MINOR_ID;

      /**
       * @class TransmitterInfo
       *
       * @brief info about a transmitting NEM
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
             i16TxPowerdBmScaled_(0)
            { }

          /**
           *
           * @brief  parameter constructor
           *
           * @param src the transmitting NEM
           * @param fTxPowerdBm the transmit power in dBm
           *
           */

           TransmitterInfo(const ACE_UINT16 src, const float fTxPowerdBm) :
             u16Src_(src),
             i16TxPowerdBmScaled_(fTxPowerdBm * DB_SCALE_FACTOR)
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
            * @return returns the transmit power in dBm
            *
            */
            float getTxPowerdBm() const 
              { 
                return static_cast<float> (i16TxPowerdBmScaled_) / DB_SCALE_FACTOR; 
              }

           /**
            *
            * @brief  returns a formated string of paramters and values for logging
            *
            */
            std::string format() const
              {
                char fmtBuff[256];

                snprintf(fmtBuff, sizeof(fmtBuff), "transmitter: %hu, tx pwr %5.4f dBm", 
                         getSrc(),
                         getTxPowerdBm());

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
              u16Src_               = ACE_NTOHS(u16Src_);
              i16TxPowerdBmScaled_  = ACE_NTOHS(i16TxPowerdBmScaled_);
            }

           /**
            *
            * @brief  convert values to network byte order
            *
            */
           void toNetworkByteOrder()
            {
              // short
              u16Src_               = ACE_HTONS(u16Src_);
              i16TxPowerdBmScaled_  = ACE_HTONS(i16TxPowerdBmScaled_);
            }

           private:         
            ACE_UINT16    u16Src_;                   // the NEM
            ACE_INT16     i16TxPowerdBmScaled_;      // tx power in dBm
      } __attribute__((packed));

       typedef std::vector<TransmitterInfo>         TransmitterInfoItems;
       typedef TransmitterInfoItems::iterator       TransmitterInfoItemsIter;
       typedef TransmitterInfoItems::const_iterator TransmitterInfoItemsConstIter;


      /**
       *
       * @brief default constructor
       *
       */
      UniversalPhyControlSendMessage();

      /**
       *
       * @brief parmater constructor
       *
       * @param msg EMANE control message
       * @exception throws  UniversalPHYControlSendMessageException
       *
       */
      UniversalPhyControlSendMessage(const EMANE::ControlMessage & msg)
        throw(UniversalPHYControlSendMessageException);

     /**
      *
      * @param  bit the requested bit
      * @return returns true if bit is set else returns false
      *
      */
      bool isBitSet(const ACE_UINT32 bit) const;

     /**
      *
      * @param  bit the requested bit
      *
      */
      void setBit(const ACE_UINT32 bit);

     /**
      *
      * @return returns the bandwidth in Hz
      *
      */

      ACE_UINT64 getBandWidthHz() const;

     /**
      *
      * @param  u64BandWidthHz the frequency in Hz
      *
      */
      void setBandWidthHz(ACE_UINT64 u64BandWidthHz);

     /**
      *
      * @return returns the antenn profile id
      *
      */
      ACE_UINT16 getAntennaProfileId() const;

     /**
      *
      * @param  u16ProfileId the antenna profile id
      *
      */
      void setAntennaProfileId(const ACE_UINT16 u16ProfileId);

     /**
      *
      * @return returns the antenna azimuth in degrees
      *
      */
      float getAntennaAzimuthDegrees() const;

     /**
      *
      * @param  fAntennaAzimuthDegrees the antenna azimuth in degrees
      *
      */
      void setAntennaAzimuthDegrees(float fAntennaAzimuthDegrees);

     /**
      *
      * @return returns the antenna elevation in degrees
      *
      */
      float getAntennaElevationDegrees() const;

     /**
      *
      * @param  fAntennaElevationDegrees the antenna elevation in degrees
      *
      */
      void setAntennaElevationDegrees(float fAntennaElevationDegrees);

     /**
      *
      * @return returns the tx power in dBm
      *
      */
      float getTxPowerdBm() const;

     /**
      *
      * @param  fTxPowerdBm the tx power in dBm
      *
      */
      void setTxPowerdBm(float fTxPowerdBm);

      /**
       *
       * @param items container of TransmitterInfoItems 
       * @exception throws  UniversalPHYControlSendMessageException
       *
       */
      void setAdditionalTransmitters (const TransmitterInfoItems & items)
         throw(UniversalPHYControlSendMessageException);

      /**
       *
       * @return returns a const reference to a container of TransmitterInfoItems 
       *
       */
      const TransmitterInfoItems & getAdditionalTransmitters () const;

      /**
       *
       * @return returns a const reference to a container of PHYTxFrequencyInfoItems 
       *
       */
      const EMANE::PHYTxFrequencyInfoItems & getFrequencyInfo () const;

      /**
       *
       * @param items container of  PHYTxFrequencyInfoItems 
       * @exception throws  UniversalPHYControlSendMessageException
       *
       */
      void setFrequencyInfo (const EMANE::PHYTxFrequencyInfoItems & items)
         throw(UniversalPHYControlSendMessageException);
     
      /**
       *
       * @brief set valid value(s) that are set by the caller
       * @param msg  UniversalPhyControlSendMessage 
       * @return returns number of values that were set
       *
       */
      int overWrite(const UniversalPhyControlSendMessage & msg);

      /**
       *
       * @return returns a complete EMANE ControlMessage 
       *
       */
      EMANE::ControlMessage buildControlMessage();

      /**
       *
       * @param msg unpacks a complete EMANE ControlMessage
       * @exception throws  UniversalPHYControlSendMessageException
       *
       */
      void unpackControlMessage(const EMANE::ControlMessage & msg)
       throw(UniversalPHYControlSendMessageException);

    /**
      *
      * @brief  returns a formated string of paramters and values for logging
      *
      */
      std::string format() const;

    private:
     /**
       * @struct  Data 
       *
       * @brief  private class data 
       *
       */
      struct Data {
        ACE_UINT64  u64BandWidthHz_;                   // frequency bandwidth in Hz
        ACE_INT32   i32AzimuthMARCS_;                  // antenna azimuth in milli arc seconds
        ACE_INT32   i32ElevationMARCS_;                // antenna evevation in milli arc seconds
        ACE_UINT32  u32BitMask_;                       // bit mask
        ACE_INT16   i16TxPowerdBmScaled_;              // tx power in dBm
        ACE_UINT16  u16ProfileId_;                     // antenna profile id
        ACE_UINT8   u8NumberOfFrequencies_;            // number of optional frequency info entries
        ACE_UINT8   u8NumberOfTransmitters_;           // number of optional transmitters

        /**
         *
         * @brief default constructor
         *
         */
        Data () :
           u64BandWidthHz_(0),
           i32AzimuthMARCS_(0),
           i32ElevationMARCS_(0),
           u32BitMask_(0),
           i16TxPowerdBmScaled_(0),
           u16ProfileId_(0),
           u8NumberOfFrequencies_(0),
           u8NumberOfTransmitters_(0)
        { } 
      } __attribute__((packed)) data_;


      // scale factor for tx power
      static const int DB_SCALE_FACTOR = 100;

      // option list limits
      const static ACE_UINT8 MAX_NUM_FREQUENCY_INFO_ENTRIES = 255; 
      const static ACE_UINT8 MAX_NUM_TRANSMITTER_ENTRIES    = 255; 

      // bit set mask
      static const ACE_UINT32 BANDWIDTH_BIT       = 0x01UL;
      static const ACE_UINT32 ANTENNAPROFILE_BIT  = 0x02UL;
      static const ACE_UINT32 AZIMUTH_BIT         = 0x04UL;
      static const ACE_UINT32 ELEVATION_BIT       = 0x08UL; 
      static const ACE_UINT32 TXPOWER_BIT         = 0x10UL;
      static const ACE_UINT32 FREQUENCY_BIT       = 0x20UL;
      static const ACE_UINT32 TRANSMITTER_BIT     = 0x40UL;
    
      void toNetworkByteOrder(Data & data);

      void toHostByteOrder(Data & data);

      EMANE::PHYTxFrequencyInfoManager frequencyInfoManager_;

      TransmitterInfoItems vec_;
   };
}

#include "universalphycontrolsendmessage.inl"

#endif //EMANE_UNIVERSALPHYCONTROLSENDMESSAGE_HEADER_
