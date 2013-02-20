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

#ifndef EMANE_COMMONPHYHEADERTRANSMITTEROPTION_HEADER_
#define EMANE_COMMONPHYHEADERTRANSMITTEROPTION_HEADER_

#include <ace/Basic_Types.h>
#include "emane/emanecommonphyheaderoptions.h"

#include <string>


namespace EMANE
{
  /**
   * @class  CommonPHYHeaderTransmitterOption 
   *
   * @brief  optional header added to the CommonPHYHeader to indicate additional transmitters
   *
   */
  class CommonPHYHeaderTransmitterOption : public CommonPHYHeaderOption
  {
    public:
     const static ACE_UINT16 COMMON_PHY_OPTION = COMMON_PHY_HEADER_OPTION_TRANSMITTERS;

    /**
     * @class  TransmitterInfo
     *
     * @brief  info about additional transmitters
     *
     */
     class TransmitterInfo
      {
        public:
        /**
         *
         * @brief default constructor
         *
         */
         TransmitterInfo() :
           u16Src_(0),
           i16TxPowerdBmScaled_(0)
         { }

        /**
         *
         * @brief paramater constructor
         *
         * @param src the transmitting NEM id
         * @param fTxPowerdBm the transmit power in dBm
         *
         */

         TransmitterInfo(const ACE_UINT16 src, const float fTxPowerdBm) :
           u16Src_(src),
           i16TxPowerdBmScaled_(fTxPowerdBm * DB_SCALE_FACTOR)
         { }

        /**
         *
         * @return  returns the transmitter src id
         *
         */
         ACE_UINT16 getSrc() const 
          { 
            return u16Src_;
          }

        /**
         *
         * @return  returns the transmitter tx power in dBm
         *
         */
         float getTxPowerdBm() const 
          { 
            return static_cast<float> (i16TxPowerdBmScaled_) / DB_SCALE_FACTOR; 
          }

        /**
         *
         * @brief convert data from network to host byte order
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
         * @brief convert data from host to network byte order
         *
         */
         void toNetworkByteOrder()
          {
            // short
            u16Src_               = ACE_HTONS(u16Src_);
            i16TxPowerdBmScaled_  = ACE_HTONS(i16TxPowerdBmScaled_);
          }

         private:         
          // scale factor for tx power
          static const float DB_SCALE_FACTOR = 100.0;

          ACE_UINT16    u16Src_;                   // the NEM
          ACE_INT16     i16TxPowerdBmScaled_;      // tx power in dBm
      };

   // transmitter info container and container iterators
   typedef std::vector<TransmitterInfo>         TransmitterInfoItems;
   typedef TransmitterInfoItems::iterator       TransmitterInfoItemsIter;
   typedef TransmitterInfoItems::const_iterator TransmitterInfoItemsConstIter;

   /**
    * CommonPHYHeaderTransmitterOption initializer
    *
    */
    CommonPHYHeaderTransmitterOption();
 
   /**
    * CommonPHYHeaderTransmitterOption initializer
    *
    * @param  vec vector of transmitter info entries
    */
    CommonPHYHeaderTransmitterOption(const TransmitterInfoItems & vec);
   
 
   /**
    * CommonPHYHeaderTransmitterOption initializer
    *
    * @param  state CommonPHYHeaderOptionObjectState 
    *
    */
    CommonPHYHeaderTransmitterOption(const CommonPHYHeaderOptionObjectState & state)
     throw(CommonPHYHeaderException);
    
   /**
    *
    * @return return reference of the transmitter info vector
    *
    */
    const TransmitterInfoItems & getTransmitterInfo() const;
    
   /**
    *
    * @return returns the transmitter data object state
    *
    */
    CommonPHYHeaderOptionObjectState getObjectState() const;
  
   /**
    *
    * @return returns a formated string of parameters and values for logging
    *
    */
    std::string format() const;
   
  private:
    TransmitterInfoItems vec_;
  };
}
#include "emane/emanecommonphyheadertransmitteroption.inl"

#endif //EMANE_COMMONPHYHEADERTRANSMITTEROPTION_HEADER_
