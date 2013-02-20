/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANE_COMMONPHYHEADER_HEADER_
#define EMANE_COMMONPHYHEADER_HEADER_

#include "emane/emanetypes.h"
#include "emane/emanenet.h"
#include "emane/emaneantennamode.h"
#include "emane/emaneupstreampacket.h"
#include "emane/emanedownstreampacket.h"
#include "emane/emanecommonphyheaderoption.h"
#include "emane/emanecommonphyfreqinfo.h"
#include "emane/emanecommonphyheaderoptionobjectstate.h"
#include "emane/emanecommonphyheaderexception.h"
#include "emaneutils/netutils.h"

#include <ace/Time_Value.h>
#include <ace/Basic_Types.h>

#include <string>
#include <list>
#include <vector>

namespace EMANE
{
  typedef std::list<CommonPHYHeaderOptionObjectState> CommonPHYHeaderOptionObjectStateList;


  /**
   * @class CommonPHYHeader
   *
   * @brief Common PHY Header used by all phy implemenations
   *
   */

  class CommonPHYHeader
   {
    public:
    /**
     *
     * phy header initializer.
     *
     * @param pkt upstream packet
     *
     * @exception throws  CommonPHYHeaderException
     */
    CommonPHYHeader(UpstreamPacket & pkt) 
      throw(CommonPHYHeaderException);
  
    /**
     *
     * phy header initializer.
     *
     * @param registrationId          phy registration id
     * @param fTxPowerdBm             tx power in dBm
     * @param u64BandWidthHz          the frequency bandwidt in Hz
     * @param fAntennaGaindBm         antenna gain in dBi
     * @param u8AntennaMode           antenna mode
     * @param tvTxTime                tx time stamp
     * @param u16Sequence             sequence number
     *
     */
    CommonPHYHeader(EMANE::RegistrationId    registrationId, 
                    float                    fTxPowerdBm,
                    ACE_UINT64               u64BandWidthHz,
                    float                    fAntennaGaindBm,
                    EMANE::ANTENNA_MODE      u8AntennaMode,
                    const ACE_Time_Value &   tvTxTime,
                    ACE_UINT16               u16Sequence);
   /**
    *
    * @return returns the RegistrationId
    *
    */
    EMANE::RegistrationId getRegistrationId() const;


   /**
    *
    * @return returns the tx power in dBm
    *
    */
    float getTxPowerdBm() const;


   /**
    *
    * @return returns the antenna gain of the sender in dbi
    *
    */
    float getAntennaGaindBi() const;


   /**
    *
    * @return returns the antenna mode
    *
    */
    EMANE::ANTENNA_MODE getAntennaMode() const;


   /**
    *
    * @return returns the absoulte tx time the message was sent
    *
    */
    ACE_Time_Value getTxTime() const;


   /**
    *
    * @return returns the duration of the message
    *
    */
    ACE_Time_Value getDuration() const;


   /**
    *
    * @return returns the first center frequency in Hz
    *
    */
    ACE_UINT64 getFirstFrequencyHz() const;


   /**
    *
    * @return returns the frequency bandwidth in Hz
    *
    */
    ACE_UINT64 getBandWidthHz() const;


   /**
    *
    * @return returns the sequence number from of the sending NEM
    *
    */
    ACE_UINT16 getSequenceNumber() const;


   /**
    *
    * @return returns true if header checksum is correct else false
    *
    */
    bool verifyCheckSum() const;

   /**
    *
    * @return returns true if header version is correct else returns false
    *
    */
    bool checkVersion() const;

   /**
    *
    * @return returns formatted string of the paramters and values for logging
    *
    */
    std::string format() const;

   /**
    *
    * @param tv the time at which this packet was received
    *
    * @return returns formatted string of the paramters and the delta between the tx and rx time
    *
    */
    std::string format(const ACE_Time_Value & tv) const;

   /**
    * prepend to pkt
    *
    * @param pkt pkt to prepend to
    *
    * @exception throws  CommonPHYHeaderException
    */
    void prependTo(DownstreamPacket & pkt) const
     throw(CommonPHYHeaderException);

   /**
    * add optional header
    *
    * @param option reference to optional header
    *
    */
    void addOption(const CommonPHYHeaderOption & option);


   /**
    * set frequency info vector
    *
    * @param vec vector of frequency info items
    *
    * @exception throws  CommonPHYHeaderException
    */
    void setFrequencyInfo (const EMANE::PHYTxFrequencyInfoItems & vec)
     throw(CommonPHYHeaderException);


   /**
    * get frequency info
    *
    * @return vec vector of frequency info items
    *
    */
    const EMANE::PHYTxFrequencyInfoItems & getFrequencyInfo () const
     throw(CommonPHYHeaderException);


   /**
    * get optional header
    *
    * @return a list of options
    *
    * @exception throws  CommonPHYHeaderException
    */
    CommonPHYHeaderOptionObjectStateList getOptionList() const;
    
  private:
    // the phy header version
    const static ACE_UINT16 COMMON_PHY_HEADER_VERSION = 0x0002;
    
    // scale factor for tx power and antenna gain
    const static float DB_SCALE_FACTOR = 100.0;

    // max number of frequency entries
    const static ACE_UINT8 MAX_NUM_FREQUENCY_INFO_ENTRIES = 255; 

    // max number of options
    const static ACE_UINT8 MAX_NUM_OPTIONS = 255; 

   /**
    * @struct Data
    *
    * @brief  the CommonPHYHeader private data
    *
    */
    struct Data 
    {
      ACE_UINT16            u16Version_;              // phy header version
      ACE_UINT16            u16CheckSum_;             // header checksum
      RegistrationId        u16RegistrationId_;       // phy id, see phy registration id's
      ACE_UINT16            u16SequenceNumber_;       // sequence number
      ACE_INT16             i16TxPowerdBmScaled_;     // scaled tx power in dBm
      ACE_UINT32            u32TxTimeSec_;            // pkt time in sec
      ACE_UINT32            u32TxTimeUsec_;           // pkt time in usec
      ACE_UINT64            u64BandWidthHz_;          // bandwidth in Hz
      ACE_INT16             i16AntennaGaindBiScaled_; // scaled antenna gain in dBi
      ACE_UINT8             u8AntennaMode_;           // antenna mode 
      ACE_UINT8             u8NumberOfFrequencies_;   // number of frequencies (1 minimum)
      ACE_UINT8             u8NumOptions_;            // number of options that follow
      PHYTxFrequencyInfo    frequencyInfo_[0];        // frequency info array

      Data () :
        u16Version_(0),
        u16CheckSum_(0),
        u16RegistrationId_(0),
        u16SequenceNumber_(0),
        i16TxPowerdBmScaled_(0),
        u32TxTimeSec_(0),
        u32TxTimeUsec_(0),
        u64BandWidthHz_(0),
        i16AntennaGaindBiScaled_(0),
        u8AntennaMode_(0),
        u8NumberOfFrequencies_(0),
        u8NumOptions_(0)
      { }
    } __attribute__((packed)) data_;

    CommonPHYHeaderOptionObjectStateList optionList_;

    EMANE::PHYTxFrequencyInfoManager frequencyInfoManager_;

    void toHostByteOrder(Data & data) const;
    
    void toNetworkByteOrder(Data & data) const;
  };
}

#include "emane/emanecommonphyheader.inl"

#endif //EMANE_COMMONPHYHEADER_HEADER_
