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

#include "emaneutils/netutils.h"
#include <sstream>

namespace EMANE
{
  /**
   * @struct CommonPHYHeaderOptionHeader
   *
   * @brief The common phy header option header
   *
   */

  struct CommonPHYHeaderOptionHeader
  {
    ACE_UINT16 u16OptionType_;
    ACE_UINT16 u16OptionLength_;

    void toNetworkByteOrder()
    {
      u16OptionType_   = ACE_HTONS(u16OptionType_);
      u16OptionLength_ = ACE_HTONS(u16OptionLength_);
    }

    void toHostByteOrder()
    {
      u16OptionType_   = ACE_NTOHS(u16OptionType_);
      u16OptionLength_ = ACE_NTOHS(u16OptionLength_);
    }
  }__attribute__((packed));
}


inline
EMANE::CommonPHYHeader::CommonPHYHeader(UpstreamPacket & pkt)
  throw(CommonPHYHeaderException)
{
  // pkt length sanity check
  if(pkt.length() < sizeof(data_))
    {
      throw(CommonPHYHeaderException("Packet length too small to contain common PHY header"));
    }
  else
    {
      // copy header data block
      memcpy(&data_, pkt.get(), sizeof(data_));
  
      // strip header from pkt
      pkt.strip(sizeof(data_));

      // convert the header data fileds to host byte order
      toHostByteOrder(data_);

      // verify the packet chekcum
      if(verifyCheckSum() == false)
        {
          throw(CommonPHYHeaderException("Common PHY Header checksum invalid"));
        }
      // verify the packet version
      else if(checkVersion() == false)
        {
          throw(CommonPHYHeaderException("Common PHY Header version mismatch"));
        }
      else
        {
          // check frequency list is present
          if(data_.u8NumberOfFrequencies_ == 0)
           {
             throw(CommonPHYHeaderException("Common PHY Header must include at least 1 frequency info entry"));
           }

          // frequency list length sanity chack
          if(pkt.length() < (frequencyInfoManager_.entrySize() * data_.u8NumberOfFrequencies_))
           {
              throw(CommonPHYHeaderException("Packet length too small to contain frequency info data"));
           }
          else
           {
             // copy the frequency data block
             const size_t numBytesCopied = frequencyInfoManager_.copyFromBuffer(pkt.get(), data_.u8NumberOfFrequencies_);

             // strip from pkt
             pkt.strip(numBytesCopied);
           }

          // build the options list (if options present)
          for(ACE_UINT8 i = 0; i < data_.u8NumOptions_; ++i)
            {
              // pkt length sanity check
              if(pkt.length() < sizeof(CommonPHYHeaderOptionHeader))
                {
                  throw(CommonPHYHeaderException("Packet length too small to contain specified option header"));
                }
              else
                {
                  // option stroge
                  CommonPHYHeaderOptionHeader hdr;
                  
                  // copy option header data block from pkt
                  memcpy(&hdr, pkt.get(), sizeof(hdr));

                  // strip from pkt
                  pkt.strip(sizeof(hdr));
                  
                  // convert the header data fileds to host byte order
                  hdr.toHostByteOrder();
              
                  // option length sanity check
                  if(pkt.length() < hdr.u16OptionLength_)
                    {
                      throw(CommonPHYHeaderException("Packet length too small to contain specified option data"));
                    }
                  else
                    {
                      // add the option to the option state list
                      optionList_.push_back(CommonPHYHeaderOptionObjectState(hdr.u16OptionType_,
                                                                             pkt.get(),
                                                                             hdr.u16OptionLength_));
                      // strip option data from pkt
                      pkt.strip(hdr.u16OptionLength_);
                    }
                }
            }
        }
    }
}


inline
EMANE::CommonPHYHeader::CommonPHYHeader(EMANE::RegistrationId    registrationId, 
                                        float                    fTxPowerdBm, 
                                        ACE_UINT64               u64BandWidthHz,
                                        float                    fAntennaGaindBm, 
                                        EMANE::ANTENNA_MODE      u8AntennaMode, 
                                        const ACE_Time_Value &   tvTxTime,
                                        ACE_UINT16               u16Sequence)
{
  memset(&data_,0,sizeof(data_));
  data_.u16Version_ = COMMON_PHY_HEADER_VERSION;
  data_.u16CheckSum_ = 0;
  data_.u16RegistrationId_ = registrationId;
  data_.i16TxPowerdBmScaled_ = fTxPowerdBm * DB_SCALE_FACTOR;
  data_.u64BandWidthHz_ = u64BandWidthHz;
  data_.i16AntennaGaindBiScaled_ = fAntennaGaindBm * DB_SCALE_FACTOR;
  data_.u8AntennaMode_ = u8AntennaMode;
  data_.u32TxTimeSec_ = tvTxTime.sec();
  data_.u32TxTimeUsec_ = tvTxTime.usec();
  data_.u16SequenceNumber_ = u16Sequence;
}

inline
EMANE::RegistrationId EMANE::CommonPHYHeader::getRegistrationId() const
{
  return  data_.u16RegistrationId_;
}


inline
float  EMANE::CommonPHYHeader::getTxPowerdBm() const
{
  return data_.i16TxPowerdBmScaled_ / float(DB_SCALE_FACTOR);
}


inline
float EMANE::CommonPHYHeader::getAntennaGaindBi() const
{
  return  data_.i16AntennaGaindBiScaled_ / float(DB_SCALE_FACTOR);
}


inline    
EMANE::ANTENNA_MODE EMANE::CommonPHYHeader::getAntennaMode() const
{
  return data_.u8AntennaMode_;
}


inline
ACE_Time_Value EMANE::CommonPHYHeader::getTxTime() const
{
  return ACE_Time_Value(data_.u32TxTimeSec_, data_.u32TxTimeUsec_);
}


inline
ACE_Time_Value EMANE::CommonPHYHeader::getDuration() const
{
  const EMANE::PHYTxFrequencyInfoItems & vec = frequencyInfoManager_.getFrequencyInfo();

  ACE_Time_Value tvBegin = ACE_Time_Value::zero;
  ACE_Time_Value tvEnd   = ACE_Time_Value::zero;

  for(PHYTxFrequencyInfoItemsConstIter iter = vec.begin(); iter != vec.end(); ++iter)
   {
     // the offset
     const ACE_Time_Value tv1 = iter->getTxOffset();

     // the duration
     const ACE_Time_Value tv2 = iter->getDuration();

     // duration position
     const ACE_Time_Value tv3 = tv1 + tv2;

     // get the begin time
     if(tv1 < tvBegin)
      {
        tvBegin = tv1;
      }

     // get the end time
     if(tv3 > tvEnd)
      {
        tvEnd = tv3;
      }
   }

  // the total duration with gaps/overlap
  return (tvEnd - tvBegin);
}


inline
ACE_UINT64 EMANE::CommonPHYHeader::getFirstFrequencyHz() const
{
  const EMANE::PHYTxFrequencyInfoItems & vec = frequencyInfoManager_.getFrequencyInfo();

  if(vec.size() > 0)
   {
     // return first freq
     return vec[0].getCenterFrequencyHz();
   }
  else
   {
     return 0;
   }
}


inline
ACE_UINT64 EMANE::CommonPHYHeader::getBandWidthHz() const
{
  return data_.u64BandWidthHz_;
}


inline
ACE_UINT16 EMANE::CommonPHYHeader::getSequenceNumber() const
{
  return data_.u16SequenceNumber_;
}


inline
bool EMANE::CommonPHYHeader::verifyCheckSum() const
{
  // do checksum over the fixed size data section
  // TODO maybe run checksum over entire message
  return EMANEUtils::inet_cksum(&data_, sizeof(data_)) == 0xFFFF;
}


inline
bool EMANE::CommonPHYHeader::checkVersion() const
{
  return data_.u16Version_ == COMMON_PHY_HEADER_VERSION;
}


inline
std::string EMANE::CommonPHYHeader::format() const
{
  char fmtBuff[4096];
  
  snprintf(fmtBuff, sizeof(fmtBuff), "phyhdr: regid %hu, seq %hu, txpwr %3.2lf dBm, bandwidth %s, antenna gain %3.2lf dBi, mode %s \n\t%s",
           getRegistrationId(),
           getSequenceNumber(),
           getTxPowerdBm(), 
           EMANEUtils::formatFrequency(getBandWidthHz()).c_str(),
           getAntennaGaindBi(),
           EMANE::antennaModeToString(getAntennaMode()),
           frequencyInfoManager_.format().c_str());

  return  fmtBuff;
}


inline
std::string EMANE::CommonPHYHeader::format(const ACE_Time_Value & tv) const
{
  const ACE_Time_Value tvOTA = tv - getTxTime();
  
  char fmtBuff[256];
  
  snprintf(fmtBuff, sizeof(fmtBuff), ", otatime %ld:%06ld", tvOTA.sec(), tvOTA.usec());
  
  return format() + fmtBuff;
}


inline
void EMANE::CommonPHYHeader::toNetworkByteOrder(Data & data) const
{
  // short
  data.u16Version_              = ACE_HTONS(data.u16Version_);
  data.u16CheckSum_             = ACE_HTONS(data.u16CheckSum_);
  data.u16RegistrationId_       = ACE_HTONS(data.u16RegistrationId_);
  data.i16TxPowerdBmScaled_     = ACE_HTONS(data.i16TxPowerdBmScaled_);
  data.i16AntennaGaindBiScaled_ = ACE_HTONS(data.i16AntennaGaindBiScaled_);
  data.u16SequenceNumber_       = ACE_HTONS(data.u16SequenceNumber_);
  
  // long
  data.u32TxTimeSec_            = ACE_HTONL(data.u32TxTimeSec_);
  data.u32TxTimeUsec_           = ACE_HTONL(data.u32TxTimeUsec_);

  // long long
  data.u64BandWidthHz_          = EMANE::HTONLL(data.u64BandWidthHz_);
}


inline
void EMANE::CommonPHYHeader::toHostByteOrder(Data & data) const
{
  // short
  data.u16Version_              = ACE_NTOHS(data.u16Version_);
  data.u16CheckSum_             = ACE_NTOHS(data.u16CheckSum_);
  data.u16RegistrationId_       = ACE_NTOHS(data.u16RegistrationId_);
  data.i16TxPowerdBmScaled_     = ACE_NTOHS(data.i16TxPowerdBmScaled_);
  data.i16AntennaGaindBiScaled_ = ACE_NTOHS(data.i16AntennaGaindBiScaled_);
  data.u16SequenceNumber_       = ACE_NTOHS(data.u16SequenceNumber_);
  
  // long
  data.u32TxTimeSec_            = ACE_NTOHL(data.u32TxTimeSec_);
  data.u32TxTimeUsec_           = ACE_NTOHL(data.u32TxTimeUsec_);

  // long long
  data.u64BandWidthHz_          = EMANE::NTOHLL(data.u64BandWidthHz_);
}


inline
void EMANE::CommonPHYHeader::prependTo(DownstreamPacket & pkt) const
  throw(CommonPHYHeaderException)
{
  // get number of frequency info entries
  const size_t numFreqEntries = frequencyInfoManager_.numEntries();

  // check for frequency info
  if(numFreqEntries == 0)
   {
     throw(CommonPHYHeaderException("Common PHY Header must include at least 1 frequency info entry"));
   }

  // add any optional headers go first and wind up at the end of the buffer 
  // when done since data is pre-pended as we build the outbound pkt
  for(CommonPHYHeaderOptionObjectStateList::const_iterator iter = optionList_.begin(); iter != optionList_.end(); ++iter)
    {
      // prepend the option data first 
      pkt.prepend(iter->get(),iter->length());

      // now the header
      CommonPHYHeaderOptionHeader hdr;

      // set type
      hdr.u16OptionType_   = iter->getType();
 
      // set length
      hdr.u16OptionLength_ = iter->length();

      // convert to net byte order
      hdr.toNetworkByteOrder();
      
      // then prepend the option header that defines the data
      pkt.prepend(&hdr, sizeof(hdr));
    }

  // outgoing data size (phy header + freq info list size)
  const ACE_UINT16 totalLen = sizeof(data_) + frequencyInfoManager_.totalSize();

  // daa bufa
  ACE_UINT8 headerBuff[totalLen];

  // data ptr
  Data * p = reinterpret_cast<Data *> (headerBuff);

  // copy phy fixed size data
  ACE_OS::memcpy(p, &data_, sizeof(data_));

  // set the number of options that were pre-pended above
  p->u8NumOptions_ = optionList_.size();

  // set number of frequency info entries
  p->u8NumberOfFrequencies_ = numFreqEntries;

  // copy the freq info entries to the buffer
  frequencyInfoManager_.copyToBuffer(p->frequencyInfo_, numFreqEntries);

  // set checksum to 0
  p->u16CheckSum_ = 0;

  // set checksum over the fixed size data section for now
  // TODO maybe run checksum over entire pkt
  p->u16CheckSum_ = ~EMANEUtils::inet_cksum(p, sizeof(Data));

  // convert the header to network byte order
  toNetworkByteOrder(*p);

  // last prepend the header to the packet so the pkt looks like this:
  // [phy header | frequency info list | options list]
  pkt.prepend(headerBuff, totalLen);
}


inline 
EMANE::CommonPHYHeaderOptionObjectStateList EMANE::CommonPHYHeader::getOptionList() const
{
  return optionList_;
}


inline
void EMANE::CommonPHYHeader::addOption(const CommonPHYHeaderOption & option)
{
  if(optionList_.size() < CommonPHYHeader::MAX_NUM_OPTIONS)
   {
      optionList_.push_back(option.getObjectState());
   }
  else
   {
      std::stringstream ss;
      ss << "Attempt to add more than " 
         <<  CommonPHYHeader::MAX_NUM_OPTIONS  
         << " CommonPHYHeader options" 
         << std::ends;

      throw(CommonPHYHeaderException(ss.str()));
   }
}



inline 
void EMANE::CommonPHYHeader::setFrequencyInfo (const EMANE::PHYTxFrequencyInfoItems & vec)
  throw(CommonPHYHeaderException)
{
   frequencyInfoManager_.setFrequencyInfo(vec);
}


inline 
const EMANE::PHYTxFrequencyInfoItems & EMANE::CommonPHYHeader::getFrequencyInfo () const
  throw(CommonPHYHeaderException)
{
  if(frequencyInfoManager_.numEntries() == 0)
   {
      std::stringstream ss;
      ss << "Frequency info list is empty" << std::ends;

      throw(CommonPHYHeaderException(ss.str()));
   }

  return frequencyInfoManager_.getFrequencyInfo();
}




