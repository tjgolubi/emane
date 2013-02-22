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

#include "emaneutils/netutils.h"
#include "emane/emaneconstants.h"

#include <sstream>

inline EMANE::UniversalPhyControlSendMessage::UniversalPhyControlSendMessage()
 { }


inline EMANE::UniversalPhyControlSendMessage::UniversalPhyControlSendMessage(const EMANE::ControlMessage & msg)
  throw(UniversalPHYControlSendMessageException)
 {
   unpackControlMessage(msg);
 }


inline bool EMANE::UniversalPhyControlSendMessage::isBitSet(const ACE_UINT32 bit) const
   { 
     return ((data_.u32BitMask_ & bit) == bit);
   }

inline void EMANE::UniversalPhyControlSendMessage::setBit(const ACE_UINT32 bit)
   { 
     data_.u32BitMask_ |=  bit;
   }

inline ACE_UINT64 EMANE::UniversalPhyControlSendMessage::getBandWidthHz() const 
   { 
     return data_.u64BandWidthHz_; 
   }

inline void EMANE::UniversalPhyControlSendMessage::setBandWidthHz(ACE_UINT64 u64BandWidthHz)
   {
     data_.u64BandWidthHz_ = u64BandWidthHz;

     setBit(BANDWIDTH_BIT);
   }


inline ACE_UINT16 EMANE::UniversalPhyControlSendMessage::getAntennaProfileId() const 
   { 
     return data_.u16ProfileId_; 
   }

inline void EMANE::UniversalPhyControlSendMessage::setAntennaProfileId(const ACE_UINT16 u16ProfileId)
   {
     data_.u16ProfileId_ = u16ProfileId;

     setBit(ANTENNAPROFILE_BIT);
   }


inline float EMANE::UniversalPhyControlSendMessage::getAntennaAzimuthDegrees() const 
   { 
     return static_cast<float> (data_.i32AzimuthMARCS_) / EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 
   }

inline void EMANE::UniversalPhyControlSendMessage::setAntennaAzimuthDegrees(float fAntennaAzimuthDegrees)
   { 
      data_.i32AzimuthMARCS_ = fAntennaAzimuthDegrees * EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 

      setBit(AZIMUTH_BIT);
   }


inline float EMANE::UniversalPhyControlSendMessage::getAntennaElevationDegrees() const 
   { 
     return static_cast<float> (data_.i32ElevationMARCS_) / EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 
   }

inline void EMANE::UniversalPhyControlSendMessage::setAntennaElevationDegrees(float fAntennaElevationDegrees)
   { 
     data_.i32ElevationMARCS_ = fAntennaElevationDegrees * EMANE::MILLI_ARC_SECONDS_PER_DEGREE; 

     setBit(ELEVATION_BIT);
   }



inline float EMANE::UniversalPhyControlSendMessage::getTxPowerdBm() const 
   { 
     return static_cast<float> (data_.i16TxPowerdBmScaled_) / DB_SCALE_FACTOR; 
   }

inline void EMANE::UniversalPhyControlSendMessage::setTxPowerdBm(float dTxPowerdBm) 
   { 
     data_.i16TxPowerdBmScaled_ = dTxPowerdBm * float(DB_SCALE_FACTOR);

     setBit(TXPOWER_BIT);
   }


inline  
void EMANE::UniversalPhyControlSendMessage::setFrequencyInfo (const EMANE::PHYTxFrequencyInfoItems & items)
     throw(UniversalPHYControlSendMessageException)
 {
   if(items.size() > EMANE::UniversalPhyControlSendMessage::MAX_NUM_FREQUENCY_INFO_ENTRIES)
    {
      std::stringstream ss;
      ss << "Attempt to set more than "
         <<  EMANE::UniversalPhyControlSendMessage::MAX_NUM_FREQUENCY_INFO_ENTRIES
         << " UniversalPHYControlSendMessage frequency info entries"
         << std::ends;

      throw(UniversalPHYControlSendMessageException(ss.str()));
    }
   else
    {
      frequencyInfoManager_.setFrequencyInfo(items);

      setBit(FREQUENCY_BIT);
    }
 }


inline  
const EMANE::PHYTxFrequencyInfoItems & EMANE::UniversalPhyControlSendMessage::getFrequencyInfo () const
 {
   return frequencyInfoManager_.getFrequencyInfo();
 }



inline  
void EMANE::UniversalPhyControlSendMessage::setAdditionalTransmitters (const EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & items)
     throw(UniversalPHYControlSendMessageException)
 {
   if(items.size() > EMANE::UniversalPhyControlSendMessage::MAX_NUM_TRANSMITTER_ENTRIES)
    {
      std::stringstream ss;
      ss << "Attempt to set more than "
         <<  EMANE::UniversalPhyControlSendMessage::MAX_NUM_TRANSMITTER_ENTRIES
         << " UniversalPHYControlSendMessage transmitter info entries"
         << std::ends;

      throw(UniversalPHYControlSendMessageException(ss.str()));
    }
   else
    {
      setBit(TRANSMITTER_BIT);

      vec_ = items;
    }
 }



inline  
const EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & EMANE::UniversalPhyControlSendMessage::getAdditionalTransmitters () const
 {
   return vec_;
 }



inline  
int EMANE::UniversalPhyControlSendMessage::overWrite(const EMANE::UniversalPhyControlSendMessage & msg) 
 {
   int num;

   if(msg.isBitSet(BANDWIDTH_BIT))
    {
      setBandWidthHz(msg.getBandWidthHz());
      ++num;
    }

   if(msg.isBitSet(ANTENNAPROFILE_BIT))
    {
      setAntennaProfileId(msg.getAntennaProfileId());
      ++num;
    }

   if(msg.isBitSet(ELEVATION_BIT))
    {
      setAntennaElevationDegrees(msg.getAntennaElevationDegrees());
      ++num;
    }

   if(msg.isBitSet(AZIMUTH_BIT))
    {
      setAntennaAzimuthDegrees(msg.getAntennaAzimuthDegrees());
      ++num;
    }

   if(msg.isBitSet(TXPOWER_BIT))
    {
      setTxPowerdBm(msg.getTxPowerdBm());
      ++num;
    }

   if(msg.isBitSet(FREQUENCY_BIT))
    {
      setFrequencyInfo(msg.getFrequencyInfo());
      ++num;
    }

   if(msg.isBitSet(TRANSMITTER_BIT))
    {
      setAdditionalTransmitters(msg.getAdditionalTransmitters());
      ++num;
    }

   return num;
 }



inline  
EMANE::ControlMessage EMANE::UniversalPhyControlSendMessage::buildControlMessage() 
 {
   // outgoing data size (data + freq info list + atv) in bytes
   const ACE_UINT16 totalLen = sizeof(data_) + 
                               frequencyInfoManager_.totalSize() + 
                               (vec_.size() * sizeof(EMANE::UniversalPhyControlSendMessage::TransmitterInfo));

   // the flat buffer
   ACE_UINT8 buff[totalLen];
 
   ACE_UINT8 * p = buff;

   // copy fixed data first
   ACE_OS::memcpy(p, &data_, sizeof(data_));

   // bump
   p += sizeof(data_);

   // data ptr
   Data * const dataPtr = reinterpret_cast<Data *const > (buff);

   // set number of frequency info entries
   dataPtr->u8NumberOfFrequencies_ = frequencyInfoManager_.numEntries();

   // copy the freq entries to the buffer after the fixed data and get the end position
   p += frequencyInfoManager_.copyToBuffer(p, frequencyInfoManager_.numEntries());

   // set number of additional transmitters
   dataPtr->u8NumberOfTransmitters_ = vec_.size();

   // transmitter info
   EMANE::UniversalPhyControlSendMessage::TransmitterInfo * tp = 
          reinterpret_cast<EMANE::UniversalPhyControlSendMessage::TransmitterInfo *> (p);

   // for each entry
   for(EMANE::UniversalPhyControlSendMessage::TransmitterInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter, ++tp)
     {
       // copy to buffer
       *tp = *iter;

       // convert to net byte order
       tp->toNetworkByteOrder();
     }

   // convert the header to network byte order
   toNetworkByteOrder(*dataPtr);

   return EMANE::ControlMessage(MAJOR_ID, MINOR_ID, buff, totalLen);
 }


inline  
void EMANE::UniversalPhyControlSendMessage::unpackControlMessage(const EMANE::ControlMessage & msg)
 throw(UniversalPHYControlSendMessageException)
 {
  if ((msg.getMajorIdentifier() != MAJOR_ID) || (msg.getMinorIdentifier() != MINOR_ID))
   {
     throw(UniversalPHYControlSendMessageException("Invalid UniversalPhyControlSendMessage major/minor id"));
   }
  else
   {
     ACE_UINT8 const * p = reinterpret_cast<ACE_UINT8 const *> (msg.get());

     size_t len = msg.length();

     if(len < sizeof(data_))
       {
         throw(UniversalPHYControlSendMessageException("Message length too small to contain header"));
       }
     else
       {
         // copy header
         memcpy(&data_, p, sizeof(data_));

         // bump
         p += sizeof(data_);

         len -= sizeof(data_);

         // convert the header to host byte order
         toHostByteOrder(data_);

         // get the frequency info first
         if(len < (data_.u8NumberOfFrequencies_ * frequencyInfoManager_.entrySize()))
          {
            throw(CommonPHYHeaderException("Message length too small to contain frequency info data"));
          }
         else
          {
            // copy the data
            const size_t numBytesCopied = frequencyInfoManager_.copyFromBuffer(p, data_.u8NumberOfFrequencies_);

            // bump
            p += numBytesCopied;

            len -= numBytesCopied;
          }

         // get the additional transmitter list next
         if(len < (data_.u8NumberOfTransmitters_ * sizeof(ACE_UINT16)))
          {
            throw(CommonPHYHeaderException("Message length too small to contain transmitter list"));
          }
         else
          {
            // transmitter info
            EMANE::UniversalPhyControlSendMessage::TransmitterInfo const * tp = 
              reinterpret_cast<EMANE::UniversalPhyControlSendMessage::TransmitterInfo const *> (p);

            for(size_t idx = 0; idx < data_.u8NumberOfTransmitters_; ++idx, ++tp)
             {
                // add to container and convert to host byte order
                vec_.insert(vec_.end(), *tp)->toHostByteOrder();
             }
          }
       }
    }
 }




inline  
void EMANE::UniversalPhyControlSendMessage::toNetworkByteOrder(EMANE::UniversalPhyControlSendMessage::Data & data)
 {
   // short
   data.i16TxPowerdBmScaled_  = ACE_HTONS(data.i16TxPowerdBmScaled_);
   data.u16ProfileId_         = ACE_HTONS(data.u16ProfileId_);

   // long
   data.i32AzimuthMARCS_      = ACE_HTONL(data.i32AzimuthMARCS_);
   data.i32ElevationMARCS_    = ACE_HTONL(data.i32ElevationMARCS_);
   data.u32BitMask_           = ACE_HTONL(data.u32BitMask_);

   // long long
   data.u64BandWidthHz_       = EMANE::HTONLL(data.u64BandWidthHz_);
 }


inline  
void EMANE::UniversalPhyControlSendMessage::toHostByteOrder(EMANE::UniversalPhyControlSendMessage::Data & data)
 {
   // short
   data.i16TxPowerdBmScaled_  = ACE_NTOHS(data.i16TxPowerdBmScaled_);
   data.u16ProfileId_         = ACE_NTOHS(data.u16ProfileId_);

   // long
   data.i32AzimuthMARCS_      = ACE_NTOHL(data.i32AzimuthMARCS_);
   data.i32ElevationMARCS_    = ACE_NTOHL(data.i32ElevationMARCS_);
   data.u32BitMask_           = ACE_NTOHL(data.u32BitMask_);

   // long long
   data.u64BandWidthHz_       = EMANE::NTOHLL(data.u64BandWidthHz_);
 }


inline  
std::string EMANE::UniversalPhyControlSendMessage::format() const
 {
   char fmtBuff[4096];

   int len = snprintf(fmtBuff, sizeof(fmtBuff), "txctrl: ");

   if(isBitSet(BANDWIDTH_BIT)) 
    {
      len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "bandwidth %s, ", EMANEUtils::formatFrequency(getBandWidthHz()).c_str());
    }

   if(isBitSet(ANTENNAPROFILE_BIT)) 
    {
      len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "antenna profile %hu, ", getAntennaProfileId());
    }

   if(isBitSet(AZIMUTH_BIT)) 
    {
      len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "antenna az %5.4f deg, ", getAntennaAzimuthDegrees());
    }

   if(isBitSet(ELEVATION_BIT)) 
    {
      len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "antenna el %5.4f deg, ", getAntennaElevationDegrees());
    }

   if(isBitSet(TXPOWER_BIT)) 
    {
      len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "tx pwr %5.4f dBm, ", getTxPowerdBm());
    }

   if(isBitSet(FREQUENCY_BIT)) 
    {
      len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "%s ", frequencyInfoManager_.format().c_str());
    }

   if(isBitSet(TRANSMITTER_BIT)) 
    {
      for(EMANE::UniversalPhyControlSendMessage::TransmitterInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter)
       {
         if(len < 64)
           {
             len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "\n more ...");
             break;
           }
         else
           {
             len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "\n%s", iter->format().c_str());
           }
       }
    }

   return fmtBuff;
 }


