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

#include <sstream>

inline EMANE::UniversalPhyControlRecvMessage::UniversalPhyControlRecvMessage()
{ }


inline EMANE::UniversalPhyControlRecvMessage::UniversalPhyControlRecvMessage(const EMANE::ControlMessage & msg)
       throw(UniversalPHYControlRecvMessageException)
{
  unpackControlMessage(msg);
}


inline ACE_UINT64 EMANE::UniversalPhyControlRecvMessage::getBandWidthHz() const 
   { 
     return data_.u64BandWidthHz_; 
   }

inline void EMANE::UniversalPhyControlRecvMessage::setBandWidthHz(ACE_UINT64 u64BandWidthHz)
   {
     data_.u64BandWidthHz_ = u64BandWidthHz;
   }


inline ACE_Time_Value EMANE::UniversalPhyControlRecvMessage::getTxTime() const 
   { 
     return ACE_Time_Value(data_.u32TxTimeSec_, data_.u32TxTimeUsec_); 
   }

inline void EMANE::UniversalPhyControlRecvMessage::setTxTime(const ACE_Time_Value & tv)
   {
     data_.u32TxTimeSec_  = tv.sec();
     data_.u32TxTimeUsec_ = tv.usec();
   }



inline  
void EMANE::UniversalPhyControlRecvMessage::setFrequencyInfo (const EMANE::PHYRxFrequencyInfoItems & vec)
     throw(UniversalPHYControlRecvMessageException)
 {
   if(vec.size() > EMANE::UniversalPhyControlRecvMessage::MAX_NUM_FREQUENCY_INFO_ENTRIES)
    {
      std::stringstream ss;
      ss << "Attempt to set more than "
         <<  EMANE::UniversalPhyControlRecvMessage::MAX_NUM_FREQUENCY_INFO_ENTRIES
         << " UniversalPHYControlRecvMessage frequency info entries"
         << std::ends;

      throw(UniversalPHYControlRecvMessageException(ss.str()));
    }
   else
    {
      frequencyInfoManager_.setFrequencyInfo(vec);
    }
 }



inline  
const EMANE::PHYRxFrequencyInfoItems & EMANE::UniversalPhyControlRecvMessage::getFrequencyInfo () const
 {
   return frequencyInfoManager_.getFrequencyInfo();
 }



inline  
void EMANE::UniversalPhyControlRecvMessage::setTransmitterInfo (const EMANE::UniversalPhyControlRecvMessage::TransmitterInfoItems & vec)
     throw(UniversalPHYControlRecvMessageException)
 {
   if(vec.size() > EMANE::UniversalPhyControlRecvMessage::MAX_NUM_TRANSMITTER_ENTRIES)
    {
      std::stringstream ss;
      ss << "Attempt to set more than "
         <<  EMANE::UniversalPhyControlRecvMessage::MAX_NUM_TRANSMITTER_ENTRIES
         << " UniversalPHYControlRecvMessage transmitter info entries"
         << std::ends;

      throw(UniversalPHYControlRecvMessageException(ss.str()));
    }
   else
    {
      vec_ = vec;
    }
 }



inline  
const EMANE::UniversalPhyControlRecvMessage::TransmitterInfoItems & EMANE::UniversalPhyControlRecvMessage::getTransmitterInfo () const
 {
   return vec_;
 }




inline  
EMANE::ControlMessage EMANE::UniversalPhyControlRecvMessage::buildControlMessage() 
 {
   // outgoing data size (data + freq info list + vec) in bytes
   const ACE_UINT16 totalLen = sizeof(data_) + 
                               frequencyInfoManager_.totalSize() + 
                               (vec_.size() * sizeof(EMANE::UniversalPhyControlRecvMessage::TransmitterInfo));

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

   // set number of transmitters
   dataPtr->u8NumberOfTransmitters_ = vec_.size();

   // transmitter info
   EMANE::UniversalPhyControlRecvMessage::TransmitterInfo * tp = 
          reinterpret_cast<EMANE::UniversalPhyControlRecvMessage::TransmitterInfo *> (p);

   // for each entry
   for(EMANE::UniversalPhyControlRecvMessage::TransmitterInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter, ++tp)
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
void EMANE::UniversalPhyControlRecvMessage::unpackControlMessage(const EMANE::ControlMessage & msg)
 throw(UniversalPHYControlRecvMessageException)
 {
  if ((msg.getMajorIdentifier() != MAJOR_ID) || (msg.getMinorIdentifier() != MINOR_ID))
   {
     throw(UniversalPHYControlRecvMessageException("Invalid UniversalPhyControlRecvMessage major/minor id"));
   }
  else
   {
     ACE_UINT8 const * p = reinterpret_cast<ACE_UINT8 const *> (msg.get());

     size_t len = msg.length();

     if(len < sizeof(data_))
       {
         throw(UniversalPHYControlRecvMessageException("Message length too small to contain header"));
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

         // get the total transmitter list next
         if(len < (data_.u8NumberOfTransmitters_ * sizeof(EMANE::UniversalPhyControlRecvMessage::TransmitterInfo)))
          {
            throw(CommonPHYHeaderException("Message length too small to contain transmitter list"));
          }
         else
          {
            // transmitter info
            EMANE::UniversalPhyControlRecvMessage::TransmitterInfo const * tp = 
              reinterpret_cast<EMANE::UniversalPhyControlRecvMessage::TransmitterInfo const *> (p);

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
void EMANE::UniversalPhyControlRecvMessage::toNetworkByteOrder(EMANE::UniversalPhyControlRecvMessage::Data & data)
 {
   // long
   data.u32TxTimeSec_  = ACE_HTONL(data.u32TxTimeSec_);
   data.u32TxTimeUsec_ = ACE_HTONL(data.u32TxTimeUsec_);

   // long long
   data.u64BandWidthHz_ = EMANE::HTONLL(data.u64BandWidthHz_);
 }


inline  
void EMANE::UniversalPhyControlRecvMessage::toHostByteOrder(EMANE::UniversalPhyControlRecvMessage::Data & data)
 {
   // long
   data.u32TxTimeSec_  = ACE_NTOHL(data.u32TxTimeSec_);
   data.u32TxTimeUsec_ = ACE_NTOHL(data.u32TxTimeUsec_);

   // long long
   data.u64BandWidthHz_ = EMANE::NTOHLL(data.u64BandWidthHz_);
 }


inline  
std::string EMANE::UniversalPhyControlRecvMessage::format(const ACE_Time_Value & tv1) const
 {
   char fmtBuff[4096];

   const ACE_Time_Value tv2 = tv1 - getTxTime();

   int len = snprintf(fmtBuff, sizeof(fmtBuff), "rxctrl: bandwidth %s, otatime %ld:%06ld, %s", 
            EMANEUtils::formatFrequency(getBandWidthHz()).c_str(), 
            tv2.sec(),
            tv2.usec(),
            frequencyInfoManager_.format().c_str());

   for(EMANE::UniversalPhyControlRecvMessage::TransmitterInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter)
    {
      len += snprintf(fmtBuff + len, sizeof(fmtBuff) - len, "\n%s", iter->format().c_str());
    }

   return fmtBuff;
 }


