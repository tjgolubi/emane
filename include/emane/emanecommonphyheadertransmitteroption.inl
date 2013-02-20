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

#include "emane/emanenet.h"

inline
EMANE::CommonPHYHeaderTransmitterOption::CommonPHYHeaderTransmitterOption() :
  CommonPHYHeaderOption(COMMON_PHY_OPTION)
{ }


inline
EMANE::CommonPHYHeaderTransmitterOption::CommonPHYHeaderTransmitterOption(const CommonPHYHeaderOptionObjectState & state)
    throw(CommonPHYHeaderException):
  CommonPHYHeaderOption(COMMON_PHY_OPTION)
{
  // buffer length sanity check
  if(state.length() % sizeof(EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo))
   {
      throw(CommonPHYHeaderException("CommonPHYHeaderTransmitterOption size mismatch"));
   }
  else
   {
     const size_t numEntries = state.length() / sizeof(EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo);

     // transmitter info
     EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo const * tp = 
              reinterpret_cast<EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo const *> (state.get());

     for(size_t idx = 0; idx < numEntries; ++idx, ++tp)
      {
        // add to container and convert to host byte order
        vec_.insert(vec_.end(), *tp)->toHostByteOrder();
      }
   }
}


inline 
EMANE::CommonPHYHeaderTransmitterOption::CommonPHYHeaderTransmitterOption(const EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfoItems & vec) :
  CommonPHYHeaderOption(COMMON_PHY_OPTION)
{
  vec_ = vec;
}


inline
EMANE::CommonPHYHeaderOptionObjectState EMANE::CommonPHYHeaderTransmitterOption::getObjectState() const
{
   // the flat buffer
   ACE_UINT8 buff[vec_.size() * sizeof(EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo)];
 
   // transmitter info
   EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo * tp = 
          reinterpret_cast<EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo *> (buff);

   // for each entry
   for(EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter, ++tp)
     {
       // copy to buffer
       *tp = *iter;

       // convert to net byte order
       tp->toNetworkByteOrder();
     }

  return CommonPHYHeaderOptionObjectState(COMMON_PHY_OPTION, buff, sizeof(buff));
}



inline
const EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfoItems & EMANE::CommonPHYHeaderTransmitterOption::getTransmitterInfo() const
{
  return vec_;
}

inline
std::string EMANE::CommonPHYHeaderTransmitterOption::format() const
{
   char buff[vec_.size() * 64];

   int len = 0;

   // for each entry
   for(EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter)
     {
       len += snprintf(buff + len, sizeof(buff) - len, "\ntransmitter %hu, tx pwr %5.4f dBm",
                       iter->getSrc(), iter->getTxPowerdBm());
     }

  return buff;
}


