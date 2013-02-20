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

#include <sstream>

inline 
void EMANE::PHYTxFrequencyInfoManager::setFrequencyInfo (const EMANE::PHYTxFrequencyInfoItems & items)
  {
    vec_ = items;
  }

inline 
const EMANE::PHYTxFrequencyInfoItems & EMANE::PHYTxFrequencyInfoManager::getFrequencyInfo () const
  {
    return vec_;
  }


inline 
size_t EMANE::PHYTxFrequencyInfoManager::numEntries() const
  { 
    return vec_.size();
  }

inline 
size_t EMANE::PHYTxFrequencyInfoManager::entrySize() const
  { 
    return sizeof(PHYTxFrequencyInfo);
  }

inline 
size_t EMANE::PHYTxFrequencyInfoManager::totalSize() const
  { 
    return vec_.size() * entrySize();
  }

inline 
void EMANE::PHYTxFrequencyInfoManager::clear()
  { 
    vec_.clear();
  }


inline 
size_t EMANE::PHYTxFrequencyInfoManager::copyToBuffer (void * buff, const size_t num) const 
  {
    size_t numBytesCopied = 0;

    EMANE::PHYTxFrequencyInfo * p = reinterpret_cast<EMANE::PHYTxFrequencyInfo *> (buff);

    // for each frequency info entry up to num
    for(size_t idx = 0; (idx < vec_.size()) && (idx < num); ++idx)
     {
       // copy to data buff
       ACE_OS::memcpy(p, &vec_[idx], entrySize());

       // convert to network byte order
       p->toNetworkByteOrder();

       // bump ptr
       ++p;

       // bump num bytes copied
       numBytesCopied += entrySize();
     }

    return numBytesCopied;
  }


inline 
size_t EMANE::PHYTxFrequencyInfoManager::copyFromBuffer (const void * buff, const size_t num)
  {
    size_t numBytesCopied = 0;

    const EMANE::PHYTxFrequencyInfo * p = reinterpret_cast<const EMANE::PHYTxFrequencyInfo *> (buff);

    // for up to num entries
    for(size_t idx = 0; idx < num; ++idx)
     {
       // local copy
       EMANE::PHYTxFrequencyInfo e;

       // copy from buff
       ACE_OS::memcpy(&e, p, entrySize());

       // convert to host byte order
       e.toHostByteOrder();

       // add to container
       vec_.push_back(e);

       // bump ptr
       ++p;

       // bump num bytes copied
       numBytesCopied += entrySize();
     }

    return numBytesCopied;
  }


inline
std::string EMANE::PHYTxFrequencyInfoManager::format() const
  {
    std::stringstream ss;

    for(EMANE::PHYTxFrequencyInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter)
     {
        ss << iter->format() << ", ";
     }

    ss << std::ends;

    return ss.str();
  }







inline 
void EMANE::PHYRxFrequencyInfoManager::setFrequencyInfo (const EMANE::PHYRxFrequencyInfoItems & items)
  {
    vec_ = items;
  }

inline 
const EMANE::PHYRxFrequencyInfoItems & EMANE::PHYRxFrequencyInfoManager::getFrequencyInfo () const
  {
    return vec_;
  }


inline 
size_t EMANE::PHYRxFrequencyInfoManager::numEntries() const
  { 
    return vec_.size();
  }

inline 
size_t EMANE::PHYRxFrequencyInfoManager::entrySize() const
  { 
    return sizeof(PHYRxFrequencyInfo);
  }

inline 
size_t EMANE::PHYRxFrequencyInfoManager::totalSize() const
  { 
    return vec_.size() * entrySize();
  }

inline 
void EMANE::PHYRxFrequencyInfoManager::clear()
  { 
    vec_.clear();
  }


inline 
size_t EMANE::PHYRxFrequencyInfoManager::copyToBuffer (void * buff, const size_t num) const 
  {
    size_t numBytesCopied = 0;

    EMANE::PHYRxFrequencyInfo * p = reinterpret_cast<EMANE::PHYRxFrequencyInfo *> (buff);

    // for each frequency info entry up to num
    for(size_t idx = 0; (idx < vec_.size()) && (idx < num); ++idx)
     {
       // copy to data buff
       ACE_OS::memcpy(p, &vec_[idx], entrySize());

       // convert to network byte order
       p->toNetworkByteOrder();

       // bump ptr
       ++p;

       // bump num bytes copied
       numBytesCopied += entrySize();
     }

    return numBytesCopied;
  }


inline 
size_t EMANE::PHYRxFrequencyInfoManager::copyFromBuffer (const void * buff, const size_t num)
  {
    size_t numBytesCopied = 0;

    const EMANE::PHYRxFrequencyInfo * p = reinterpret_cast<const EMANE::PHYRxFrequencyInfo *> (buff);

    // for up to num entries
    for(size_t idx = 0; idx < num; ++idx)
     {
       // local copy
       EMANE::PHYRxFrequencyInfo e;

       // copy from buff
       ACE_OS::memcpy(&e, p, entrySize());

       // convert to host byte order
       e.toHostByteOrder();

       // add to container
       vec_.push_back(e);

       // bump ptr
       ++p;

       // bump num bytes copied
       numBytesCopied += entrySize();
     }

    return numBytesCopied;
  }

inline
std::string EMANE::PHYRxFrequencyInfoManager::format() const
  {
    std::stringstream ss;

    for(EMANE::PHYRxFrequencyInfoItemsConstIter iter = vec_.begin(); iter != vec_.end(); ++iter)
     {
        ss << iter->format() << ", ";
     }

    ss << std::ends;

    return ss.str();
  }
