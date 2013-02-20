/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSCIRCULARBUFFER_HEADER_
#define EMANEUTILSCIRCULARBUFFER_HEADER_

#include "emaneutils/spawnmemberfunc.h"

#include <vector>

#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>

namespace EMANEUtils
{
  /**
   * @class CircularBuffer
   *
   * @brief Generic thread safe circular buffer
   *
   * @note Uses a thread to allow blocking dequeues
   */
  template <typename T>
  class CircularBuffer
  {
  public:
    explicit CircularBuffer(ACE_UINT32 u32Size);
  
    ~CircularBuffer();
  
    /**
     * enqueue item @a T.
     *
     * @param item Item to enqueue
     */
    void enqueue(const T & item);

    /**
     * dequeue item.
     *
     * @return Item
     *
     * @note call will block if the queue is empty
     */
    T dequeue();
  
  private:
    std::vector<T> buffer_;

    ACE_Thread_Mutex mutex_;
    ACE_Condition<ACE_Thread_Mutex> cond_;

    unsigned long u32ReadIndex_;
    unsigned long u32WriteIndex_;

    ACE_THR_FUNC_RETURN process();
  };

  // implementation 
#include <ace/Guard_T.h>

  template <typename T>
  CircularBuffer<T>::CircularBuffer(ACE_UINT32 u32Size):
    buffer_(u32Size,T()),
    cond_(mutex_),
    u32ReadIndex_(0),
    u32WriteIndex_(0)
  {}
  
  template <typename T>
  CircularBuffer<T>::~CircularBuffer(){}

  template <typename T>
  void CircularBuffer<T>::enqueue(const T & item)
  {
    ACE_Guard<ACE_Thread_Mutex> m(mutex_);
    
    // store item in next write location
    buffer_[u32WriteIndex_] = item;

    // circularly increment write index
    ++u32WriteIndex_ %=  buffer_.size();

    // nudge read index along if it was overlapped
    if(u32WriteIndex_ == u32ReadIndex_)
      {
        ++u32ReadIndex_ %= buffer_.size();
      }

    // signal queued item availablity 
    cond_.signal();
  }

  template <typename T>
  T CircularBuffer<T>::dequeue()
  {
    ACE_Guard<ACE_Thread_Mutex> m(mutex_);

    // wait for an item to become available
    while(u32ReadIndex_ == u32WriteIndex_)
      {
        cond_.wait();
      }

    // get the next item
    T rtn = buffer_[u32ReadIndex_];

    // circularly increment write index
    ++u32ReadIndex_ %=  buffer_.size();
  
    return rtn;
  }
}

#endif //EMANEUTILSCIRCULARBUFFER_HEADER_
