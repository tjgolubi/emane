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

#ifndef EMANEUTILSPRODUCERCONSUMERBUFFER_HEADER_
#define EMANEUTILSPRODUCERCONSUMERBUFFER_HEADER_

#include "circularbuffer.h"
#include <ace/Semaphore.h>

namespace EMANEUtils
{
  /**
   * @class ProducerConsumerBuffer
   *
   * @brief Generic producer consumer  
   *
   * @details Generic thread safe implmentation of the the producer consumer
   * design pattern
   *
   */
  template <typename T>
  class ProducerConsumerBuffer
  {
  public:
    /**
     * Constructor
     *
     * @param u32Size Queue size
     */
    explicit ProducerConsumerBuffer(ACE_UINT32 u32Size);
  
    ~ProducerConsumerBuffer();
  
    /**
     * Produce item T.  Will block and wait if queue is full.
     *
     * @param item produced item
     */
    void produce(const T & item);
  
    /**
     * Consume an item T.  Will block and wait if the queue is empty.
     *
     * @return T Item to consume
     */
    T consume();

  private:
    CircularBuffer<T> buffer_;
    ACE_Semaphore readerSem_;
    ACE_Semaphore writerSem_;
  };


  template <typename T>
    ProducerConsumerBuffer<T>::ProducerConsumerBuffer(ACE_UINT32 u32Size):
    buffer_(u32Size + 1),
    readerSem_(u32Size),
    writerSem_(0){}

  template <typename T>
    ProducerConsumerBuffer<T>::~ProducerConsumerBuffer(){}

  template <typename T>
    T ProducerConsumerBuffer<T>::consume()
  {
    // wait for an item to consume
    writerSem_.acquire();

    T rtn = buffer_.dequeue();
  
    // signal the availablity to consume another item
    readerSem_.release();

    return rtn;
  }

  template <typename T>
    void ProducerConsumerBuffer<T>::produce(const T & item)
  {
    // wait for a location to produce an item
    readerSem_.acquire();
  
    buffer_.enqueue(item);
  
    // signal the production of another item
    writerSem_.release();
  }
}

#endif //EMANEUTILSPRODUCERCONSUMERBUFFER_HEADER_
