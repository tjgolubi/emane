/*
 * Copyright (c) 2008-2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEDOWNSTREAMPACKET_HEADER_
#define EMANEDOWNSTREAMPACKET_HEADER_

#include "emane/emanepacket.h"

#include <list>
#include <vector>

namespace EMANE
{
  /**
   * @class DownstreamPacket
   *
   * @brief Specialized packet the allows downstream
   * processing to add layer specific headers as the packet travels up the
   * stack.
   */
  class DownstreamPacket : public Packet
  {
  public:
    DownstreamPacket(const PacketInfo & info, const void * buf, size_t size);
    
    ~DownstreamPacket(){};
   
    /**
     * Prepend @a bytes of @a buf to the begining of packet.  This method is used
     * to add layer specific headers to a packet
     *
     * @param buf Pointer to the start of the layer specific data
     * @param size Number of bytes to prepend
     */
    void prepend(const void * buf, size_t size);

    /**
     * Combine the internal representation of the message into a single
     * contiguous buffer suitable for network transport.
     */
    void combine();

    /**
     * Get a pointer to the internal buffer holding the message
     *
     * @return @c Ptr to message or @c 0 if the packet was not combined or 
     * if there is no data
     *
     */
    const void * get() const;

    /**
     * Get the packet length in bytes
     *
     * @return length in bytes
     */
    size_t length() const;
    
    /**
     * Get a reference to the packet information
     *
     * @return packet information
     */
    const PacketInfo & getPacketInfo() const;

  private:
    typedef std::vector<unsigned char> PacketSegment;
    typedef std::list<PacketSegment> SegmentList;

    SegmentList segmentList_;
    PacketSegment::size_type totalLengthBytes_;
    PacketInfo info_;
  };

#include "emane/emanedownstreampacket.inl"
}

#endif // EMANEDOWNSTREAMPACKET_HEADER_
