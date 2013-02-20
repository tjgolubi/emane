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

#ifndef EMANECONTROLMESSAGE_HEADER_
#define EMANECONTROLMESSAGE_HEADER_

#include "emane/emanetypes.h"

#include <vector>

namespace EMANE
{
  /*
   * Reserved Radio Control Message.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2011.11.30
   */
  static const INT32 EMANE_RESERVEDRADIO_1_MAC_CONTROLMESSAGE_MAJOR_ID = 200;


  /**
   * @class ControlMessage
   *
   * @brief Opaque control message used to allow
   * generic inter layer communication.
   */
  class ControlMessage
  {
  public:
     /**
      * Constructor
      *
      * @param iMajorIdentifier Major message identifier
      * @param iMinorIdentifier Minor message identifier
      * @param buf Message
      * @param len Message length in bytes
      */
    ControlMessage(EMANE::INT32 iMajorIdentifier, EMANE::INT32 iMinorIdentifier, const void * buf, size_t len);
    
    ~ControlMessage(){};
    
    /**
     * Get a pointer to the control message
     *
     * @return ptr to buffer
     */
    const void * get() const;
    
    /**
     * Get the control message length in bytes
     *
     * @return length in bytes
     */
    size_t length() const;
    
    /**
     * Get the control message major identifier
     *
     * @return major identifier
     */
    EMANE::INT32 getMajorIdentifier() const;

    /**
     * Get the control message minor identifier
     *
     * @return minor identifier
     */
    EMANE::INT32 getMinorIdentifier() const;
    
  private:
    typedef std::vector<unsigned char> ControlSegment;
    EMANE::INT32 iMajorIdentifier_;
    EMANE::INT32 iMinorIdentifier_;
    ControlSegment controlSegment_;
  };
}

#define EMPTY_CONTROL_MESSAGE EMANE::ControlMessage(0,0,0,0)

#include "emane/emanecontrolmessage.inl"

#endif // EMANECONTROLMESSAGE_HEADER_
