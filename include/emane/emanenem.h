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

#ifndef EMANENEM_HEADER_
#define EMANENEM_HEADER_

#include "emane/emanecomponent.h"
#include "emane/emanebuildable.h"
#include "emane/emanetypes.h"
#include <ace/INET_Addr.h>

namespace EMANE
{
  /**
   * @class NEM
   *
   * @brief Network emulation module container interface. A container for
   *        NEM component layers connected to a transport.
   *
   * @details Virtual base class for NEM component containers
   */
  class NEM : public EMANE::Component,
              public EMANE::Buildable
  {
  public:
    virtual ~NEM() {}
    
    /**
     * Get the NEM's NEMId
     *
     * @return the NEM's NEMId
     */
    virtual NEMId getNEMId() const = 0;

    /**
     * Get the Platform Endpoint address of this NEM
     *
     * @return the NEM's Platform Endpoint addresss
     */        
    virtual ACE_INET_Addr getPlatformEndpoint() const = 0;

    /**
     * Get the Transport Endpoint address of this NEM
     *
     * @return the NEM's Transport Endpoint addresss
     */        
    virtual ACE_INET_Addr getTransportEndpoint() const = 0;

  protected:
    NEM() {}
  };
}

#endif //EMANENEM_HEADER_
