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

#ifndef EMANENEMLAYER_HEADER_
#define EMANENEMLAYER_HEADER_

#include "emane/emanecomponent.h"
#include "emane/emaneplatformserviceuser.h"
#include "emane/emaneupstreamtransport.h"
#include "emane/emanedownstreamtransport.h"
#include "emane/emanebuildable.h"

namespace EMANE
{
  /**
   * @class NEMLayer
   *
   * @brief Base class for NEMLayer containers. Builders
   *        construct NEMLayer objects to contain concrete instances
   *        of NEMLayer objects and track them through the
   *        system. This class simply mixes the NEMLayer
   *        interface with the Buildable interface. Model implementors
   *        should derive from one of the layer implementor classes,
   *        not this class.
   */
  class NEMLayer : public Component,
                   public UpstreamTransport,
                   public DownstreamTransport,
                   public PlatformServiceUser,
                   public Buildable
  {
  public:
    virtual ~NEMLayer() {}

    /**
     * Get the NEM identifier.
     *
     * @return NEM identifier
     */
    NEMId getNEMId() const { return id_; }

  protected:
    NEMLayer(NEMId id, 
             PlatformServiceProvider *p) :
      PlatformServiceUser(p),
      id_(id)
      {}

    NEMId         id_;
  };
}

#endif //EMANENEMLAYER_HEADER_
