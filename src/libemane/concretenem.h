/*
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef CONCRETEEMANENEM_HEADER_
#define CONCRETEEMANENEM_HEADER_

#include "emane/emanenem.h"
#include "nemlayerstack.h"
#include "nemotaadapter.h"
#include "nemnetworkadapter.h"
#include "logserviceproxy.h"

#include "emane/emanecomponent.h"

#include "emaneutils/componenttypes.h"

#include <memory>

namespace EMANE
{
  /**
   * @class ConcreteNEM
   *
   * @brief Implementation of the Network emulation module consisting of NEM
   * components, OTA Adapater and network adapter
   */
  class ConcreteNEM : public EMANE::NEM
  {
  public:
    /**
     * Constructor
     *
     * @param id NEMid of this NEM
     * @param pNEMLayerStack pointer to a NEMLayerStack to contain in this NEM
     *
     */
    ConcreteNEM(NEMId id, 
                NEMLayerStack   * pNEMLayerStack);
    
    ~ConcreteNEM();

    /**
     * Initialize the NEM
     */    
    void initialize()
      throw(InitializeException);
    
    /**
     * Configure the NEM
     *
     * @param items pItems pointer to the NEM ConfigurationItems
     */    
    void configure(const ConfigurationItems & items)
      throw(ConfigureException);
    
    /**
     * Start all the NEMs contained in the platform
     */
    void start()
      throw(StartException);

    /**
     * Run any post start hooks for all the layers contained in this NEM
     */
    void postStart();

    /**
     * Stop all the components contained in this NEM
     */    
    void stop()
      throw(StopException);
    
    /**
     * Destroy all the NEM ans it layer stack
     */
    void destroy()
      throw();

    /**
     * Get the NEM's NEMId
     *
     * @return the NEM's NEMId
     */    
    NEMId getNEMId() const;

    /**
     * Get the Platform Endpoint address of this NEM
     *
     * @return the NEM's Platform Endpoint addresss
     */        
    ACE_INET_Addr getPlatformEndpoint() const;

    /**
     * Get the Transport Endpoint address of this NEM
     *
     * @return the NEM's Transport Endpoint addresss
     */        
    ACE_INET_Addr getTransportEndpoint() const;

  private:
    std::auto_ptr<NEMLayerStack> pNEMLayerStack_;
    NEMId             id_;
    
    NEMOTAAdapter     NEMOTAAdapter_;
    NEMNetworkAdapter NEMNetworkAdapter_;

    ACE_INET_Addr platformEndpointAddr_;
    ACE_INET_Addr transportEndpointAddr_;
    
    // prevent NEM copies
    ConcreteNEM(const ConcreteNEM &);
    ConcreteNEM & operator=(const ConcreteNEM &);
  };
}

#endif //CONCRETEEMANENEM_HEADER_
