/*
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTDAEMONCONFIGURATION_HEADER
#define EMANEEVENTDAEMONCONFIGURATION_HEADER

/*
 * Includes
 */
#include "layerconfiguration.h"
#include "nemconfiguration.h"
#include "emane/emanetypes.h"

#include <string>

/**
 * @class EMANE::EventDaemonConfiguration eventdaemonconfiguration.h "eventdaemonconfiguration.h"
 *
 * @brief Implementation of Configuration to properly configure the EventDaemonManager.
 *
 * The EventDaemonConfiguration class is a concrete class implementing the 
 * Configuration interface. The implementation allows for general treatment
 * of layer configuration data for any/all config layers (including event 
 * generators.
 *
 * @sa LayerConfiguration
 */

namespace EMANE
{
  class EventDaemonConfiguration : public EMANE::LayerConfiguration
  {
  public:
    /**
     * Constructor
     *
     * @param sFile File with the eventdaemon/daemon configuration
     *
     * @exception ParseException ValidateException
     */
    EventDaemonConfiguration(const std::string &sFile)
      throw(ParseException,ValidateException);
    
    /**
     * Destructor
     */
    virtual ~EventDaemonConfiguration();
    
    /**
     * Returns the configuration items
     *
     * @return ConfigurationItems accumulated through parsing
     */
    EMANE::ConfigurationItems getConfigurationItems();

    /**
     * Returns the pointer to the configuration items
     *
     * @return Pointer to ConfigurationItems accumulated through parsing
     */
    EMANE::ConfigurationItems *getConfigurationItemsPtr();

    /**
     * Gets this daemon's nem id
     *
     * @return Id of this daemon's nem
     */
    ACE_UINT16 getNEMId();

    /**
     * Returns the name of this layer
     *
     * @return Name of this layer
     */
    std::string getName();

    /**
     * Returns the type with this layer
     *
     * @return Type for this layer
     */
    std::string getType();

    /**
     * Returns the container with Agent Layers
     *
     * @return Container with Layer (agent) config objects
     */
    const LayerConfigurationContainer &getAgents();
    
  protected:
    /**
     * Does processing of the root node as if it was an 'nem'
     *
     * @param pRoot Pointer to the root node
     *
     * @exception ParseException ValidateException
     */
    void doProcessRootNode(xmlNodePtr pRoot)
      throw(ParseException,ValidateException);

    /**
     * Does processing of a child node as if it was a 'layer'
     * 
     * @param pNode Pointer to the node
     *
     * @exception ParseException ValidateException
     */
    void doProcessChildNode(xmlNodePtr pNode)
      throw(ParseException,ValidateException);

  private:    
    /**
     * Container with Daemon configurations
     */
    LayerConfigurationContainer agents_;

    /**
     * NEMId
     */
    EMANE::NEMId nemId_;

    /**
     * Name of this daemon
     */
    std::string sName_;

  };

} // end namespace EMANE  
#endif /* EMANEEVENTDAEMONCONFIGURATION_HEADER */
