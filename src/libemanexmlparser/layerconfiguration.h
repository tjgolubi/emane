/*
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef LAYERCONFIGURATION_HEADER_
#define LAYERCONFIGURATION_HEADER_

#include "emaneparseexception.h"
#include "emane/emaneconfigurationitem.h"
#include "emane/emanecomponent.h"
#include "configurationparser.h"

#include <string>
#include <map>

#include <libxml/tree.h>

namespace EMANE
{
  /**
   * @class LayerConfiguration
   *
   * @brief Provides default implementation to common layer functionalities
   */
  class LayerConfiguration
  {
  public:
    /* Parameter holder */
    typedef std::multimap<std::string, EMANE::ConfigurationItem> ParamContainer;

    /**
     * Constructor
     *
     * @param pParser Pointer to the Configuration Parser
     */
    LayerConfiguration(ConfigurationParser *pParser);

    /**
     * Destructor
     */
    virtual ~LayerConfiguration();

    /**
     * Updates the internals of this layer
     *
     * @param pNode Pointer to the root node with updates
     */
    void update(xmlNodePtr pNode);

    /**
     * Updates the internals of this layer
     *
     * @param rParams Const reference to a ParamContainer with updated items
     */
    void update(const ParamContainer &rParams);

    /**
     * Returns the configuration items
     *
     * @return ConfigurationItems accumulated through parsing
     */
    virtual EMANE::ConfigurationItems getConfigurationItems();

    /**
     * Returns the pointer to the configuration items
     *
     * @return Pointer to ConfigurationItems accumulated through parsing
     */
    virtual EMANE::ConfigurationItems *getConfigurationItemsPtr();

    /**
     * Returns the name of this layer
     *
     * @return Name of this layer
     */
    virtual std::string getName();

    /**
     * Returns the library with this layer
     *
     * @return Library for this layer
     */
    virtual std::string getLibrary();

    /**
     * Returns the type with this layer
     *
     * @return Type for this layer
     */
    virtual std::string getType() = 0;

    /**
     * Returns the URI of the definition used for this layer
     *
     * @return Definition URI for this layer
     */
    std::string getDefinitionURI();

  protected:
    /**
     * Processes the definition of a given layer
     *
     * @param pxzName Name of the xml node to expect as root
     * @param sURI The URI to parse
     *
     * @exception ParseException ValidateException
     */
    void processDefinition(const xmlChar *pxzName, const std::string &sURI)
      throw(ParseException,ValidateException);

    /**
     * Forwards the processing of the root node to a specific derived class
     *
     * @param pRoot Pointer to the root node
     *
     * @exception ParseException ValidateException
     */
    virtual void doProcessRootNode(xmlNodePtr pRoot)
      throw(ParseException,ValidateException) = 0;

    /**
     * Forwards the child node to a specific derived class
     * 
     * @param pNode Pointer to the node
     *
     * @exception ParseException ValidateException
     */
    virtual void doProcessChildNode(xmlNodePtr pNode)
      throw(ParseException,ValidateException) = 0;    

    /**
     * The configuration parser
     */
    ConfigurationParser *pParser_;

  private:
    /**
     * Updates local storage of configuration items to correspond to updated
     * parameters.
     */
    void updateConfigurationItems();

    /**
     * Actual defined name of this layer
     */
    std::string sLayerName_;

    /**
     * Definition URI of this layer
     */
    std::string sDefinitionURI_;

    /**
     * Configuration Items
     */
    ConfigurationItems items_;
    
    /**
     * Local storage of configuration items
     */
    ParamContainer params_;
  };

}

#endif /* LAYERCONFIGURATION_HEADER_ */
