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

#ifndef SHIMLAYERCONFIGURATION_HEADER_
#define SHIMLAYERCONFIGURATION_HEADER_

#include "configurationparser.h"
#include "emaneparseexception.h"
#include "layerconfiguration.h"

#include <string>

namespace EMANE
{
  /**
   * @class SHIMLayerConfiguration
   *
   * @brief Contains the configuration for a SHIM layer
   *
   * @sa LayerConfiguration
   */
  class SHIMLayerConfiguration : public LayerConfiguration
  {
  public:
    
    /**
     * Constructor
     *
     * @param pNode Pointer to the element node
     * @param pParser Pointer to the parser
     *
     * @exception ParseException ValidateException
     */
    SHIMLayerConfiguration(xmlNodePtr pNode,
                           ConfigurationParser *pParser)
      throw(ParseException, ValidateException);

    /**
     * Destructor
     */
    virtual ~SHIMLayerConfiguration();

    /**
     * Returns the name of this layer
     *
     * @return Name of this layer
     */
    std::string getName();

    /**
     * Returns the library with this layer
     *
     * @return Library for this layer
     */
    std::string getLibrary();

    /**
     * Returns the type with this layer
     *
     * @return Type for this layer
     */
    std::string getType();

  protected:
    /**
     * Does processing of the root node as if it was a 'shim'
     *
     * @param pRoot Pointer to the root node
     *
     * @exception ParseException ValidateException
     */
    void doProcessRootNode(xmlNodePtr pRoot)
      throw(ParseException,ValidateException);

    /**
     * Does the processing of child node of a 'shim'
     * 
     * @param pNode Pointer to the node
     *
     * @exception ParseException ValidateException
     */
    void doProcessChildNode(xmlNodePtr pNode)
      throw(ParseException,ValidateException);

  private:   
    /**
     * Name of this SHIM layer
     */
    std::string sName_;

    /**
     * Library used for this layer
     */
    std::string sLibrary_;
    
  };

} // end namespace EMANE

#endif /* SHIMLAYERCONFIGURATION_HEADER_ */
