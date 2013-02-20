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

#ifndef EMANENEMBUILDER_HEADER_
#define EMANENEMBUILDER_HEADER_

#include "emane/emaneplatform.h"
#include "emane/emanenem.h"
#include "emane/emanenemlayer.h"
#include "emane/emaneconfigurationitem.h"
#include "emane/emanebuildexception.h"

#include "emaneutils/factoryexception.h"

#include <string>
#include <list>

namespace EMANE
{
  typedef std::list<EMANE::NEMLayer*> NEMLayers;
  typedef std::list<EMANE::NEM*> NEMs;

  /**
   * @class NEMBuilder
   *
   * @brief Provides methods for contructing an EMANE Platform from
   * its constituent parts. 
   *
   * @note Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
   * Design Patterns: Elements of Reusable Object-Oriented Software.
   * Addison-Wesley, Reading MA, 1995
   * Bridge, p 152
   */
  class NEMBuilder
  {
  public:
    NEMBuilder();

    ~NEMBuilder();
    
    /**
     * Build a PHY layer
     *
     * @param id id of the NEM that will contain the phy
     * @param sLibraryFile Name of the dll containing the layer
     * @param pItems pointer to ConfigurationItems list
     * @return pointer to an initialized and configured phy
     */
    EMANE::NEMLayer * 
    buildPHYLayer(NEMId id,
                  const std::string & sLibraryFile,
                  const ConfigurationItems * pItems = 0)
      throw(EMANEUtils::FactoryException,
            ConfigureException,
            InitializeException);

    /**
     * Build a MAC layer
     *
     * @param id id of the NEM that will contain the mac
     * @param sLibraryFile Name of the dll containing the layer
     * @param pItems pointer to ConfigurationItems list
     * @return pointer to an initialized and configured mac
     */
    EMANE::NEMLayer *
    buildMACLayer(NEMId id,
                  const std::string & sLibraryFile,
                  const ConfigurationItems * pItems = 0)
      throw(EMANEUtils::FactoryException,
            ConfigureException,
            InitializeException);

    /**
     * Build a Shim layer
     *
     * @param id id of the NEM that will contain the shim
     * @param sLibraryFile Name of the dll containing the layer
     * @param pItems pointer to ConfigurationItems list
     * @return pointer to an initialized and configured shim
     */
    EMANE::NEMLayer *
    buildShimLayer(NEMId id,
                   const std::string & sLibraryFile,
                   const ConfigurationItems * pItems = 0)
      throw(EMANEUtils::FactoryException,
            ConfigureException,
            InitializeException);

    /**
     * Build a NEM
     *
     * @param id NEM id
     * @param layers The NEMLayers comprising the NEM
     * @param pItems pointer to ConfigurationItems list
     * @return pointer to an initialized and configured NEM
     */
    EMANE::NEM * buildNEM(NEMId id, 
                          const NEMLayers &layers,
                          const ConfigurationItems * pItems = 0)
      throw(ConfigureException,
            InitializeException,
            BuildException);

    /**
     * Build a Platform
     *
     * @param id Platform id
     * @param nems The NEMs comprising the Platform
     * @param pItems Pointer to the Platform configuration items
     * @return pointer to an initialized and configured Platform
     */
    EMANE::Platform * 
    buildPlatform(PlatformId id,
                  const NEMs &nems,
                  const ConfigurationItems * pItems = 0)
    throw(ConfigureException,
          InitializeException,
          BuildException);
  };
}

#endif //EMANENEMBUILDER_HEADER_
