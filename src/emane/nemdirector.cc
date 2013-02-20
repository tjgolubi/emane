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

#include "configurationparser.h"
#include "nemdirector.h"
#include <sstream>

#include "emane/emaneexception.h"
/*
 * Constructor
 *
 * @param filename reference to the base XML filename
 * @param builder reference to the NEM builder
 */
EMANE::NEMDirector::NEMDirector(const std::string &filename,
                                EMANE::NEMBuilder &builder) 
  throw(ParseException,ValidateException):
  configPlatform_(EMANE::ConfigurationParserSingleton::instance(), filename),
  rNEMBuilder_(builder)
{}
  
/*
 * Destructor
 */
EMANE::NEMDirector::~NEMDirector()
{}

/*
 * Constructs the passed-in platform
 *
 */
EMANE::Platform * EMANE::NEMDirector::construct()
{
  EMANE::NEMConfigurationContainer::const_iterator iter = 
    configPlatform_.getNEMs().begin();
  
  EMANE::NEMs nems;
  for ( ; iter != configPlatform_.getNEMs().end(); ++iter) 
    {
      EMANE::NEM *pNEM = createNEM(*iter);
      nems.push_back(pNEM);
    }

  /* Construct a platform first (initialized) */
  return  rNEMBuilder_.buildPlatform(
                            configPlatform_.getPlatformId(),
                            nems,
                            configPlatform_.getConfigurationItemsPtr());
}

/*
 * Uses the passed-in builder to create an NEM object and return
 * a pointer to it.
 *
 * @param pNEMConfig Pointer to the NEMConfiguration object for this NEM
 *
 * @retval a pointer to the newly created NEM object (memory
 *         ownership is trasferred to the caller).
 *
 * @exception ConfigureException
 */
EMANE::NEM* EMANE::NEMDirector::createNEM(EMANE::NEMConfiguration *pNEMConfig)
  throw(ConfigureException,EMANEUtils::FactoryException,PlatformException)
{
  NEMLayers layers;
  if (pNEMConfig->isValid()) 
    {
      EMANE::LayerConfigurationContainer::const_iterator layerIter = 
        pNEMConfig->getLayers().begin();

      for ( ; layerIter != pNEMConfig->getLayers().end(); ++layerIter) 
        {
          if ((*layerIter)->getType() == "phy") 
            {
              layers.push_back(rNEMBuilder_.buildPHYLayer(
                                   pNEMConfig->getNEMId(),
                                   (*layerIter)->getLibrary(),
                                   (*layerIter)->getConfigurationItemsPtr()));
            }
          else if ((*layerIter)->getType() == "mac") 
            {
              layers.push_back(rNEMBuilder_.buildMACLayer(
                                   pNEMConfig->getNEMId(),
                                   (*layerIter)->getLibrary(),
                                   (*layerIter)->getConfigurationItemsPtr()));
            }
          else if ((*layerIter)->getType() == "shim") 
            {
              layers.push_back(rNEMBuilder_.buildShimLayer(
                                   pNEMConfig->getNEMId(),
                                   (*layerIter)->getLibrary(),
                                   (*layerIter)->getConfigurationItemsPtr()));
            }
          else 
            {
              /* Transport layer, do not build, but may be useful to 
               * configure other subsytems based on transport's config */
            }
        }// end for layers    
    }// end if valid
  else 
    {
      std::stringstream excStream;
      excStream << "NEM " << pNEMConfig->getName() 
                << " (id = "
                << pNEMConfig->getNEMId()
                << ")"
                << " is NOT properly configured!" 
                << std::endl
                << std::endl
                << "Possible reason(s):"
                << std::endl
                << std::endl
                << " * " 
                << " NEM XML is missing one of phy|mac|transport."
                << std::endl
                << std::ends;

      throw ConfigureException(excStream.str());    
    }

  return rNEMBuilder_.buildNEM(pNEMConfig->getNEMId(), 
                               layers,
                               pNEMConfig->getConfigurationItemsPtr());
}
