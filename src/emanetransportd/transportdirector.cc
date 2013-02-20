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

#include "transportdirector.h"
#include <sstream>

/*
 * Constructor
 *
 * @param filename reference to the base XML filename
 * @param builder reference to the TransportBuilder
 */
EMANE::TransportDirector::TransportDirector(const std::string &filename, 
                                            EMANE::TransportBuilder &builder)
throw(ParseException,ValidateException):
  transportConfig_(filename),
  rTransportBuilder_(builder)
{}
  
/*
 * Destructor
 */
EMANE::TransportDirector::~TransportDirector()
{}

/*
 * Constructs the event agent and generators
 *
 */
EMANE::TransportManager *
EMANE::TransportDirector::construct()
{
  /* Now go through configuration of each instance and build appropriately */
  EMANE::TransportInstanceConfigurationContainer::const_iterator instanceIter = 
    transportConfig_.getInstances().begin();
  
  EMANE::TransportAdapters adapters;
  for ( ; instanceIter != transportConfig_.getInstances().end(); ++instanceIter)
    {
      /* Create a Transport */
      EMANE::Transport * pTransport = createTransport(*instanceIter);

      /* Create the adapter for this transport */
      EMANE::TransportAdapter * pAdapter =
        rTransportBuilder_.buildTransportAdapter(
                                pTransport,
                                (*instanceIter)->getConfigurationItemsPtr());
     
      adapters.push_back(pAdapter);
    }

  /* Build TransportManager */
  return rTransportBuilder_.buildTransportManager(
                                 adapters,
                                 transportConfig_.getConfigurationItemsPtr());
}

/*
 * Uses the passed-in builder to create a Transport and return
 * a pointer to it.
 *
 * @param pTIConfig Pointer to the Instance XML configuration
 *
 * @exception ConfigureException
 */
EMANE::Transport *
EMANE::TransportDirector::createTransport(
                                TransportInstanceConfiguration *pTIConfig)
  throw(EMANE::ConfigureException)
{
  EMANE::LayerConfigurationContainer::const_iterator transportIter = 
    pTIConfig->getLayers().begin();

  EMANE::LayerConfiguration *pTransportConfig = (*transportIter);

  if (pTransportConfig == 0)
    {
      std::stringstream excStream;
      excStream << "Transport inside instance "
                << pTIConfig->getId() << " is NOT properly configured!" 
                << std::ends;
      throw ConfigureException(excStream.str());    
    }
  
  return rTransportBuilder_.buildTransport(
                                 pTIConfig->getId(),
                                 pTransportConfig->getLibrary(),
                                 pTransportConfig->getConfigurationItemsPtr());
}
