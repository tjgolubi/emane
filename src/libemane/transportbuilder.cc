/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "libemane/transportbuilder.h"
#include "concretetransportmanager.h"
#include "concretetransportadapter.h"
#include "transportfactorymanager.h"
#include "logservice.h"
#include "timerservice.h"
#include "eventservice.h"
#include "statisticsmanager.h"
#include "platformservice.h"
#include "componentmap.h"

#include <sstream>

typedef EMANE::TransportAdapters::const_iterator TransportAdaptersIter;

EMANE::TransportBuilder::TransportBuilder(){}

EMANE::TransportBuilder::~TransportBuilder(){}

EMANE::TransportManager *
EMANE::TransportBuilder::buildTransportManager(
                              const EMANE::TransportAdapters &adapters,
                              const ConfigurationItems * pItems)
  throw(BuildException,ConfigureException,InitializeException)
{
  if(adapters.empty())
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build a TransportManager without any TransportAdapters"
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }

  if(ComponentMapSingleton::instance()->registeredTransportManagerCount() > 0)
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build multiple TransportManagers. Only one permitted."
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }      

  BuildId buildId = ComponentMapSingleton::instance()->generateId();
  TransportManager * pManager = new ConcreteTransportManager;
  pManager->setBuildId(buildId);
  
  pManager->initialize();
  
  if(pItems)
    {
      pManager->configure(*pItems);
    }

  // register the configured manager
  ComponentMapSingleton::instance()->registerTransportManager(buildId);

  for(TransportAdaptersIter iter=adapters.begin(); 
      iter!=adapters.end(); 
      iter++)
    {
      // check that adapter isn't already in use
      if((*iter)->isContained())
        {
          std::stringstream sstream;
          
          sstream<<"Trying to add TransportAdapter to a TranportManager "
                 <<"that already belongs to another Manager."
                 <<std::endl
                 <<std::ends;

          throw BuildException(sstream.str());
        }

      // add adapter
      pManager->add(*iter);
      (*iter)->setContained();
    }

  return pManager;
}


EMANE::TransportAdapter *
EMANE::TransportBuilder::buildTransportAdapter(
                              EMANE::Transport * pTransport,
                              const ConfigurationItems * pItems)
  throw(BuildException,ConfigureException,InitializeException)
{
  if(pTransport == NULL)
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build a TransportAdapter without a Transport"
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }

  // set transport
  if(pTransport->isContained())
    {
      std::stringstream sstream;
          
      sstream<<"Trying to add Transport (nemid="
             <<pTransport->getNEMId()
             <<") to assign a TransportAdapter "
             <<"that already belongs to another Adapter."
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }

  EMANE::TransportAdapter * pAdapter = 
    new ConcreteTransportAdapter(pTransport->getNEMId());
      
  pAdapter->initialize();

  if(pItems)
    {
      pAdapter->configure(*pItems);
    }

  pAdapter->setTransport(pTransport);
  pTransport->setContained();

  return pAdapter;
}


EMANE::Transport *
EMANE::TransportBuilder::buildTransport(NEMId id,
                                        const std::string & sLibraryFile,
                                        const ConfigurationItems * pItems)
  throw(EMANEUtils::FactoryException,ConfigureException,InitializeException)
{
  std::string sNativeLibraryFile = ACE_DLL_PREFIX + 
                                   sLibraryFile + 
                                   ACE_DLL_SUFFIX;

  const EMANE::TransportFactory & transportFactory = 
    EMANE::TransportFactoryManagerSingleton::instance()->getTransportFactory(sNativeLibraryFile);
      
  // new platform service
  EMANE::PlatformService * pPlatformService = 
    new EMANE::PlatformService(LogServiceSingleton::instance(),
                               EventServiceSingleton::instance(),
                               TimerServiceSingleton::instance(),
                               StatisticsManagerSingleton::instance(),
                               0);

  // create transport
  Transport * pTransport = 
    transportFactory.createTransport(id, pPlatformService);

  initializeTransport(pTransport, pPlatformService, pItems);

  return pTransport;
}


EMANE::PlatformServiceProvider * 
EMANE::TransportBuilder::newPlatformService() const
{
  return new EMANE::PlatformService(LogServiceSingleton::instance(),
                                    EventServiceSingleton::instance(),
                                    TimerServiceSingleton::instance(),
                                    StatisticsManagerSingleton::instance(),
                                    0);
}


void 
EMANE::TransportBuilder::initializeTransport(Transport * pTransport, 
                                             PlatformServiceProvider * pProvider,
                                             const ConfigurationItems * pItems) const
  throw(ConfigureException, InitializeException)
{
  // pass transport to platform service
  dynamic_cast<EMANE::PlatformService*>(pProvider)->setPlatformServiceUser(pTransport);
      
  // initialize
  pTransport->initialize();
     
  // configure 
  if(pItems)
    {
      pTransport->configure(*pItems);
    }
}
