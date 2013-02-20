/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#include "libemane/eventagentbuilder.h"
#include "eventagentfactorymanager.h"
#include "concreteeventagentmanager.h"
#include "logservice.h"
#include "timerservice.h"
#include "eventservice.h"
#include "platformservice.h"
#include "statisticsmanager.h"
#include "componentmap.h"

#include <sstream>

typedef EMANE::EventAgents::const_iterator EventAgentsIter;


EMANE::EventAgentBuilder::EventAgentBuilder(){}

EMANE::EventAgentBuilder::~EventAgentBuilder(){}

EMANE::EventAgentManager *
EMANE::EventAgentBuilder::buildEventAgentManager(
                               const EventAgents &agents,
                               const ConfigurationItems * pItems)
  throw(BuildException,ConfigureException,InitializeException)
{
  if(agents.empty())
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build an EventAgentManager without any EventAgents"
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }

  if(ComponentMapSingleton::instance()->registeredEventAgentManagerCount() > 0)
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build multiple EventAgentManagers. Only one permitted."
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }      

  BuildId buildId = ComponentMapSingleton::instance()->generateId();
  EMANE::EventAgentManager * pManager = new ConcreteEventAgentManager;
  pManager->setBuildId(buildId);
  pManager->initialize();
  
  if(pItems)
    {
      pManager->configure(*pItems);
    }

  // register the configured manager
  ComponentMapSingleton::instance()->registerEventAgentManager(buildId);

  for(EventAgentsIter e=agents.begin(); e != agents.end(); e++) 
    {
      // check that adapter isn't already in use
      if((*e)->isContained())
        {
          std::stringstream sstream;
          
          sstream<<"Trying to add EventAgent to an EventAgentManager "
                 <<"that already belongs to another Manager."
                 <<std::endl
                 <<std::ends;

          throw BuildException(sstream.str());
        }

      pManager->add(*e);
      (*e)->setContained();
    }
  
  return pManager;
}


EMANE::EventAgent *
EMANE::EventAgentBuilder::buildEventAgent(EMANE::NEMId nemId,
                                          const std::string & sLibraryFile,
                                          const ConfigurationItems * pItems)
  throw(EMANEUtils::FactoryException,ConfigureException,InitializeException)
{
  std::string sNativeLibraryFile = ACE_DLL_PREFIX + 
                                   sLibraryFile + 
                                   ACE_DLL_SUFFIX;

  const EMANE::EventAgentFactory & eventAgentFactory = 
    EMANE::EventAgentFactoryManagerSingleton::instance()->getEventAgentFactory(sNativeLibraryFile);
      
  // new platform service 
  EMANE::PlatformService * pPlatformService = 
    new EMANE::PlatformService(LogServiceSingleton::instance(),
                               EventServiceSingleton::instance(),
                               TimerServiceSingleton::instance(),
                               StatisticsManagerSingleton::instance(),
                               0);

  // create agent
  EventAgent * pAgent = 
    eventAgentFactory.createEventAgent(nemId, pPlatformService);

  // pass agent to platform service
  pPlatformService->setPlatformServiceUser(pAgent);
 
  // initialize 
  pAgent->initialize();

  // configure
  if(pItems)
    {
      pAgent->configure(*pItems);
    }

  return pAgent;
}
