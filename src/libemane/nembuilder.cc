/*
 * Copyright (c) 2008-2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "libemane/nembuilder.h"
#include "timerservice.h"
#include "maclayer.h"
#include "phylayer.h"
#include "shimlayer.h"
#include "nemstatefullayer.h"
#include "concretenem.h"
#include "concreteplatform.h"
#include "componentmap.h"
#include "layerfactorymanager.h"
#include "logservice.h"
#include "logserviceproxy.h"
#include "eventservice.h"
#include "platformservice.h"

#include <sstream>


typedef EMANE::NEMLayers::const_iterator NEMLayersIter;
typedef EMANE::NEMs::const_iterator NEMsIter; 


EMANE::NEMBuilder::NEMBuilder(){}

EMANE::NEMBuilder::~NEMBuilder(){}


EMANE::NEMLayer *
EMANE::NEMBuilder::buildPHYLayer(NEMId id,
                                 const std::string & sLibraryFile,
                                 const ConfigurationItems * pItems)
  throw(EMANEUtils::FactoryException,ConfigureException,InitializeException)
{
  std::string sNativeLibraryFile = ACE_DLL_PREFIX + 
                                   sLibraryFile + 
                                   ACE_DLL_SUFFIX;

  const EMANE::PHYLayerFactory & phyLayerFactory = 
    EMANE::LayerFactoryManagerSingleton::instance()->getPHYLayerFactory(sNativeLibraryFile);
   
  // new log serivce proxy
  LogServiceProxy * pLogServiceProxy = 
    new LogServiceProxy(LogServiceSingleton::instance(),ERROR_LEVEL);

  BuildId buildId = ComponentMapSingleton::instance()->generateId();

  // new platform service
  EMANE::PlatformService * pPlatformService = 
    new EMANE::PlatformService(pLogServiceProxy,
                               EventServiceSingleton::instance(),
                               TimerServiceSingleton::instance(),
                               StatisticsManagerSingleton::instance(),
                               buildId);
      
  // create plugin
  PHYLayerImplementor * impl = 
    phyLayerFactory.createLayer(id, pPlatformService);

  // new concrete layer 
  NEMLayer * pNEMLayer = 
    new NEMStatefulLayer(id, 
                         new PHYLayer(id, impl, pPlatformService),
                         pPlatformService);
  pNEMLayer->setBuildId(buildId);

  // register to the component map
  ComponentMapSingleton::instance()->registerNEMLayer(buildId,
                                                      COMPONENT_PHYILAYER,
                                                      pNEMLayer);
    
  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(pNEMLayer);

  // register log service proxy 
  LogServiceSingleton::instance()->registerProxy(id, 
                                                 pLogServiceProxy, 
                                                 COMPONENT_PHYILAYER);

  // register event service handler with event service
  EventServiceSingleton::instance()->registerEventServiceHandler(id, COMPONENT_PHYILAYER, pNEMLayer);

  // initialize
  pNEMLayer->initialize();
     
  // configure 
  if(pItems)
    {
      pNEMLayer->configure(*pItems);
    }

  return pNEMLayer;
}


EMANE::NEMLayer *
EMANE::NEMBuilder::buildMACLayer(NEMId id,
                                 const std::string & sLibraryFile,
                                 const ConfigurationItems * pItems)
  throw(EMANEUtils::FactoryException,ConfigureException,InitializeException)
{
  std::string sNativeLibraryFile = ACE_DLL_PREFIX + 
                                   sLibraryFile + 
                                   ACE_DLL_SUFFIX;

  const EMANE::MACLayerFactory & macLayerFactory = 
    EMANE::LayerFactoryManagerSingleton::instance()->getMACLayerFactory(sNativeLibraryFile);
      
  // new log serivce proxy
  LogServiceProxy * pLogServiceProxy = 
    new LogServiceProxy(LogServiceSingleton::instance(),ERROR_LEVEL);

  // create a unique id for the new mac
  BuildId buildId = ComponentMapSingleton::instance()->generateId();


  // new platform service
  EMANE::PlatformService * pPlatformService = 
    new EMANE::PlatformService(pLogServiceProxy,
                               EventServiceSingleton::instance(),
                               TimerServiceSingleton::instance(),
                               StatisticsManagerSingleton::instance(),
                               buildId);

  // create plugin
  MACLayerImplementor * impl = 
    macLayerFactory.createLayer(id, pPlatformService);

  // new concreate layer 
  NEMLayer * pNEMLayer =  
    new NEMStatefulLayer(id, 
                         new MACLayer(id, impl, pPlatformService), 
                         pPlatformService);
  pNEMLayer->setBuildId(buildId);

  // register to the component map
  ComponentMapSingleton::instance()->registerNEMLayer(buildId,
                                                      COMPONENT_MACILAYER,
                                                      pNEMLayer);

  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(pNEMLayer);
     
  // register log service proxy 
  LogServiceSingleton::instance()->registerProxy(id, 
                                                 pLogServiceProxy, 
                                                 COMPONENT_MACILAYER);

  // register event service handler with event service
  EventServiceSingleton::instance()->registerEventServiceHandler(id,COMPONENT_MACILAYER,pNEMLayer);

  // initialize
  pNEMLayer->initialize();

  // configure
  if(pItems)
    {
      pNEMLayer->configure(*pItems);
    }

  return pNEMLayer;
}


EMANE::NEMLayer *
EMANE::NEMBuilder::buildShimLayer(NEMId id,
                                  const std::string & sLibraryFile,
                                  const ConfigurationItems * pItems)
  throw(EMANEUtils::FactoryException,ConfigureException,InitializeException)
{
  std::string sNativeLibraryFile = ACE_DLL_PREFIX + 
                                   sLibraryFile + 
                                   ACE_DLL_SUFFIX;



  const EMANE::ShimLayerFactory & shimLayerFactory = 
    EMANE::LayerFactoryManagerSingleton::instance()->getShimLayerFactory(sNativeLibraryFile);
      
  // new log serivce proxy
  LogServiceProxy * pLogServiceProxy = 
    new LogServiceProxy(LogServiceSingleton::instance(),ERROR_LEVEL);

  // create a unique id for the new shim
  BuildId buildId = ComponentMapSingleton::instance()->generateId();

  // new platform service
  EMANE::PlatformService * pPlatformService = 
    new EMANE::PlatformService(pLogServiceProxy,
                               EventServiceSingleton::instance(),
                               TimerServiceSingleton::instance(),
                               StatisticsManagerSingleton::instance(),
                               buildId);

  // create plugin
  ShimLayerImplementor * impl = 
    shimLayerFactory.createLayer(id, pPlatformService); 
     
  // new concreate layer 
  NEMLayer * pNEMLayer = 
    new NEMStatefulLayer(id, 
                         new ShimLayer(id, impl, pPlatformService), 
                         pPlatformService);
  pNEMLayer->setBuildId(buildId);

  // register to the component map
  ComponentMapSingleton::instance()->registerNEMLayer(buildId,
                                                      COMPONENT_SHIMILAYER,
                                                      pNEMLayer);

  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(pNEMLayer);

  // register log service proxy 
  LogServiceSingleton::instance()->registerProxy(id, pLogServiceProxy, COMPONENT_SHIMILAYER);

  // register event service handler with event service
  EventServiceSingleton::instance()->registerEventServiceHandler(id,COMPONENT_SHIMILAYER,pNEMLayer);
     
  // initialize 
  pNEMLayer->initialize();

  // configure
  if(pItems)
    {
      pNEMLayer->configure(*pItems);
    }

  return pNEMLayer;
}


EMANE::NEM * 
EMANE::NEMBuilder::buildNEM(NEMId id, 
                            const NEMLayers &layers,
                            const ConfigurationItems * pItems)
  throw(ConfigureException,
        InitializeException,
        BuildException)
{
  /* Create NEM*/
  BuildId buildId = ComponentMapSingleton::instance()->generateId();

  EMANE::NEMLayerStack * pLayerStack = new NEMLayerStack;

  if(layers.empty())
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build a NEM without NEMLayers."
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }

  std::vector<BuildId> nemBuildIds;
  for(NEMLayersIter iter=layers.begin(); iter != layers.end(); ++iter)
    {
      if((*iter)->isContained())
        {
          std::stringstream sstream;
          
          sstream<<"Trying to add NEMLayer to NEM (id="
                 <<id
                 <<") that already belongs to another NEM."
                 <<std::endl
                 <<std::ends;

          throw BuildException(sstream.str());
        }
      pLayerStack->addLayer(*iter);

      if((*iter)->getNEMId() != id)
        {
          std::stringstream sstream;
          
          sstream<<"NEMId mismatch: NEMLayer ("
                 <<(*iter)->getNEMId()
                 <<") NEM ("
                 <<id
                 <<")"
                 <<std::endl
                 <<std::ends;

          throw BuildException(sstream.str());
        }

      (*iter)->setContained();
      // save layer ids for registration after nem is created
      nemBuildIds.push_back((*iter)->getBuildId());
    }

  NEM* pNEM =  new ConcreteNEM(id, pLayerStack);
  pNEM->setBuildId(buildId);


  // register to the component map
  ComponentMapSingleton::instance()->registerNEM(buildId, id, pNEM);

  // register nem layers now that NEM is created
  for(std::vector<BuildId>::const_iterator nci = nemBuildIds.begin();
      nci != nemBuildIds.end();
      ++nci)
    {
      ComponentMapSingleton::instance()->assignNEMLayerToNEM(buildId, 
                                                             *nci);
    }

  pNEM->initialize();

  if(pItems)
    {
      pNEM->configure(*pItems);
    }

  return pNEM;
}


EMANE::Platform * 
EMANE::NEMBuilder::buildPlatform(PlatformId id, 
                                 const NEMs &nems,
                                 const ConfigurationItems * pItems)
  throw(ConfigureException,InitializeException,BuildException)
{
  if(nems.empty())
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build a Platform without any NEMs."
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }

  if(ComponentMapSingleton::instance()->registeredPlatformCount() > 0)
    {
      std::stringstream sstream;
          
      sstream<<"Trying to build multiple platforms. Only one permitted."
             <<std::endl
             <<std::ends;

      throw BuildException(sstream.str());
    }      

  BuildId buildId = ComponentMapSingleton::instance()->generateId();

  EMANE::Platform * pPlatform = 
    new ConcretePlatform(id,
                         StatisticsManagerSingleton::instance());
  pPlatform->setBuildId(buildId);

  pPlatform->initialize();

  if(pItems)
    {
      pPlatform->configure(*pItems);
    }

  // register the configured Platform
  ComponentMapSingleton::instance()->registerPlatform(buildId, 
                                                      id,
                                                      pPlatform);

  for(NEMsIter iter=nems.begin(); iter != nems.end(); ++iter)
    {
      if((*iter)->isContained())
        {
          std::stringstream sstream;
          
          sstream<<"Trying to add NEM (id="
                 <<(*iter)->getNEMId()
                 <<") to Platform (id="
                 <<id
                 <<") that already belongs to another Platform."
                 <<std::endl
                 <<std::ends;

          throw BuildException(sstream.str());
        }
      pPlatform->add((*iter)->getNEMId(), *iter);
      (*iter)->setContained();

      ComponentMapSingleton::instance()->assignNEMToPlatform(buildId, 
                                                             (*iter)->getBuildId());
    }

  return pPlatform;
}
