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

/*
 * Includes
 */
#include "libemane/logger.h"
#include "eventserviceconfiguration.h"
#include "eventgeneratorconfiguration.h"
#include "configurationparser.h"
#include "deploymentmanager.h"

#include <sstream>
#include <iostream>

#include <ace/OS_NS_stdlib.h>


EMANE::EventServiceConfiguration::EventServiceConfiguration(const std::string &sFile)
  throw(ParseException,ValidateException)
  : LayerConfiguration(EMANE::ConfigurationParserSingleton::instance())
{ 
  processDefinition(reinterpret_cast<const xmlChar*>("eventservice"), sFile);
}
  
/*
 * Destructor
 */
EMANE::EventServiceConfiguration::~EventServiceConfiguration()
{
  LayerConfigurationContainer::iterator iter = generators_.begin();

  for (; iter != generators_.end(); ++iter)
    {
      delete (*iter);
    }

  generators_.clear();
}

EMANE::ConfigurationItems EMANE::EventServiceConfiguration::getConfigurationItems()
{
  return LayerConfiguration::getConfigurationItems();
}

EMANE::ConfigurationItems *EMANE::EventServiceConfiguration::getConfigurationItemsPtr()
{
  return LayerConfiguration::getConfigurationItemsPtr();
}

std::string EMANE::EventServiceConfiguration::getDeployment()
{
  return sDeployment_;
}

std::string EMANE::EventServiceConfiguration::getName()
{
  return sName_;
}

std::string EMANE::EventServiceConfiguration::getType()
{
  return "eventservice";
}

const EMANE::LayerConfigurationContainer &EMANE::EventServiceConfiguration::getGenerators()
{
  return generators_;
}

void EMANE::EventServiceConfiguration::doProcessRootNode(xmlNodePtr pRoot)
  throw(ParseException,ValidateException)
{
  xmlChar *pxzName = xmlGetProp(pRoot, reinterpret_cast<const xmlChar*>("name"));
  xmlChar *pxzDepl = xmlGetProp(pRoot, reinterpret_cast<const xmlChar*>("deployment"));

  if (pxzName)
    {
      sName_ = reinterpret_cast<const char*>(pxzName);

      xmlFree(pxzName);
    }

  if (pxzDepl)
    {
      sDeployment_ = reinterpret_cast<const char*>(pxzDepl);

      xmlFree(pxzDepl);

      /* Have the deployment file, prep it (letting exceptions propagate) */
      xmlDocPtr pDoc = pParser_->parse(sDeployment_);

      xmlNodePtr pDeploymentRoot = xmlDocGetRootElement(pDoc);

      if ( pDeploymentRoot && 
           xmlStrEqual(pDeploymentRoot->name, 
                       reinterpret_cast<const xmlChar*>("deployment")) )
        {
          bool bEntriesFound = false;

          xmlNodePtr pPlatform = pDeploymentRoot->children;

          while (pPlatform)
            {
              if (pPlatform->type == XML_ELEMENT_NODE)
                {
                  // get id
                  xmlChar *pxzPlatformId = xmlGetProp(pPlatform, 
                                                      reinterpret_cast<const xmlChar*>("id"));

                  ACE_UINT32 u32PlatformId = 0;

                  if (pxzPlatformId)
                    {
                      u32PlatformId = ACE_OS::strtoul(reinterpret_cast<const char*>(pxzPlatformId),
                                                      0,
                                                      10);

                      xmlFree(pxzPlatformId);
                    }

                  // only continue with a valid platform id
                  if (u32PlatformId)
                    {
                      xmlNodePtr pNem = pPlatform->children;

                      while (pNem)
                        {
                          if (pNem->type == XML_ELEMENT_NODE)
                            {
                              // get id
                              xmlChar *pxzNemId = xmlGetProp(pNem, 
                                                             reinterpret_cast<const xmlChar*>("id"));
                              
                              ACE_UINT32 u32NemId = 0;
                              
                              if (pxzNemId)
                                {
                                  u32NemId = ACE_OS::strtoul(reinterpret_cast<const char*>(pxzNemId),
                                                             0,
                                                             10);
                                  
                                  xmlFree(pxzNemId);
                                }
                              
                              /* Update deployment based on a valid nem id */
                              if (u32NemId)
                                {
                                  EMANE::DeploymentManagerSingleton::instance()->addNEMPlatformEntry(u32NemId,
                                                                                                     u32PlatformId);
                                    
                                  bEntriesFound = true;
                                }
                            }// end if NEM element node

                          pNem = pNem->next;
                        }// end while NEM

                    }// end if PlatformId

                }// end if Platform ELEMENT node

              pPlatform = pPlatform->next;
            }// end while platform

          xmlFreeDoc(pDoc);

          if (!bEntriesFound)
            {
              EMANE::Logger logger;
              logger.log(ERROR_LEVEL,
                         "EventServiceConfig::eventservice: NO Deployment entries found!");
            }
        }// end if deployment
      else
        {
          /* Data failed to validate, invalid root or none present */
          std::stringstream sstream;
          
          sstream<<"Failed to validate '"
                 <<sName_
                 <<"' document."
                 <<std::endl
                 <<std::endl
                 <<"Possible reason(s):"
                 <<std::endl
                 <<" * "
                 <<"'deployment' root is missing."
                 <<std::endl
                 <<" * "
                 <<"Deployment document root is not 'deployment'."
                 <<std::endl
                 <<std::ends;
          
          xmlFreeDoc(pDoc);
          
          throw ValidateException(sstream.str());
        }
          
    }// end if deployment
  else
    {
      /* Failed to parse, Need deployment */
      std::stringstream sstream;
      
      sstream<<"Failed to parse document '"
             <<sName_
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<"'deployment' definition is not specified."
             <<std::endl
             <<std::ends;

      throw ParseException(sstream.str());
    }

}

void EMANE::EventServiceConfiguration::doProcessChildNode(xmlNodePtr pNode)
  throw(ParseException,ValidateException)
{
  LayerConfiguration *pGeneratorConfig = new EventGeneratorConfiguration(pNode, pParser_);

  if (pGeneratorConfig)
    {
      generators_.push_back(pGeneratorConfig);
    }

}
