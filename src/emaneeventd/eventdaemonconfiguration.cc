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

/*
 * Includes
 */
#include "eventdaemonconfiguration.h"
#include "eventagentconfiguration.h"
#include "configurationparser.h"

#include <ace/OS_NS_stdlib.h>

#include <utility>

EMANE::EventDaemonConfiguration::EventDaemonConfiguration(const std::string &sFile)
  throw(ParseException,ValidateException)
  : LayerConfiguration(EMANE::ConfigurationParserSingleton::instance())
{ 
  processDefinition(reinterpret_cast<const xmlChar*>("eventdaemon"), sFile);
}
  
/*
 * Destructor
 */
EMANE::EventDaemonConfiguration::~EventDaemonConfiguration()
{
  LayerConfigurationContainer::iterator iter = agents_.begin();

  for (; iter != agents_.end(); ++iter)
    {
      delete (*iter);
    }

  agents_.clear();
}

EMANE::ConfigurationItems EMANE::EventDaemonConfiguration::getConfigurationItems()
{
  return LayerConfiguration::getConfigurationItems();
}

EMANE::ConfigurationItems *EMANE::EventDaemonConfiguration::getConfigurationItemsPtr()
{
  return LayerConfiguration::getConfigurationItemsPtr();
}

/*
 * Returns the NEM Id attribute
 *
 * @return nemId The NEM id attribute 
 */
EMANE::NEMId EMANE::EventDaemonConfiguration::getNEMId()
{
  return nemId_;
}

std::string EMANE::EventDaemonConfiguration::getName()
{
  return sName_;
}

std::string EMANE::EventDaemonConfiguration::getType()
{
  return "eventdaemon";
}

const EMANE::LayerConfigurationContainer &EMANE::EventDaemonConfiguration::getAgents()
{
  return agents_;
}

void EMANE::EventDaemonConfiguration::doProcessRootNode(xmlNodePtr pRoot)
  throw(ParseException,ValidateException)
{
  xmlChar *pxzName  = xmlGetProp(pRoot, reinterpret_cast<const xmlChar*>("name"));
  xmlChar *pxzNemId = xmlGetProp(pRoot, reinterpret_cast<const xmlChar*>("nemid"));

  if (pxzName)
    {
      sName_ = reinterpret_cast<const char*>(pxzName);

      xmlFree(pxzName);
    }

  if (pxzNemId)
    {
      nemId_ = static_cast<EMANE::NEMId>(ACE_OS::strtoul(reinterpret_cast<const char*>(pxzNemId),
                                                         0,
                                                         10));

      xmlFree(pxzNemId);
    }

}

void EMANE::EventDaemonConfiguration::doProcessChildNode(xmlNodePtr pNode)
  throw(ParseException,ValidateException)
{
  LayerConfiguration *pAgentConfig = new EventAgentConfiguration(pNode, pParser_);

  if (pAgentConfig)
    {
      agents_.push_back(pAgentConfig);
    }

}
