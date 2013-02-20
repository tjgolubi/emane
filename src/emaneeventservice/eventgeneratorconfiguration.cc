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

#include "eventgeneratorconfiguration.h"

#include <sstream>
#include <iostream>

#include <libxml/tree.h>

EMANE::EventGeneratorConfiguration::EventGeneratorConfiguration(xmlNodePtr pEventGeneratorNode,
                                                        EMANE::ConfigurationParser *pParser)
  throw(ParseException, ValidateException)
  : LayerConfiguration(pParser)
{
  xmlChar *pxzDefinition = xmlGetProp(pEventGeneratorNode,
                                      reinterpret_cast<const xmlChar*>("definition"));

  std::string sDefinition;

  if (pxzDefinition)
    {
      sDefinition = reinterpret_cast<const char*>(pxzDefinition);
      
      xmlFree(pxzDefinition);
    }

  xmlChar *pxzName = xmlGetProp(pEventGeneratorNode,
                                reinterpret_cast<const xmlChar*>("name"));

  if (pxzName)
    {
      sName_ = reinterpret_cast<const char*>(pxzName);

      xmlFree(pxzName);
    }

  /* Parse the definition */
  processDefinition(reinterpret_cast<const xmlChar *>("eventgenerator"), sDefinition);

  /* Then update with the current node's content */
  update(pEventGeneratorNode);
}

EMANE::EventGeneratorConfiguration::~EventGeneratorConfiguration()
{ }

EMANE::ConfigurationItems EMANE::EventGeneratorConfiguration::getConfigurationItems()
{
  return LayerConfiguration::getConfigurationItems();
}

EMANE::ConfigurationItems *EMANE::EventGeneratorConfiguration::getConfigurationItemsPtr()
{
  return LayerConfiguration::getConfigurationItemsPtr();
}

std::string EMANE::EventGeneratorConfiguration::getName()
{
  return sName_;
}

std::string EMANE::EventGeneratorConfiguration::getLibrary()
{
  return sLibrary_;
}

std::string EMANE::EventGeneratorConfiguration::getType()
{
  return "eventgenerator";
}

void EMANE::EventGeneratorConfiguration::doProcessRootNode(xmlNodePtr pRoot)
  throw(ParseException,ValidateException)
{
  /* Extract attributes we need */
  xmlChar *pxzLib  = xmlGetProp(pRoot, reinterpret_cast<const xmlChar*>("library"));

  if (pxzLib)
    {
      sLibrary_ = reinterpret_cast<const char*>(pxzLib);

      xmlFree(pxzLib);
    }
  
}

void EMANE::EventGeneratorConfiguration::doProcessChildNode(xmlNodePtr)
  throw(ParseException,ValidateException)
{
  //nothing
}
