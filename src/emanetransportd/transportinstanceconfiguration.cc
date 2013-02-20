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

#include "transportinstanceconfiguration.h"
#include "transportlayerconfiguration.h"

#include <sstream>
#include <iostream>

#include <libxml/tree.h>

EMANE::TransportInstanceConfiguration::TransportInstanceConfiguration(xmlNodePtr pTransportInstanceNode,
                                                                      EMANE::ConfigurationParser *pParser)
  throw(ParseException, ValidateException)
  : LayerConfiguration(pParser),
    u16Id_(0)
{
  /* Get nem Id */
  xmlChar *pxzTransportInstanceId   = xmlGetProp(pTransportInstanceNode, 
                                                 reinterpret_cast<const xmlChar*>("nemid"));

  u16Id_ = ACE_OS::strtoul(reinterpret_cast<const char*>(pxzTransportInstanceId), 0, 10);

  xmlFree(pxzTransportInstanceId);

  /* Then update with the current node's content */
  update(pTransportInstanceNode);
}

EMANE::TransportInstanceConfiguration::~TransportInstanceConfiguration()
{
  LayerConfigurationContainer::iterator iter = layers_.begin();

  for (; iter != layers_.end(); ++iter)
    {
      delete (*iter);
    }

  layers_.clear();
}

EMANE::ConfigurationItems EMANE::TransportInstanceConfiguration::getConfigurationItems()
{
  return LayerConfiguration::getConfigurationItems();
}

EMANE::ConfigurationItems *EMANE::TransportInstanceConfiguration::getConfigurationItemsPtr()
{
  return LayerConfiguration::getConfigurationItemsPtr();
}

ACE_UINT16 EMANE::TransportInstanceConfiguration::getId()
{
  return u16Id_;
}

std::string EMANE::TransportInstanceConfiguration::getType()
{
  return "instance";
}

const EMANE::LayerConfigurationContainer &EMANE::TransportInstanceConfiguration::getLayers()
{
  return layers_;
}

void EMANE::TransportInstanceConfiguration::doProcessRootNode(xmlNodePtr)
  throw(ParseException,ValidateException)
{
  //nothing, processDefinition is not called by constructor
}

void EMANE::TransportInstanceConfiguration::doProcessChildNode(xmlNodePtr pNode)
  throw(ParseException,ValidateException)
{
  std::string sLayerName = reinterpret_cast<const char*>(pNode->name);

  xmlChar *pxzDefinition = xmlGetProp(pNode,
                                      reinterpret_cast<const xmlChar*>("definition"));
  
  /* 
   * 1) If definition exists, overwrite the layer
   * 2) If definition is not there, use the already-present layer
   *    and overwrite what's necessary
   */
  if (pxzDefinition)
    {
      if (sLayerName == "transport")
        {
          // ok
          layers_.push_back(new TransportLayerConfiguration(pNode, pParser_));
        }
      else
        {
          // error
          /* Invalid layer found */
          std::stringstream sstream;
          
          sstream<<"Failed to parse '"
                 <<sLayerName
                 <<"' layer."
                 <<std::endl
                 <<std::endl
                 <<"Possible reason(s):"
                 <<std::endl
                 <<" * "
                 <<"'"
                 <<sLayerName
                 <<"' layer is not allowed inside an 'instance'."
                 <<std::endl
                 <<std::ends;
          
          throw ParseException(sstream.str());
        }

      xmlFree(pxzDefinition);      
    }// end if definition
  else
    {
      // error
      /* No definition found for 'transport */
      std::stringstream sstream;
      
      sstream<<"Failed to parse '"
             <<sLayerName
             <<"' layer."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<"'"
             <<sLayerName
             <<"' layer definition missing."
             <<std::endl
             <<std::ends;
      
      throw ParseException(sstream.str());
    }
 
}
