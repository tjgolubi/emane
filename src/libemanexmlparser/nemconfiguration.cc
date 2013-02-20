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

#include "nemconfiguration.h"
#include "phylayerconfiguration.h"
#include "maclayerconfiguration.h"
#include "shimlayerconfiguration.h"
#include "transportlayerconfiguration.h"

#include <sstream>
#include <iostream>

#include <libxml/tree.h>

EMANE::NEMConfiguration::NEMConfiguration(xmlNodePtr pNEMNode,
                                          EMANE::ConfigurationParser *pParser)
  throw(ParseException, ValidateException)
  : LayerConfiguration(pParser),
    u16Id_(0),
    type_(STRUCTURED),
    pPhy_(layers_.end()),
    pMac_(layers_.end()),
    pTransport_(layers_.end())

{
  xmlChar *pxzNEMDefinition = xmlGetProp(pNEMNode, reinterpret_cast<const xmlChar*>("definition"));

  std::string sDefinition = reinterpret_cast<const char*>(pxzNEMDefinition);

  /* Allow base class to process the definition for this node*/
  processDefinition(pNEMNode->name, sDefinition);

  /* Get Name and Id */
  xmlChar *pxzNEMName = xmlGetProp(pNEMNode, reinterpret_cast<const xmlChar*>("name"));
  xmlChar *pxzNEMId   = xmlGetProp(pNEMNode, reinterpret_cast<const xmlChar*>("id"));

  sName_ = reinterpret_cast<const char*>(pxzNEMName);

  u16Id_ = ACE_OS::strtoul(reinterpret_cast<const char*>(pxzNEMId), 0, 10);

  xmlFree(pxzNEMName);
  xmlFree(pxzNEMId);
  xmlFree(pxzNEMDefinition);

  /* Then update with the current node's content */
  update(pNEMNode);
}

EMANE::NEMConfiguration::~NEMConfiguration()
{
  LayerConfigurationContainer::iterator iter = layers_.begin();

  for (; iter != layers_.end(); ++iter)
    {
      delete (*iter);
    }

  layers_.clear();
}

EMANE::ConfigurationItems EMANE::NEMConfiguration::getConfigurationItems()
{
  return LayerConfiguration::getConfigurationItems();
}

EMANE::ConfigurationItems *EMANE::NEMConfiguration::getConfigurationItemsPtr()
{
  return LayerConfiguration::getConfigurationItemsPtr();
}

ACE_UINT16 EMANE::NEMConfiguration::getNEMId()
{
  return u16Id_;
}

std::string EMANE::NEMConfiguration::getName()
{
  return sName_;
}

std::string EMANE::NEMConfiguration::getType()
{
  return "nem";
}

const EMANE::LayerConfigurationContainer &EMANE::NEMConfiguration::getLayers()
{
  return layers_;
}

bool EMANE::NEMConfiguration::isValid()
{
  return (type_ == UNSTRUCTURED) || (*pPhy_ && *pMac_ && *pTransport_);
}

void EMANE::NEMConfiguration::doProcessRootNode(xmlNodePtr pRoot)
  throw(ParseException,ValidateException)
{
  xmlChar *pxzType = xmlGetProp(pRoot, reinterpret_cast<const xmlChar*>("type"));

  if (xmlStrEqual(pxzType, reinterpret_cast<const xmlChar*>("unstructured")))
    {
      type_ = UNSTRUCTURED;
    }

    xmlFree(pxzType);
}

void EMANE::NEMConfiguration::doProcessChildNode(xmlNodePtr pNode)
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
      std::string sDefinitionURI = reinterpret_cast<const char*>(pxzDefinition);

      /* Check if layer exists, clear and re-set if it does */
      if (sLayerName == "phy")
        {
          if (pPhy_ == layers_.end())
            {
              pPhy_ = layers_.insert(pPhy_, 
                                     new PHYLayerConfiguration(pNode, pParser_));
            }
          else
            {
              if ((*pPhy_)->getDefinitionURI() == sDefinitionURI)
                {
                  (*pPhy_)->update(pNode);
                }
              else
                {
                  /* Invalid definition */
                  std::stringstream sstream;
      
                  sstream<<"Unexpected definition for '"
                         <<sLayerName
                         <<"'."
                         <<std::endl
                         <<std::endl
                         <<" * Found: '"
                         <<sDefinitionURI
                         <<"', "
                         <<"expected :'"
                         <<(*pPhy_)->getDefinitionURI()
                         <<"'"
                         <<std::endl
                         <<std::ends;

                  throw ValidateException(sstream.str());
                }
            }
        }
      else if (sLayerName == "mac")
        {
          if (pMac_ == layers_.end())
            {
              pMac_ = layers_.insert(pMac_, 
                                     new MACLayerConfiguration(pNode, pParser_));
            }
          else
            {
              if ((*pMac_)->getDefinitionURI() == sDefinitionURI)
                {
                  (*pMac_)->update(pNode);
                }
              else
                {
                  /* Invalid definition */
                  std::stringstream sstream;
      
                  sstream<<"Unexpected definition for '"
                         <<sLayerName
                         <<"'."
                         <<std::endl
                         <<std::endl
                         <<" * Found: '"
                         <<sDefinitionURI
                         <<"', "
                         <<"expected :'"
                         <<(*pMac_)->getDefinitionURI()
                         <<"'"
                         <<std::endl
                         <<std::ends;

                  throw ValidateException(sstream.str());
                }
            }
        }
      else if (sLayerName == "transport")
        {
          if (pTransport_ == layers_.end())
            {
              pTransport_ = layers_.insert(pTransport_, 
                                           new TransportLayerConfiguration(pNode, pParser_));
            }
          else
            {
              if ((*pTransport_)->getDefinitionURI() == sDefinitionURI)
                {
                  (*pTransport_)->update(pNode);
                }
              else
                {
                  /* Invalid definition */
                  std::stringstream sstream;
      
                  sstream<<"Unexpected definition for '"
                         <<sLayerName
                         <<"'."
                         <<std::endl
                         <<std::endl
                         <<" * Found: '"
                         <<sDefinitionURI
                         <<"', "
                         <<"expected :'"
                         <<(*pTransport_)->getDefinitionURI()
                         <<"'"
                         <<std::endl
                         <<std::ends;

                  throw ValidateException(sstream.str());
                }
            }
        }
      else
        {
          // shim
          layers_.push_back(new SHIMLayerConfiguration(pNode, pParser_));
        }
      
      xmlFree(pxzDefinition);      
    }// end if definition
  else
    {
      /* Update the layer internals */
      if (sLayerName == "phy")
        {
          if (pPhy_ == layers_.end())
            {
              /* Invalid layer state */
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
                     <<"' layer definition missing from 'nem' or 'platform' file."
                     <<std::endl
                     <<std::ends;
              
              throw ParseException(sstream.str());
            }

          (*pPhy_)->update(pNode);
        }
      else if (sLayerName == "mac")
        {
          if (pMac_ == layers_.end())
            {
              /* Invalid layer state */
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
                     <<"' layer definition missing from 'nem' or 'platform' file."
                     <<std::endl
                     <<std::ends;
              
              throw ParseException(sstream.str());
            }

          (*pMac_)->update(pNode);
        }
      else if (sLayerName == "transport")
        {
          if (pTransport_ == layers_.end())
            {
              /* Invalid layer state */
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
                     <<"' layer NOT defined within 'nem' or 'platform' file."
                     <<std::endl
                     <<std::ends;
              
              throw ParseException(sstream.str());
            }

          (*pTransport_)->update(pNode);
        }
      else
        {
          //shim, can't pinpoint which one right now
        }
    }
  
}
