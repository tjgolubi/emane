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

#include "layerconfiguration.h"
#include "configurationparser.h"

#include <sstream>

#include <libxml/xmlstring.h>

EMANE::LayerConfiguration::LayerConfiguration(EMANE::ConfigurationParser *pParser)
  : pParser_(pParser)
{  }

EMANE::LayerConfiguration::~LayerConfiguration()
{ }

void EMANE::LayerConfiguration::processDefinition(const xmlChar *pxzName,
                                                  const std::string &sURI)
  throw(ParseException,ValidateException)
{
  /* Parse the definition */
  xmlDocPtr pDoc = pParser_->parse(sURI);

  /* Get root node */
  xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);

  /* Check if right root */
  if (!xmlStrEqual(pRoot->name, pxzName))
    {
      /* Invalid root node */
      std::stringstream sstream;
      
      sstream<<"Failed to parse document in "
             <<sURI
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<sURI
             <<": Document root node is not '"
             <<reinterpret_cast<const char*>(pxzName)
             <<"'"
             <<std::endl
             <<std::ends;

      /* Free the doc */
      xmlFreeDoc(pDoc);

      throw ParseException(sstream.str());
    }

  sDefinitionURI_ = sURI;

  /* Allow derived-class to extract info from root */
  doProcessRootNode(pRoot);

  /* Extract attributes we need */
  xmlChar *pxzLayerName = xmlGetProp(pRoot, 
                                     reinterpret_cast<const xmlChar*>("name"));

  if (pxzLayerName)
    {
      sLayerName_ = reinterpret_cast<const char*>(pxzLayerName);

      xmlFree(pxzLayerName);
    }
  
  /* Go through children and populate items/Layers */
  xmlNodePtr pNode = pRoot->children;

  while (pNode)
    {
      if (pNode->type == XML_ELEMENT_NODE)
        {
          if (xmlStrEqual(pNode->name, reinterpret_cast<const xmlChar*>("param")))
            {
              xmlChar *pxzName = xmlGetProp(pNode, 
                                            reinterpret_cast<const xmlChar*>("name"));
              xmlChar *pxzValue = xmlGetProp(pNode, 
                                             reinterpret_cast<const xmlChar*>("value"));

              if (pxzName && pxzValue)
                {
                  std::string sParamName(reinterpret_cast<const char*>(pxzName));

                  params_.insert(std::make_pair(sParamName, 
                                                EMANE::ConfigurationItem(sParamName,
                                                                         reinterpret_cast<const char*>(pxzValue),
                                                                         "", "")));
                }

              xmlFree(pxzName);
              xmlFree(pxzValue);
            }
          else //a node other than 'param' has been found - refer to derived class
            {
              doProcessChildNode(pNode);
            }

        }

      pNode = pNode->next;
    }

  /* Free the doc */
  xmlFreeDoc(pDoc);
}

void EMANE::LayerConfiguration::update(xmlNodePtr pRoot)
{
  /* Go through children and populate items/Layers */
  xmlNodePtr pNode = pRoot->children;

  ParamContainer localContainer;

  while (pNode)
    {
      if (pNode->type == XML_ELEMENT_NODE)
        {
          if (xmlStrEqual(pNode->name, reinterpret_cast<const xmlChar*>("param")))
            {
              xmlChar *pxzName = xmlGetProp(pNode, 
                                            reinterpret_cast<const xmlChar*>("name"));
              xmlChar *pxzValue = xmlGetProp(pNode, 
                                             reinterpret_cast<const xmlChar*>("value"));

              if (pxzName && pxzValue)
                {
                  std::string sItemName(reinterpret_cast<const char*>(pxzName));

                  std::string sItemValue(reinterpret_cast<const char*>(pxzValue));

                  localContainer.insert(std::make_pair(sItemName,
                                                       EMANE::ConfigurationItem(sItemName, 
                                                                                sItemValue, "", "")));

                  /*                  updateConfigurationItem(reinterpret_cast<const char*>(pxzName),
                                          reinterpret_cast<const char*>(pxzValue));
                  */
                }

              xmlFree(pxzName);
              xmlFree(pxzValue);
            }
          else //a node other than 'param' has been found - refer to derived class
            {
              doProcessChildNode(pNode);
            }

        }

      pNode = pNode->next;
    }

  /* If there are any local parameters, update */
  if (localContainer.size())
    {
      update(localContainer);
    }

}

void EMANE::LayerConfiguration::update(const ParamContainer &rParams)
{
  /* Iterate through incoming params and overwrite local values */
  ParamContainer::const_iterator constParamIter = rParams.begin();

  /* Ranges for parameter keys for both containers */
  std::pair<ParamContainer::const_iterator, ParamContainer::const_iterator> incomingRange;

  std::pair<ParamContainer::iterator, ParamContainer::iterator> currentRange;

  std::string sCurrentKey;

  for (; constParamIter != rParams.end(); ++constParamIter)
    {
      /* We need to do wholesale replacement on per-key basis. */
      if (constParamIter->first != sCurrentKey)
        {
          sCurrentKey = constParamIter->first;

          /* Ranges for the given key for both containers */
          incomingRange = rParams.equal_range(sCurrentKey);

          currentRange = params_.equal_range(sCurrentKey);

          /* Counts for the given key for current container */
          int iCurrentCount = params_.count(sCurrentKey);

          /* Iterators for the corresponding ranges */
          ParamContainer::const_iterator incomingIter = incomingRange.first;

          /* This one we'll modify */
          ParamContainer::iterator currentIter = currentRange.first;
          
          int iValueCount = 0;

          /* Notice incrementing 3 things in the for-loop, below: */
          for (; incomingIter != incomingRange.second; ++incomingIter, ++currentIter, ++iValueCount)
            {
              /* if incoming count (currently) is greater than the current count, insert */
              if (iValueCount >= iCurrentCount)
                {
                  params_.insert(std::make_pair(incomingIter->first, incomingIter->second));
                }
              else
                {
                  /* simply replace */
                  currentIter->second = incomingIter->second;
                }

            }// end for incoming iter
          
        }// end if new key

    }// end for constparamiter

}

void EMANE::LayerConfiguration::updateConfigurationItems()
{
  ParamContainer::iterator paramIter = params_.begin();

  items_.clear();

  for (; paramIter != params_.end(); ++paramIter)
    {
      items_.push_back(paramIter->second);
    }
}

EMANE::ConfigurationItems EMANE::LayerConfiguration::getConfigurationItems()
{
  /* Update configuration items */
  updateConfigurationItems();

  return items_;
}

EMANE::ConfigurationItems *EMANE::LayerConfiguration::getConfigurationItemsPtr()
{
  /* Update configuration items */
  updateConfigurationItems();

  return &items_;
}

std::string EMANE::LayerConfiguration::getName()
{
  //this is the default behavior (if derived class doesn't implement this func)
  return sLayerName_;
}

std::string EMANE::LayerConfiguration::getLibrary()
{
  //this is the default behavior (if derived class doesn't implement this func)
  return std::string();
}

std::string EMANE::LayerConfiguration::getDefinitionURI()
{
  return sDefinitionURI_;
}

