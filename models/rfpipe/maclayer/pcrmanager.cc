/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "pcrmanager.h"
#include "emaneutils/parameterconvert.h"

#include <libxml/parser.h>

#include <sstream>

#include <math.h>

#define  XMLCHAR (const xmlChar*)

namespace 
{
   const int PRECISION_FACTOR = 100;
}


RFPIPEMAC::PCRManager::PCRManager(EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService) :
 id_(id),
 pPlatformService_(pPlatformService),
 tablePacketSize_(0)
{ }



RFPIPEMAC::PCRManager::~PCRManager()
{ }


void 
RFPIPEMAC::PCRManager::Open(const std::string & uri, xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument, xmlNode ** ppRoot)
{
  if((*ppContext = xmlNewParserCtxt()) == NULL)
   {
      std::stringstream excString;
      excString << "RFPIPEMAC::PCRManager::Open: Failed to allocate parser context" << std::ends;

      throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
   }
  else if((*ppDocument = xmlCtxtReadFile(*ppContext, uri.c_str(), NULL, XML_PARSE_DTDVALID)) == NULL)
   {
      std::stringstream excString;
      excString << "RFPIPEMAC::PCRManager::Open: Failed to parse document " << uri << std::ends;

      throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
   }
  else if((*ppContext)->valid == false)
    {          
      std::stringstream excString;
      excString << "RFPIPEMAC::PCRManager::Open: Document in " << uri << " is not valid" << std::ends;

      throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
    }
  else if((*ppRoot = xmlDocGetRootElement (*ppDocument)) == NULL) 
    {
      std::stringstream excString;
      excString << "RFPIPEMAC::PCRManager::Open: Could not get root element" << std::ends;

      throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
    }
}


void 
RFPIPEMAC::PCRManager::Close(xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument)
{
  if(ppContext)
    {
      xmlFreeParserCtxt(*ppContext);      
      *ppContext = NULL;
    }

  if(ppDocument) 
    {
      xmlFreeDoc (*ppDocument);
      *ppDocument = NULL;
    }

  xmlCleanupParser ();
}



void RFPIPEMAC::PCRManager::load (const std::string & uri)
      throw (EMANE::EMANEException)
{
  xmlDoc  * doc  = NULL;
  xmlNode * root = NULL;
  xmlParserCtxtPtr pContext = NULL;

  if(uri.empty())
   {
      std::stringstream excString;
      excString << "RFPIPEMAC::PCRManager::load: must supply curve file name" << std::ends;

      throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
   }

  // open doc
  Open(uri, &pContext, &doc, &root);

  // root
  xmlNodePtr cur = root;

  // clear pcr entries
  pcrEntryVector_.clear();

  // clear por entries
  porVector_.clear();

  // get table
  while(cur) 
    {
      if((!xmlStrcmp (cur->name, XMLCHAR "pcr"))) 
        {
          getTable(cur->xmlChildrenNode);
        }

      cur = cur->next;
    }

  // close doc
  Close(&pContext, &doc);

  // need at least 1 point
  if(pcrEntryVector_.size() < 1)
   {
     std::stringstream excString;
     excString << "RFPIPEMAC::PCRManager::getRows: need at least 1 point to define a pcr curve " << std::ends;

     throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
   }

  // fill in points
  interpolate();
}



void
RFPIPEMAC::PCRManager::getTable(xmlNodePtr cur)
{
   while(cur) 
    {
      if((!xmlStrcmp (cur->name, XMLCHAR "table")))
       {
         tablePacketSize_ = EMANEUtils::ParameterConvert(getAttribute(cur, XMLCHAR  "pktsize")).toUINT32();

         getRows(cur->xmlChildrenNode);
       }

      cur = cur->next;
    }
}



void
RFPIPEMAC::PCRManager::getRows(xmlNodePtr cur)
{
   while(cur) 
    {
      if((!xmlStrcmp (cur->name, XMLCHAR "row"))) 
        {
          // get sinr value
          const float fSINR = EMANEUtils::ParameterConvert(getAttribute(cur, XMLCHAR "sinr")).toFloat(-255.0, 255.0);

          // get por value, convert from percent to fraction
          const float fPOR  = EMANEUtils::ParameterConvert(getAttribute(cur, XMLCHAR "por")).toFloat(0.0, 100.0) / 100.0;

          for(PCREntryVectorIter iter = pcrEntryVector_.begin(); iter != pcrEntryVector_.end(); ++iter)
           {
             if(fSINR == iter->fSINR_)
              {
                std::stringstream excString;
                excString << "RFPIPEMAC::PCRManager::getRows: duplicate sinr value " << fSINR << std::ends;
                throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
              }
             else if(fSINR < iter->fSINR_)
              {
                std::stringstream excString;
                excString << "RFPIPEMAC::PCRManager::getRows: out of order sinr value, must be in increasing value " << fSINR << std::ends;
                throw EMANE::EMANEException("RFPIPEMAC::PCRManager", excString.str());
              }
           }
          
          pcrEntryVector_.push_back(RFPIPEMAC::PCRManager::PCREntry(fSINR, fPOR));
        }
 
       cur = cur->next;
    }
}


void 
RFPIPEMAC::PCRManager::interpolate()
{
   // for each sinr/pcr entry
   for(size_t i = 0; i < (pcrEntryVector_.size() - 1); ++i)
    {
      // x1
      const SINRType x1 = pcrEntryVector_[i].fSINR_ * PRECISION_FACTOR;

      // y1
      const float y1 = pcrEntryVector_[i].fPOR_;

      // x2
      const SINRType x2 = pcrEntryVector_[i + 1].fSINR_ * PRECISION_FACTOR;

      // y2
      const float y2 = pcrEntryVector_[i + 1].fPOR_;

      // slope
      const float slope = (y2 - y1) / (x2 - x1);

      // fill in between points
      for(SINRType dx = x1; dx < x2; ++dx)
       {
         // y = mx + b
         const float por = (dx - x1) * slope + y1;

#ifdef EMANE_DEBUG
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu PCRManager::%s: dx %d, por %3.2f",
                  id_, __func__, dx, por);
#endif

         // save value to end of vector
         porVector_.push_back(por);
       }
    }
}


float 
RFPIPEMAC::PCRManager::getPCR(float fSINR, size_t packetLen)
{
  // check low end
  if(fSINR < pcrEntryVector_.front().fSINR_)
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu PCRManager::%s: sinr %3.2f is below low limit of %3.2f",
                id_, __func__, fSINR, pcrEntryVector_.front().fSINR_);
#endif

      // full loss
      return 0.0; 
    }
  // check high end
  else if(fSINR > pcrEntryVector_.back().fSINR_)
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu PCRManager::%s: sinr %3.2f is above high limit of %3.2f",
                id_, __func__, fSINR, pcrEntryVector_.back().fSINR_);
#endif

      // no loss
      return 1.0;
    }
  // lookup
  else
    {
      // por value
      float fPOR;

      // single point
      if(fSINR == pcrEntryVector_.front().fSINR_ && fSINR == pcrEntryVector_.back().fSINR_)
       {
         fPOR = pcrEntryVector_.front().fPOR_;
       }
      else
       {
         // sinr adjusted
         const SINRType sinrScaled = fSINR * PRECISION_FACTOR;

         // sinr offset from the front
         const SINRType sinrOffset = pcrEntryVector_.front().fSINR_ * PRECISION_FACTOR;

         // get the table index
         size_t idx = sinrScaled - sinrOffset;

         // cap max index, low end check covers min index
         if(idx >= porVector_.size())
          {
            idx = porVector_.size() - 1;
          }

         // direct lookup
         fPOR = porVector_[idx];

#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu PCRManager::%s: lookup por %3.2f at table index %zd",
                                id_, __func__, fPOR, idx);
#endif
       }

      // adjust for pkt size is given in the table
      if(tablePacketSize_ > 0)
       {
         fPOR = pow (fPOR, (float) packetLen / (float) tablePacketSize_);
       }

#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu PCRManager::%s: sinr %3.2f, len %zu,  por %3.2f",
                              id_, __func__, fSINR, packetLen, fPOR);
#endif

       return fPOR;
    }
}




std::string 
RFPIPEMAC::PCRManager::getAttribute (xmlNodePtr cur, const xmlChar * id)
{
  std::string str = "0";

  if(id) 
    {
      xmlChar *attr = xmlGetProp (cur, id);

      if(attr) 
        {
          str = (const char *) attr;
        }
       xmlFree (attr);
    }

  return str;
}



std::string 
RFPIPEMAC::PCRManager::getContent (xmlNodePtr cur)
{
  std::string str = "0";

  xmlChar *attr = xmlNodeGetContent (cur);

  if(attr) 
    {
      str = (const char *) attr;
    }
   xmlFree (attr);

  return str;
}
