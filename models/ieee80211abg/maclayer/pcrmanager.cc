/*
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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


IEEE80211ABG::PCRManager::PCRManager (EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService):
id_ (id), 
pPlatformService_ (pPlatformService), 
tablePacketSize_ (0)
{ }



IEEE80211ABG::PCRManager::~PCRManager ()
{ }


void
IEEE80211ABG::PCRManager::Open (const std::string & uri,
                                xmlParserCtxtPtr * ppContext,
                                xmlDoc ** ppDocument, xmlNode ** ppRoot)
{
  if ((*ppContext = xmlNewParserCtxt ()) == NULL)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::Open: Failed to allocate parser context" << std::ends;

      throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
    }

  else if ((*ppDocument = xmlCtxtReadFile (*ppContext, uri.c_str (), NULL, XML_PARSE_DTDVALID)) == NULL)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::Open: Failed to parse document " << uri << std::ends;

      throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
    }

  else if ((*ppContext)->valid == false)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::Open: Document in " << uri << " is not valid" << std::ends;

      throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
    }

  else if ((*ppRoot = xmlDocGetRootElement (*ppDocument)) == NULL)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::Open: Could not get root element" << std::ends;

      throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
    }
}


void
IEEE80211ABG::PCRManager::Close (xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument)
{
  if (ppContext)
    {
      xmlFreeParserCtxt (*ppContext);
      *ppContext = NULL;
    }

  if (ppDocument)
    {
      xmlFreeDoc (*ppDocument);
      *ppDocument = NULL;
    }

  xmlCleanupParser ();
}



void
IEEE80211ABG::PCRManager::load (const std::string & uri)
throw (EMANE::EMANEException)
{
  xmlDoc * doc = NULL;
  xmlNode * root = NULL;
  xmlParserCtxtPtr pContext = NULL;

  if (uri.empty ())
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::load: must supply curve file name" << std::ends;

      throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
    }

  // open doc
  Open (uri, &pContext, &doc, &root);

  // root
  xmlNodePtr cur = root;

  // clear pcr entries
  pcrPorMap_.clear ();

  // get table
  while (cur)
    {
      if ((!xmlStrcmp (cur->name, XMLCHAR "pcr")))
        {
          getTable (cur->xmlChildrenNode, pcrPorMap_);
        }

      cur = cur->next;
    }

  // close doc
  Close (&pContext, &doc);

  // need at least 1 point
  for (PCRPORMapIter iter = pcrPorMap_.begin (); iter != pcrPorMap_.end (); ++iter)
    {
      if (iter->second.pcr_.size () < 1)
        {
          std::stringstream excString;
          excString << "IEEE80211ABG::PCRManager::load: need at least 1 point to define a pcr curve " << std::ends;
          throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
        }
    }

  // fill in points
  interpolate ();
}



void
IEEE80211ABG::PCRManager::getTable (xmlNodePtr cur, IEEE80211ABG::PCRManager::PCRPORMap & map)
{
  while (cur)
    {
      if ((!xmlStrcmp (cur->name, XMLCHAR "table")))
        {
          // save table packet size
          tablePacketSize_ = EMANEUtils::ParameterConvert (getAttribute (cur, XMLCHAR "pktsize")).toUINT32 ();

          // get each data rate
          getDataRate (cur->xmlChildrenNode, map);
        }

      cur = cur->next;
    }
}


void
IEEE80211ABG::PCRManager::getDataRate (xmlNodePtr cur, IEEE80211ABG::PCRManager::PCRPORMap & map)
{
  while (cur)
    {
      if ((!xmlStrcmp (cur->name, XMLCHAR "datarate")))
        {
          // get data rate index
          const ACE_UINT16 u16DataRateIndex = EMANEUtils::ParameterConvert (getAttribute (cur, XMLCHAR "index")).toUINT16 ();

          // entry 
          IEEE80211ABG::PCRManager::PCRPOR entry;

          // get pcr each row 
          getRows (cur->xmlChildrenNode, entry.pcr_);

          if (map.insert (std::make_pair (u16DataRateIndex, entry)).second == false)
            {
              std::stringstream excString;
              excString << "IEEE80211ABG::PCRManager::getDataRate: duplicate datarate index value " << u16DataRateIndex << std::ends;
              throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
            }
        }

      cur = cur->next;
    }
}


void
IEEE80211ABG::PCRManager::getRows (xmlNodePtr cur, IEEE80211ABG::PCRManager::PCREntryVector & vec)
{
  while (cur)
    {
      if ((!xmlStrcmp (cur->name, XMLCHAR "row")))
        {
          // get sinr value
          const float
            fSINR = EMANEUtils::ParameterConvert (getAttribute (cur, XMLCHAR "sinr")).toFloat (-255.0, 255.0);

          // get por value, convert from percent to fraction
          const float
            fPOR = EMANEUtils::ParameterConvert (getAttribute (cur, XMLCHAR "por")).toFloat (0.0, 100.0) / 100.0;

          for (PCREntryVectorIter iter = vec.begin (); iter != vec.end (); ++iter)
            {
              if (fSINR == iter->fSINR_)
                {
                  std::stringstream excString;
                  excString << "IEEE80211ABG::PCRManager::getRows: duplicate sinr value " << fSINR << std::ends;
                  throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
                }
              else if (fSINR < iter->fSINR_)
                {
                  std::stringstream excString;
                  excString << "IEEE80211ABG::PCRManager::getRows: out of order sinr value, must be in increasing value " 
                            << fSINR << std::ends;
                  throw EMANE::EMANEException ("IEEE80211ABG::PCRManager", excString.str ());
                }
            }

          // append entry 
          vec.push_back (IEEE80211ABG::PCRManager::PCREntry (fSINR, fPOR));
        }

      cur = cur->next;
    }
}


void
IEEE80211ABG::PCRManager::interpolate ()
{
  // for each datarate
  for (PCRPORMapIter iter = pcrPorMap_.begin (); iter != pcrPorMap_.end (); ++iter)
    {
      // for each sinr/pcr entry
      for (size_t i = 0; i < (iter->second.pcr_.size () - 1); ++i)
        {
          // x1
          const SINRType x1 = iter->second.pcr_[i].fSINR_ * PRECISION_FACTOR;

          // y1
          const float y1 = iter->second.pcr_[i].fPOR_;

          // x2
          const SINRType x2 = iter->second.pcr_[i + 1].fSINR_ * PRECISION_FACTOR;

          // y2
          const float y2 = iter->second.pcr_[i + 1].fPOR_;

          // slope (m)
          const float slope = (y2 - y1) / (x2 - x1);

          // fill in between points
          for (SINRType dx = x1; dx < x2; ++dx)
            {
              // y = mx + b
              const float por = (dx - x1) * slope + y1;

              // save value to end of vector
              iter->second.por_.push_back (por);
            }
        }
    }
}




float
IEEE80211ABG::PCRManager::getPCR (float fSINR, size_t packetLen, ACE_UINT16 u16DataRateIndex)
{
  const PCRPORMapIter iter = pcrPorMap_.find (u16DataRateIndex);

  if (iter != pcrPorMap_.end ())
    {
      // check low end
      if (fSINR < iter->second.pcr_.front ().fSINR_)
        {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL,
                     "MACI %03hu PCRManager::%s: sinr %3.2f, for datarate index %hu, is below the low limit sinr of %3.2f",
                     id_, __func__, fSINR, u16DataRateIndex, iter->second.pcr_.front ().fSINR_);
#endif

          // full loss
          return 0.0;
        }
      // check high end
      else if (fSINR > iter->second.pcr_.back ().fSINR_)
        {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL,
                     "MACI %03hu PCRManager::%s: sinr %3.2f, for datarate index %hu, is above the high limit sinr of %3.2f",
                     id_, __func__, fSINR, u16DataRateIndex, iter->second.pcr_.back ().fSINR_);
#endif
          // no loss
          return 1.0;
        }
      // lookup
      else
        {
          float fPOR;

          // single point (front == back)
          if (fSINR == iter->second.pcr_.front ().fSINR_ && fSINR == iter->second.pcr_.back ().fSINR_)
            {
              // get por
              fPOR = iter->second.pcr_.front ().fPOR_;
            }
          else
            {
              // sinr adjusted
              const SINRType sinrScaled = fSINR * PRECISION_FACTOR;

              // sinr offset from the front
              const SINRType sinrOffset = iter->second.pcr_.front ().fSINR_ * PRECISION_FACTOR;

              // get the table index
              size_t idx = sinrScaled - sinrOffset;

              // cap max index, low end check covers min index
              if(idx >= iter->second.por_.size())
               {
                  idx = iter->second.por_.size() - 1;
               }

              // direct por lookup
              fPOR = iter->second.por_[idx];

#ifdef VERBOSE_LOGGING
              pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu PCRManager::%s: lookup por %3.2f at table index %zd",
                                     id_, __func__, fPOR, idx);
#endif
            }

          // adjust por for packet length
          if(tablePacketSize_ > 0)
           {
             fPOR = pow (fPOR, (float) packetLen / (float) tablePacketSize_);
           }

#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL,
                     "MACI %03hu PCRManager::%s: sinr %3.2f, for datarate index %hu, por %3.2f",
                     id_, __func__, fSINR, u16DataRateIndex, fPOR);
#endif

          // return por
          return fPOR;
        }
    }
  else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL,
                 "MACI %03hu PCRManager::%s: unsupported datarate index %hu, por %3.2f",
                 id_, __func__, u16DataRateIndex, 0.0);
#endif

      // full loss
      return 0.0;
    }
}




std::string IEEE80211ABG::PCRManager::getAttribute (xmlNodePtr cur, const xmlChar * id)
{
  std::string str = "0";

  if (id)
    {
      xmlChar * attr = xmlGetProp (cur, id);

      if (attr)
        {
          str = (const char *) attr;
        }

      xmlFree (attr);
    }

  return str;
}



std::string IEEE80211ABG::PCRManager::getContent (xmlNodePtr cur)
{
  std::string str = "0";

  xmlChar * attr = xmlNodeGetContent (cur);

  if (attr)
    {
      str = (const char *) attr;
    }

  xmlFree (attr);

  return str;
}
