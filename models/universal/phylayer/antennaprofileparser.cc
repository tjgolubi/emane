/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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


#include "emaneutils/parameterconvert.h"
#include "emane/emaneconstants.h"
#include "emane/emaneantennamode.h"

#include "antennaprofilemanager.h"

#include <cmath>
#include <sstream>


namespace {
 const char * MODULE = "AntennaProfileParser";

 const xmlChar * toXmlChar(const char * s)
  { 
    return reinterpret_cast<const xmlChar*> (s); 
  }
}



UniversalPHY::AntennaProfileParser::AntennaProfileParser(EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService) :
   id_(id),
   pPlatformService_(pPlatformService)
{ }




void 
UniversalPHY::AntennaProfileParser::Open(const std::string & uri, xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument, xmlNode ** ppRoot)
      throw (EMANE::EMANEException)
  
{
  if((*ppContext = xmlNewParserCtxt()) == NULL)
   {
      std::stringstream ss;
      ss << "UniversalPHY::AntennaProfileParser::Open: Failed to allocate parser context" << std::ends;

      throw EMANE::EMANEException("UniversalPHY::AntennaProfileParser", ss.str());
   }
  else if((*ppDocument = xmlCtxtReadFile(*ppContext, uri.c_str(), NULL, XML_PARSE_DTDVALID)) == NULL)
   {
      std::stringstream ss;
      ss << "UniversalPHY::AntennaProfileParser::Open: Failed to parse document " << uri << std::ends;

      throw EMANE::EMANEException("UniversalPHY::AntennaProfileParser", ss.str());
   }
  else if((*ppContext)->valid == false)
    {          
      std::stringstream ss;
      ss << "UniversalPHY::AntennaProfileParser::Open: Document in " << uri << " is not valid" << std::ends;

      throw EMANE::EMANEException("UniversalPHY::AntennaProfileParser", ss.str());
    }
  else if((*ppRoot = xmlDocGetRootElement (*ppDocument)) == NULL) 
    {
      std::stringstream ss;
      ss << "UniversalPHY::AntennaProfileParser::Open: Could not get root element" << std::ends;

      throw EMANE::EMANEException("UniversalPHY::AntennaProfileParser", ss.str());
    }
}



void 
UniversalPHY::AntennaProfileParser::Close(xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument)
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


void
UniversalPHY::AntennaProfileParser::getProfileDescriptions (xmlNodePtr cur, ProfileDescriptionMap & map)
{
   while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("profile"))))
       {
         const ACE_UINT16 u16ProfileId = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("id"))).toUINT16();

         const std::string antenneGain  = getAttribute(cur, toXmlChar("antennapatternuri"));

         const std::string antennaBlockage = getAttribute(cur, toXmlChar("blockagepatternuri"));

         // optional placement
         int north = 0, east = 0, up = 0;

         getAntennaPlacement(cur->xmlChildrenNode, north, east, up);

         if(map.insert(std::make_pair(u16ProfileId, 
                                      UniversalPHY::AntennaProfileParser::ProfileDescription(u16ProfileId,          // profile id
                                                                                             antenneGain,           // antenna gain pattern
                                                                                             antennaBlockage,       // antenna blockage pattern
                                                                                             east,                  // placement easting  (x)
                                                                                             north,                 // placement northing (y)
                                                                                             up))).second == true)  // placement up       (z)
          {
            pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s, profile id %hu, pattern file %s, blockage file %s, north %d, east %d, up %d", 
                                   id_, MODULE, __func__, u16ProfileId, antenneGain.c_str(), antennaBlockage.c_str(), north, east, up);
          }
         else
          {
            pPlatformService_->log(EMANE::ERROR_LEVEL,"%03hu %s::%s, duplicate profile id %hu, pattern file %s, blockage file %s, ignore", 
                                   id_, MODULE, __func__, u16ProfileId, antenneGain.c_str(), antennaBlockage.c_str());
          }
      }

      cur = cur->next;
    }
}


UniversalPHY::AntennaProfilePattern::AntennaProfilePatternMap
UniversalPHY::AntennaProfileParser::getAntennaProfiles (const std::string & uri)
      throw (EMANE::EMANEException)
{
  xmlDoc  * doc  = NULL;
  xmlNode * root = NULL;
  xmlParserCtxtPtr pContext = NULL;

  if(uri.empty() == true)
   {
     std::stringstream ss;
     ss << "UniversalPHY::AntennaProfileParser::getAntennaProfiles: must supply antenna profile description uri path" << std::ends;

     throw EMANE::EMANEException("UniversalPHY::AntennaProfileParser", ss.str());
   }

  // open doc
  Open(uri, &pContext, &doc, &root);

  // root
  xmlNodePtr cur = root;

  // the profile descriptions
  UniversalPHY::AntennaProfileParser::ProfileDescriptionMap profileDescriptions;

  // get profile(s)
  while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("profiles")))) 
        {
          getProfileDescriptions(cur->xmlChildrenNode, profileDescriptions);
        }

      cur = cur->next;
    }

  // close doc
  Close(&pContext, &doc);

  // now create the antenna patterns
  UniversalPHY::AntennaProfilePattern::AntennaProfilePatternMap result;

  for(UniversalPHY::AntennaProfileParser::ProfileDescriptionMapIter iter = profileDescriptions.begin(); 
      iter != profileDescriptions.end(); ++iter)
   {
     if(iter->second.gainURI_.empty() == true)
      {
        std::stringstream ss;
        ss << "UniversalPHY::AntennaProfileParser::getAntennaProfiles: must supply antenna profile uri path" << std::ends;

        throw EMANE::EMANEException("UniversalPHY::AntennaProfileParser", ss.str());
      }
     else
      {
        UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap antennaGain;
        UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap antennaBlockage;

        Open(iter->second.gainURI_, &pContext, &doc, &root);

        // root
        xmlNodePtr cur = root;

        // get antenna profile
        while(cur) 
         {
           if((!xmlStrcmp (cur->name, toXmlChar("antennaprofile")))) 
            {
              getAntennaPattern(cur->xmlChildrenNode, antennaGain);
            }

           cur = cur->next;
         }

        // close doc
        Close(&pContext, &doc);

        // optional blockage
        if(iter->second.blockageURI_.empty() == false)
         {
           Open(iter->second.blockageURI_, &pContext, &doc, &root);

           // root
           xmlNodePtr cur = root;

           // get blockage profile
           while(cur) 
            {
              if((!xmlStrcmp (cur->name, toXmlChar("antennaprofile")))) 
               {
                 getBlockagePattern(cur->xmlChildrenNode, antennaBlockage);
               }

              cur = cur->next;
            }

           // close doc
           Close(&pContext, &doc);
         }
        // add antenna and blockage to result for this profile id
        result[iter->second.u16ProfileId_] = 
              UniversalPHY::AntennaProfilePattern(iter->second.u16ProfileId_, 
                                                  antennaGain, 
                                                  antennaBlockage,
                                                  EMANE::PositionNEU(iter->second.placementNorth_,
                                                                     iter->second.placementEast_,
                                                                     iter->second.placementUp_));
      }
   }

  return result;
}


void
UniversalPHY::AntennaProfileParser::getAntennaPattern(xmlNodePtr cur, UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap & map)
{
  while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("antennapattern")))) 
        {
          getElevation(cur->xmlChildrenNode, map);

          break;
        }

      cur = cur->next;
    }
}


void
UniversalPHY::AntennaProfileParser::getBlockagePattern(xmlNodePtr cur, UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap & map)
{
  while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("blockagepattern")))) 
        {
          getElevation(cur->xmlChildrenNode, map);

          break;
        }

      cur = cur->next;
    }
}


void
UniversalPHY::AntennaProfileParser::getAntennaPlacement(xmlNodePtr cur, int & north, int & east, int & up)
{
  while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("placement")))) 
        {
          north = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("north"))).toINT32();

          east  = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("east"))).toINT32();

          up    = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("up"))).toINT32();

          break;
        }

      cur = cur->next;
    }
}


void
UniversalPHY::AntennaProfileParser::getElevation(xmlNodePtr cur, UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap & map)
{
   while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("elevation"))))
       {
          const int min = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("min"))).toINT32();

          const int max = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("max"))).toINT32();

#ifdef VERBOSE_LOGGING
          pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s, min %d, max %d", 
                                 id_, MODULE, __func__, min, max);
#endif

          // add the bearing info
          map[UniversalPHY::AntennaProfilePattern::ArcRangePair (min, max)] = getBearing(cur->xmlChildrenNode);
      }

      cur = cur->next;
    }
}



UniversalPHY::AntennaProfilePattern::BearingGainMap
UniversalPHY::AntennaProfileParser::getBearing(xmlNodePtr cur)
{
  UniversalPHY::AntennaProfilePattern::BearingGainMap result;

   while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("bearing"))))
       {
          const int min = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("min"))).toINT32();

          const int max = EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("max"))).toINT32();
  
          const float fGain = (getGainValue(cur->xmlChildrenNode));

#ifdef VERBOSE_LOGGING
          pPlatformService_->log(EMANE::DEBUG_LEVEL,"%03hu %s::%s, min %d, max %d, gain %5.3f", 
                                 id_, MODULE, __func__, min, max, fGain);
#endif

          // add the gain for the range value 
          result[UniversalPHY::AntennaProfilePattern::ArcRangePair (min, max)] = fGain;
       }

      cur = cur->next;
    }

   return result;
}



float
UniversalPHY::AntennaProfileParser::getGainValue(xmlNodePtr cur)
{
   while(cur) 
    {
      if((!xmlStrcmp (cur->name, toXmlChar("gain"))))
       {
          return EMANEUtils::ParameterConvert(getAttribute(cur, toXmlChar("value"))).toFloat();
       }

      cur = cur->next;
    }

   return -EMANE::FULL_PATH_LOSS_DB;
}



std::string 
UniversalPHY::AntennaProfileParser::getAttribute (xmlNodePtr cur, const xmlChar * id)
{
  std::string str = "";

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
