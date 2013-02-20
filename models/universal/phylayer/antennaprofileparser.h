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

#ifndef UNIVERSALPHY_ANTENNAPROFILEPARSER_HEADER_
#define UNIVERSALPHY_ANTENNAPROFILEPARSER_HEADER_

#include "emane/emanetypes.h"
#include "emane/emaneexception.h"
#include "emane/emaneplatformserviceprovider.h"

#include "antennaprofilepattern.h"

#include <string>
#include <map>

#include <ace/Basic_Types.h>

#include <libxml/parser.h>

namespace UniversalPHY {

  /**
   *
   * @class  AntennaProfileParser 
   *
   * @brief Parses antenna profile pattern files
   *
   */
  class AntennaProfileParser {
    public: 

    /**
     * @brief constructor
     *
     * @param id the NEM id of this instance
     * @param pPlatformService pointer to the platform servicd
     *
     */
     AntennaProfileParser(EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService);

    /**
     *
     * @param uri           antenna profile uri
     *
     * @exception throws EMANEException
     *
     */
     UniversalPHY::AntennaProfilePattern::AntennaProfilePatternMap getAntennaProfiles(const std::string &uri)
        throw (EMANE::EMANEException);

    private:
      /**
       *
       * @struct ProfileDescription
       *
       * @brief describes the antenna profile attributes
       *
       */
      struct ProfileDescription {
        ACE_UINT16   u16ProfileId_;      // the profile id
        std::string  gainURI_;           // the antenna gain uri
        std::string  blockageURI_;       // the antenna blockage uri
        int          placementEast_;     // the antenna placement east in meters
        int          placementNorth_;    // the antenna placement north in meters
        int          placementUp_;       // the antenna placement up in meters

        /**
         *
         * @brief default constructor
         *
         */
        ProfileDescription () :
          u16ProfileId_(0),
          gainURI_(""),
          blockageURI_(""),
          placementEast_(0),
          placementNorth_(0),
          placementUp_(0)
         { }

        /**
         *
         * @brief parameter constructor
         *
         * @param u16ProfileId the antenna profile id
         * @param gainURI      the antenna gain uri
         * @param blockageURI  the antenna blockage uri
         * @param east         the antenna placement east in meters
         * @param north        the antenna placement up in meters
         * @param up           the antenna placement up in meters
         *
         */
        ProfileDescription (const ACE_UINT16 u16ProfileId, 
                            const std::string & gainURI, 
                            const std::string & blockageURI,
                            const int east,
                            const int north,
                            const int up) :
          u16ProfileId_(u16ProfileId),
          gainURI_(gainURI),
          blockageURI_(blockageURI),
          placementEast_(east),
          placementNorth_(north),
          placementUp_(up)
         { }
      };

      // profile description mapping and iterators
      typedef std::map<ACE_UINT16, ProfileDescription>  ProfileDescriptionMap;
      typedef ProfileDescriptionMap::iterator           ProfileDescriptionMapIter;
      typedef ProfileDescriptionMap::const_iterator     ProfileDescriptionMapConstIter;
   
      void Open(const std::string & uri, xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument, xmlNode ** ppRoot)
        throw (EMANE::EMANEException);

      void Close(xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument);

      std::string getAttribute (xmlNodePtr cur, const xmlChar * id);

      void getAntennaPattern (xmlNodePtr cur, UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap & map);

      void getAntennaPlacement (xmlNodePtr cur, int & north, int & east, int & up);

      void getBlockagePattern(xmlNodePtr cur, UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap & map);

      void getProfileDescriptions (xmlNodePtr cur, ProfileDescriptionMap & map);

      void getElevation (xmlNodePtr cur, UniversalPHY::AntennaProfilePattern::ElevationBearingGainMap & map);

      UniversalPHY::AntennaProfilePattern::BearingGainMap getBearing (xmlNodePtr cur);

      float getGainValue(xmlNodePtr cur);

      std::string getString(xmlNodePtr cur, const std::string & name);

      EMANE::NEMId id_;

      EMANE::PlatformServiceProvider * pPlatformService_;
   };
}
#endif //UNIVERSALPHY_ANTENNAPROFILEPARSER_HEADER_
