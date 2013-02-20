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


#ifndef IEEE80211MAC_PCRMANAGER_HEADER_
#define IEEE80211MAC_PCRMANAGER_HEADER_

#include "emane/emanetypes.h"
#include "emane/emaneexception.h"
#include "emane/emaneplatformserviceprovider.h"

#include <libxml/parser.h>

#include <ace/Basic_Types.h>


#include <string>
#include <vector>
#include <map>

namespace IEEE80211ABG
{
/**
 *
 * @class PCRManager
 *
 * @brief provides access to the pcr curves
 *
 */
  class PCRManager
  {
  /**
   *
   * @struct PCREntry
   *
   * @brief proides a direct lookup mapping of packet completion to probability of reception
   *
   */
    struct PCREntry
    {
      float fSINR_;
      float fPOR_;

    /**
     *
     * default constructor
     *
     */
      PCREntry (): 
       fSINR_ (0), 
       fPOR_ (0)
       { } 

    /**
     *
     * initialized constructor
     *
     * @param fSinr the singnal to noise ratio
     * @param por  the probability of reception
     *
     */
      PCREntry (float fSinr, float por):
        fSINR_ (fSinr), 
        fPOR_ (por)
      { }
    }; 

    /**
     *
     * @brief vector of PCREntry
     *
     */
    typedef std::vector < PCREntry > PCREntryVector;
    typedef PCREntryVector::iterator PCREntryVectorIter;

    /**
     *
     * @brief vector of POR (probability of reception) values
     *
     */
    typedef std::vector < float >PORVector;
    typedef PORVector::iterator PORVectorIter;

   /**
    *
    * @struct PCRPOR
    *
    * @brief proides a direct lookup mapping of packet completion to probability of reception over a range of entries
    *
    */
    struct PCRPOR
    {
      PCREntryVector pcr_;
      PORVector      por_;
    };

    /**
     *
     * @brief map of data rate index to PCRPOR
     *
     */
    typedef std::map < ACE_UINT16, PCRPOR > PCRPORMap;
    typedef PCRPORMap::iterator PCRPORMapIter;

    /**
     *
     * @brief signal to noise ration data type
     *
     */
    typedef ACE_INT32 SINRType;


  public:
    /**
     * initialized constructor
     * 
     * @param id the NEM id
     *
     * @param pPlatformService a reference to the platform service(s)
     *
     */
    PCRManager (EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService);

    /**
     *  
     * destructor
     *
     */
    ~PCRManager ();

    /**
     *  
     * loads the pcr curve
     *
     * @param uri the location of the pcr curve file
     *
     */
    void load (const std::string & uri) throw (EMANE::EMANEException);

    /**
     *  
     * provides the pcr for a given sinr, packet size and data rate index
     *
     * @param fSinr the signal to noise ratio
     * @param size the packet size in bytes
     * @param DataRateIndex the data rate index
     *
     * @return retunrs the pcr
     *
     */
    float getPCR (float fSinr, size_t size, ACE_UINT16 DataRateIndex);

  private:
    void Open (const std::string & uri, xmlParserCtxtPtr * ppContext,
               xmlDoc ** ppDocument, xmlNode ** ppRoot);

    void Close (xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument);

    std::string getAttribute (xmlNodePtr cur, const xmlChar * id);

    std::string getContent (xmlNodePtr cur);

    void getTable (xmlNodePtr cur, IEEE80211ABG::PCRManager::PCRPORMap & map);

    void getDataRate (xmlNodePtr cur, IEEE80211ABG::PCRManager::PCRPORMap & map);

    void getRows (xmlNodePtr cur, IEEE80211ABG::PCRManager::PCREntryVector & vec);

    void interpolate ();

    const EMANE::NEMId id_;

    EMANE::PlatformServiceProvider * pPlatformService_;

    PCRPORMap pcrPorMap_;

    size_t tablePacketSize_;
  };
}


#endif //IEEE80211MAC_PCRMANAGER_HEADER_
