/*
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANECONFIGURATIONREQUIREMENT_HEADER_
#define EMANECONFIGURATIONREQUIREMENT_HEADER_

#include "emane/emaneconfigurationitem.h"

#include <map>

namespace EMANE
{
   /**
    *
    * @brief Container for a configuration definition entry.
    *
    */
  struct ConfigurationDefinition
  {
    const bool         bRequired_;
    const bool         bDefault_;
    const unsigned int uiCount_;
    const char *       pzName_;
    const char *       pzValue_;
    const char *       pzType_;
    const char *       pzDescription_;
  };

   /**
    *
    * @brief Container for a configuration requirement.
    *
    */
  struct ConfigurationRequirement
  {
    const bool bRequired_;
    bool bPresent_;
    bool bDefault_;
    unsigned int uiCount_;
    ConfigurationItem item_;


   /**
    *
    * @brief default container for a configuration requirement.
    *
    */

    ConfigurationRequirement():
      bRequired_(false),
      bPresent_(false),
      bDefault_(false),
      uiCount_(1){}

   /**
    *
    * @brief initialized container for a configuration requirement.
    *
    * @param bRequired the value is required
    * @param bPresent  the value is present
    * @param bDefault  the value is has a default value
    * @param uiCount   the num of times the value can occur
    * @param item      reference to configuration items
    */

    ConfigurationRequirement(bool bRequired,
                             bool bPresent,
                             bool bDefault,
                             unsigned int uiCount,
                             const ConfigurationItem & item):
      bRequired_(bRequired),
      bPresent_(bPresent),
      bDefault_(bDefault),
      uiCount_(uiCount),
      item_(item)
    {}

    
    ~ConfigurationRequirement(){};
  };
  
  typedef std::multimap<std::string, ConfigurationRequirement> ConfigurationRequirements;

  ConfigurationRequirements loadConfigurationRequirements(const ConfigurationDefinition defs[]);
}

#include "emane/emaneconfigurationrequirement.inl"

#endif // EMANECONFIGURATIONREQUIREMENT_HEADER_
