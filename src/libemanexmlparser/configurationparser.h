/*
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef CONFIGURATIONPARSER_HEADER_
#define CONFIGURATIONPARSER_HEADER_

#include "emaneparseexception.h"

#include <string>

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>

namespace EMANE
{
  /**
   * @class ConfigurationParser
   *
   * @brief Wrapper around the libxml2 XML parsing capabilities for EMANE
   */
  class ConfigurationParser
  {
  public:
    /**
     * Default constructor
     */
    ConfigurationParser();

    /**
     * Destructor
     */
    virtual ~ConfigurationParser();

    /**
     * Parses the specified URI
     *
     * @param sURI URI to be parsed
     *
     * @return pDoc Pointer to the parsed document, NULL on failure.
     *              It is up to the caller to free the document.
     *
     * @exception ParseException ValidateException
     */
    virtual xmlDocPtr parse(const std::string &sURI)
      throw (ParseException, ValidateException);

    /**
     * Returns the URN of the document currently being parsed
     *
     * @return sURN_ The URN of document currently being parsed
     */
    const std::string &getURN();

  protected:
    
  private:
    /**
     * Uniform Resource Name of the document currently being parsed
     */
    std::string sURN_;

    /**
     * Uniform Resource Locat(or|ion) of the document currently being parsed
     */
    std::string sURL_;

    /**
     * Parser context for libxml2
     */
    xmlParserCtxtPtr pContext_;

  };

  typedef ACE_Singleton<ConfigurationParser,ACE_Null_Mutex> ConfigurationParserSingleton; 
} /* end namespace EMANE */

#endif /* CONFIGURATIONPARSER_HEADER_ */
