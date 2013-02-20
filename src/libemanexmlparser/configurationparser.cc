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

#include "configurationparser.h"

#include <iostream>
#include <sstream>

namespace 
{
  /* DTD validation options */
  const int DEFAULT_PARSER_OPTIONS = XML_PARSE_NOENT |    //sub entities
                                     XML_PARSE_DTDLOAD |  //load ext subset
                                     XML_PARSE_DTDATTR |  //default attr vals
                                     XML_PARSE_DTDVALID;  //validate w/dtd
}

EMANE::ConfigurationParser::ConfigurationParser()
  : pContext_(xmlNewParserCtxt())
{
  xmlInitParser();
}

EMANE::ConfigurationParser::~ConfigurationParser()
{
  if (pContext_)
    {
      xmlFreeParserCtxt(pContext_);
    }

  xmlCleanupParser();
}

xmlDocPtr EMANE::ConfigurationParser::parse(const std::string &sURI)
  throw (ParseException, ValidateException)
{
  /* Check the context first */
  if (!pContext_)
    {
      throw ParseException("Parser initialized incorrectly (Context is NULL)");
    }

  /* Eval location and name */
  size_t pos = sURI.npos;

  if ((pos = sURI.rfind("/")) != sURI.npos) 
    {
      /* *nix */
      sURL_ = sURI.substr(0, pos+1); //includes the '/'    
      sURN_ = sURI.substr(pos+1); //to the end
    }
  else if ( (pos = sURI.rfind("\\")) != sURI.npos) 
    {
      /* windoze */
      sURL_ = sURI.substr(0, pos+1); //includes the '\'    
      sURN_ = sURI.substr(pos+1); //to the end
    }
  else 
    {
      /* keep URL the same and set sURI_ to passed-in value */
      sURN_ = sURI;
    }

  /* Use complete URI */
  std::string sFullURI = sURL_ + sURN_;
  
  /* Allocate document */
  xmlDocPtr pDoc = xmlCtxtReadFile(pContext_,
                                   sFullURI.c_str(),
                                   0,
                                   DEFAULT_PARSER_OPTIONS);

  if (!pDoc)
    {
      /* Failed to parse, update the message */
      std::stringstream sstream;
      
      sstream<<"Failed to parse document in '"
             <<sFullURI
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<sFullURI
             <<" is not located at '"
             <<sURL_
             <<"'"
             <<std::endl
             <<std::ends;

      throw ParseException(sstream.str());
    }

  /* Parse was successful, check if valid */
  if (pContext_->valid == 0)
    {

      xmlDtdPtr pDTD = xmlGetIntSubset(pDoc);
      
      /* Data failed to validate */
       std::stringstream sstream;

      sstream<<"Failed to validate document in '"
             <<sFullURI
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<(pDTD ? (const char *) pDTD->SystemID : "DTD")
             <<" is missing."
             <<std::endl
             <<" * "
             <<(pDTD ? (const char *) pDTD->SystemID : "DTD")
             <<" is inaccessible (network unreachable/http down)."
             <<std::endl
             <<" * "
             <<(pDTD ? (const char *) pDTD->SystemID : "DTD")
             <<" is incorrect (run xmllint to validate)."
             <<std::endl
             <<std::ends;

      xmlFreeDoc(pDoc);

      throw ValidateException(sstream.str());
    }
  
  /* Everything checks out, return the document */
  return pDoc;
}

const std::string &EMANE::ConfigurationParser::getURN()
{
  return sURN_;
}
