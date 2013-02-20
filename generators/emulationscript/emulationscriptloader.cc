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

#include "emulationscriptloader.h"
#include "emane/emanecomponenttypes.h"
#include "emane/emaneconstants.h"

namespace
{
  /**
   * Casts a const char* value into a const xmlChar * value
   *
   * @param pczData Pointer to a const char 
   *
   * @return xchar Pointer to a const xmlChar
   */
  xmlChar *toXMLChar(const char *pczData)
  {
    return reinterpret_cast<xmlChar*>(const_cast<char*>(pczData));
  }
  
  /**
   * Casts a const xmlChar* value into a const char * value
   *
   * @param xchar Pointer to a const xmlChar
   *
   * @return  pczData Pointer to a const char 
   */
  const char *toConstChar(const xmlChar *xchar)
  {
    return reinterpret_cast<const char*>(xchar);
  }

  /**
   * Converts an NEM id to an array index
   *
   * @return u16Index The array index corresponding to the NEM Id
   */
  ACE_UINT16 toArrayIndex(ACE_UINT16 u16NEMId)
  {
    return u16NEMId - 1;
  }
} //end anonymous namespace

/* Class definition */
EmulationScriptLoader::EmulationScriptLoader()
  : PlatformServiceUser(0),
    pDocument_(0),
    pXPathContext_(0),
    pCurrentEvent_(0),
    bLastEventReached_(false),
    u16RepeatCount_(1),
    lastTimePeriod_(0),
    thread_(0),
    bError_(false),
    bCancel_(false),
    eventBuffer_(60)

{
  /* No default functionality, see the open() and load() functions */ 
}

EmulationScriptLoader::~EmulationScriptLoader()
{
  if(thread_)
    {
      ACE_OS::thr_cancel(thread_);
      ACE_OS::thr_join(thread_,0,0);
    }

  /* Cleanup the document and XPath pointers (our own private function) */
  xmlCleanupDocument();

  /* Cleanup the rest of the parser */
  xmlCleanupParser();
}

void EmulationScriptLoader::open(const FileVectorType &fileNames, 
                                 const std::string &sSchemaName, ACE_UINT16 u16RepCnt)
{
  files_          = fileNames;
  sSchemaFile_    = sSchemaName;
  u16RepeatCount_ = (u16RepCnt == 0) ? ACE_UINT16_MAX : u16RepCnt;

  /* Spawn the thread */
  EMANEUtils::spawn(*this, &EmulationScriptLoader::svc, &thread_);
}

const char *EmulationScriptLoader::getStartTime()
{
  xmlXPathObjectPtr pStartTime = getNodesByXPath(toXMLChar("//startTime"));

  if (pStartTime == 0)
    {
      /* Returning NULL */
      return 0;
    }
  
  /* Check how many elements were returned */
  xmlNodeSetPtr pNodes = pStartTime->nodesetval;

  if (pNodes->nodeNr != 1)
    {
      /* There should be only 1 start time */
      /* Clean up the xpath expression return value */
      xmlXPathFreeObject(pStartTime);

      /* Returning failure */
      return 0;
    }

  /* Taken care of all the errors, get actual value */
  xmlChar *time = xmlNodeListGetString(pDocument_, 
                                       pNodes->nodeTab[0]->xmlChildrenNode,
                                       1);

  if (time == 0)
    {
      /* This failure should have been taken care of with validation */

      /* Clean up the xpath expression return value */
      xmlXPathFreeObject(pStartTime);

      return 0;
    }

  /* Clean up the xpath expression return value */
  xmlXPathFreeObject(pStartTime);

  /* The caller MUST deallocate this one */
  return toConstChar(time);
}

void EmulationScriptLoader::fillEventContainer(EventContainerType &cont, 
                                               bool bUseXPath)
{
  /* Only use xpath if requested to. Might be faster, but out of order */
  if (bUseXPath)
    {
      xmlXPathObjectPtr pEvents = getNodesByXPath(toXMLChar("//Event"));
      
      if (pEvents == 0)
        {
          /* No 'Event' tags found */
          return;
        }
      
      /* Check how many elements were returned */
      xmlNodeSetPtr pNodes = pEvents->nodesetval;
      
      int iEventCount = pNodes->nodeNr;
      
      for (int i = 0; i < iEventCount; ++i)
        {
          /* Parse one event at a time */
          cont.insert(cont.end(),
                      parseEventElement(pNodes->nodeTab[i]));
        }
      
      /* Cleanup the events XPath*/
      xmlXPathFreeObject(pEvents);
    }
  else
    {
      /* No XPath, hammer it down */
      Event *pEvent = getNextEvent();
      while (pEvent != 0)
        {
          /* Parse one event at a time */
          cont.insert(cont.end(), pEvent);
          
          pEvent = getNextEvent();
        }
    }

}

EmulationScriptLoader::LocationListType EmulationScriptLoader::getNextLocations(ACE_Time_Value &sleepTime)
{
  mutex_.acquire();

  if (bError_ || bCancel_)
    {
      mutex_.release();

      return LocationListType();
    }

  mutex_.release();

  /* Location event list */
  LocationListType locationList;

  /* Get new event */
  Event *pEvent = eventBuffer_.consume();

  if (pEvent)
    {
      /* Go through the event and populate location list */
      const Event::NodeContainerType &rNodes = pEvent->getNodeContainer();
      Event::NodeContainerType::const_iterator nodeIter;

      if ((sleepTime = pEvent->getTime() - lastTimePeriod_) < ACE_Time_Value::zero)
        {
          sleepTime = ACE_Time_Value::zero;
        }
      
      /* Update last time period */
      lastTimePeriod_ = pEvent->getTime();

      LocationEvent::LocationEntry entry;

      for (nodeIter = rNodes.begin(); 
           nodeIter != rNodes.end(); ++nodeIter)
        {
          /* Get the node id */
          ACE_UINT16 u16NEM  = ACE_OS::atoi((*nodeIter)->getNodeId());
         
          entry.u16Node_ = u16NEM;
          entry.i32LatitudeMARCS_  = static_cast<ACE_INT32>((*nodeIter)->getLatitude()  * EMANE::MILLI_ARC_SECONDS_PER_DEGREE);
          entry.i32LongitudeMARCS_ = static_cast<ACE_INT32>((*nodeIter)->getLongitude() * EMANE::MILLI_ARC_SECONDS_PER_DEGREE);
          entry.i32YawMARCS_       = 0;  // TODO
          entry.i32PitchMARCS_     = 0;  // TODO
          entry.i32RollMARCS_      = 0;  // TODO

          entry.i32AltitudeMeters_  = static_cast<ACE_INT32>((*nodeIter)->getAltitude());

          locationList.push_back(entry);

          memset(&entry, 0, sizeof(LocationEvent::LocationEntry));
        }
      
      /* Clean this event */
      delete(pEvent);
    }
  else
    {
      sleepTime.set(0.0);
    }

  /* Return the stuff */
  return locationList;
}

Event *EmulationScriptLoader::getNextEvent()
{
  /* To prevent undesired looping */
  if (bLastEventReached_)
    {
      /* pCurrentEvent_ should be null, as well */
      return 0;
    }

  /* If current event pointer is null, we need to get the first one */
  if (pCurrentEvent_ == 0)
    {
      /* Get root element */
      xmlNodePtr pRootNode = xmlDocGetRootElement(pDocument_);

      /* Assign current event to kids, so that looping can begin */
      pCurrentEvent_ = pRootNode->xmlChildrenNode;

      /* Loop through nodes until the first Event element is hit */
      while ( (pCurrentEvent_ != 0) && 
              (xmlStrcmp(pCurrentEvent_->name, toXMLChar("Event"))) )
        {
          pCurrentEvent_ = pCurrentEvent_->next;
        }

      /* Now pCurrentEvent_ should be pointing to a valid Event element */
    }
  else
    {
      /* Advance pCurrentEvent_ to leave the last element */
      pCurrentEvent_ = pCurrentEvent_->next;

      /* Advance pCurrentEvent_ until next Event element is hit or NULL */
      while ( (pCurrentEvent_ != 0) && 
              (xmlStrcmp(pCurrentEvent_->name, toXMLChar("Event"))) )
        {
          pCurrentEvent_ = pCurrentEvent_->next;
        }
    }

  /* If pCurrentEvent_ is NOT null, process, otherwise set flag and return */
  if (pCurrentEvent_ == 0)
    {
      bLastEventReached_ = true;
      return 0;
    }
  
  /* All's well, process and return new event */
  /* Caller MUST DEALLOCATE memory */
  return parseEventElement(pCurrentEvent_);
}

void EmulationScriptLoader::resetLastEventReached()
{
  bLastEventReached_ = false;
}

/* --  Member functions below are private -- */

int EmulationScriptLoader::isDocumentValid(const char *pczSchemaFilename)
{
    xmlDocPtr pSchemaDoc = xmlReadFile(pczSchemaFilename, NULL, XML_PARSE_NODICT);
    if (pSchemaDoc == NULL) 
      {
        /* the schema cannot be loaded or is not well-formed */
        return SCHEMA_LOAD_FAILED;
      }

    xmlSchemaParserCtxtPtr pSchemaParseCtxt = xmlSchemaNewDocParserCtxt(pSchemaDoc);
    if (pSchemaParseCtxt == NULL) 
      {
        /* unable to create a parser context for the schema */
        xmlFreeDoc(pSchemaDoc);
        return SCHEMA_CTXT_FAILED;
      }

    xmlSchemaPtr schema = xmlSchemaParse(pSchemaParseCtxt);
    if (schema == NULL) 
      {
        /* the schema itself is not valid */
        xmlSchemaFreeParserCtxt(pSchemaParseCtxt);
        xmlFreeDoc(pSchemaDoc);
        return SCHEMA_INVALID;
      }

    xmlSchemaValidCtxtPtr pSchemaCtxt = xmlSchemaNewValidCtxt(schema);
    if (pSchemaCtxt == NULL) 
      {
        /* unable to create a validation context for the schema */
        xmlSchemaFree(schema);
        xmlSchemaFreeParserCtxt(pSchemaParseCtxt);
        xmlFreeDoc(pSchemaDoc);
        return SCHEMA_CTXT_INVALID; 
      }

    int iValid = (xmlSchemaValidateDoc(pSchemaCtxt, pDocument_) == 0);

    xmlSchemaFreeValidCtxt(pSchemaCtxt);
    xmlSchemaFree(schema);
    xmlSchemaFreeParserCtxt(pSchemaParseCtxt);
    xmlFreeDoc(pSchemaDoc);

    /* force the return value to be non-negative on success */
    return iValid ? 1 : 0;
}

xmlXPathObjectPtr EmulationScriptLoader::getNodesByXPath(const xmlChar *xpath)
{
  /* Resulting node set */
  xmlXPathObjectPtr pResult = 0;

  if (pXPathContext_ == 0)
    {
      /* Return default value (NULL) */
      return pResult; 
    }

  /* Actually evaluate the expression */
  pResult = xmlXPathEvalExpression(xpath, pXPathContext_);

  if (pResult == 0)
    {
      /* Failed on XPath expression evaluation */
      
      /* Return the value (NULL) */
      return pResult;
    }

  /* Internal check to return NULL if resulting set is empty */
  if (xmlXPathNodeSetIsEmpty(pResult->nodesetval))
    {
      /* Resulting XPath set is empty */

      xmlXPathFreeObject(pResult);
      
      /* Returning NULL */
      return 0;
    }

  /* At this point result should be valid */
  return pResult;
}

Node *EmulationScriptLoader::parseNodeElement(xmlNodePtr node)
{
  /* Get the node id */
  xmlChar *idVal = xmlGetProp(node, toXMLChar("id"));

  xmlChar *durationVal = 0;
  xmlChar *locationVal = 0;

  /* Set the current node to the first child */
  xmlNodePtr pCurrNode = node->xmlChildrenNode;
  
  while (pCurrNode != 0)
    {
      if (!xmlStrcmp(pCurrNode->name, toXMLChar("location")))
        {
          /* Get location's content */
          locationVal = xmlNodeListGetString(pDocument_,
                                             pCurrNode->xmlChildrenNode,
                                             1);
          
          durationVal = xmlGetProp(pCurrNode, toXMLChar("duration"));
        }
      else
        {
          /* Disregard all other elements */
        }
      
      pCurrNode = pCurrNode->next;
    }

  /* Allocate new Node object */
  Node *pNode = new Node(toConstChar(idVal), toConstChar(locationVal));

  if (durationVal != 0)
    {
      xmlFree(durationVal);
    }

  xmlFree(idVal);
  xmlFree(locationVal);

  return pNode;
}

Event *EmulationScriptLoader::parseEventElement(xmlNodePtr node)
{
  /* Set the current node to the first child */
  xmlNodePtr pCurrNode = node->xmlChildrenNode;
  Event *pEvent        = 0;

  while (pCurrNode != 0)
    {
      if (!xmlStrcmp(pCurrNode->name, toXMLChar("time")))
        {
          /* Don't forget the time type (secs|duration|ass) */
          /* Get time's content */
          xmlChar *timeVal = xmlNodeListGetString(pDocument_,
                                                  pCurrNode->xmlChildrenNode,
                                                  1);
          
          pEvent = new Event(toConstChar(timeVal));

          xmlFree(timeVal);
        }
      else if (!xmlStrcmp(pCurrNode->name, toXMLChar("Node")))
        {
          if (pEvent != 0)
            {
              /* Parse the 'Node' element */
              pEvent->addNode(parseNodeElement(pCurrNode));
            }
        }
      else
        {
          /* Disregard all other elements */
        }
      
      pCurrNode = pCurrNode->next;
    }
  
  return pEvent;
}

int EmulationScriptLoader::load(const char *pczFilename)
{
  /* Cleanup first, to make sure we're good */
  xmlCleanupDocument();
  resetLastEventReached();

  /* Parse the specific file */
  pDocument_ = xmlParseFile(pczFilename);

  if (pDocument_ == 0)
    {
      /* Failed to parse the document */
      return DOCUMENT_PARSE_FAILED;
    }

  /* Parsing succeded, check if valid */
  int iValid = isDocumentValid(sSchemaFile_.c_str());

  if (iValid != 1)
    {
      /* Failed to validate the document, cleanup and return */
      xmlCleanupDocument();
      return iValid;
    }

  /* Parsing and validation succeded, allocating xpath context */
  pXPathContext_ = xmlXPathNewContext(pDocument_);
  
  if (pXPathContext_ == 0)
    {
      /* Failed to allocate xpath context, cleanup and return */
      xmlCleanupDocument();
      return XPATH_CONTEXT_FAILED;
    }

  return DOCUMENT_PARSE_SUCCESS;
}

void EmulationScriptLoader::xmlCleanupDocument()
{
  /* Free the document pointer */
  if (pDocument_ != 0)
    {
      xmlFreeDoc(pDocument_);
      pDocument_ = 0;
    }

  /* Free the context */
  if (pXPathContext_ != 0)
    {
      xmlXPathFreeContext(pXPathContext_);
      pXPathContext_ = 0;
    }
}

bool EmulationScriptLoader::errorOccurred()
{
  mutex_.acquire();

  bool bRetVal = bError_;

  mutex_.release();

  return bRetVal;
}

void EmulationScriptLoader::cancel()
{
  eventBuffer_.produce(0);

  mutex_.acquire();

  bCancel_ = true;

  mutex_.release();
}

void EmulationScriptLoader::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{

}

void EmulationScriptLoader::processEvent(const EMANE::EventId&, const EMANE::EventObjectState&)
{

}

ACE_THR_FUNC_RETURN EmulationScriptLoader::svc()
{
  /* Repeat as many times as requested */
  for (ACE_UINT16 u16Index = 0; u16Index < u16RepeatCount_; ++u16Index)
    {
      /* Go through the list of files */
      FileVectorType::const_iterator iter;
      for (iter = files_.begin(); iter != files_.end(); ++iter)
        {
          /* Load the next file, see if parses OK */
          int iErrorCode = load(iter->c_str());

          if (iErrorCode != DOCUMENT_PARSE_SUCCESS)
            {
              pPlatformService_->log(EMANE::ERROR_LEVEL,
                  "EmulationScriptLoader::Failed to parse %s. Returned ErrorCode: %s\n",
                  iter->c_str(), 
                  (iErrorCode==DOCUMENT_PARSE_FAILED)?"DOCUMENT_PARSE_FAILED":
                  (iErrorCode==SCHEMA_LOAD_FAILED)?"SCHEMA_LOAD_FAILED":
                  (iErrorCode==SCHEMA_CTXT_FAILED)?"SCHEMA_CTXT_FAILED":
                  (iErrorCode==SCHEMA_INVALID)?"SCHEMA_INVALID":
                  (iErrorCode==SCHEMA_CTXT_INVALID)?"SCHEMA_CTXT_INVALID":"????");
              
              /* Fatal error, someone goofed...*/
              mutex_.acquire();

              bError_ = true;

              mutex_.release();

              /* Dropping in a NULL value to indicate ERROR */
              eventBuffer_.produce(0);

              /* Clean-up buffered events */
              Event *pEvent = eventBuffer_.consume();
              while (pEvent)
                {
                  delete pEvent;
                  pEvent = eventBuffer_.consume();
                }

              /* Exit thread */
              return 0;
            }

          /* File parsed OK, continue */
          /* Get events */
          Event *pEvent = getNextEvent();
          while (pEvent != 0)
            {
              /* Put this event on the queue */
              eventBuffer_.produce(pEvent);

              /* Get next event */
              pEvent = getNextEvent();
            }// end while event != 0

        }// end for list of files

    }// end for repeat count
 
  /* Thread is complete */
  return 0;
}
