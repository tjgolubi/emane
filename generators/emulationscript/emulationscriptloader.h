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

#ifndef EMULATIONSCRIPTLOADER_HEADER_
#define EMULATIONSCRIPTLOADER_HEADER_

#include "node.h"
#include "event.h"

#include "emane/emaneplatformserviceuser.h"
#include "emaneevents/locationevent.h"
#include "emaneutils/producerconsumerbuffer.h"

#include <vector>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>
#include <libxml/xpath.h>

/**
 * @class EmulationScriptLoader
 *
 * @brief Loads Events from an emulation script-formatted XML file
 */
class EmulationScriptLoader : public EMANE::PlatformServiceUser
{
public:
  /*
   * Internal error codes
   */
  enum
    {
      DOCUMENT_PARSE_SUCCESS =  1,
      DOCUMENT_PARSE_FAILED  = -1,
      SCHEMA_LOAD_FAILED     = -2,
      SCHEMA_CTXT_FAILED     = -3,
      SCHEMA_INVALID         = -4,
      SCHEMA_CTXT_INVALID    = -5,
      XPATH_CONTEXT_FAILED   = -6
    };

  /*
   * Local Type definitions
   */
  typedef std::vector<Event*> EventContainerType;
  typedef std::vector<std::string> FileVectorType;
  typedef std::vector<LocationEvent::LocationEntry> LocationListType;

  /**
   * Constructor
   *
   */
  EmulationScriptLoader();

  /**
   * Destructor
   */
  virtual ~EmulationScriptLoader();

  /**
   * Initializes the loader with the specific params and kicks off the thread
   *
   * @param fileNames Reference to a container of FileVectorType
   * @param sSchemaName Name of the schema to validate against
   * @param u16RepCnt Repeat count (default is 1)
   *
   * @todo Add entry replay capability as in original generator
   */
  void open(const FileVectorType &fileNames, const std::string &sSchemaName,
            ACE_UINT16 u16RepCnt = 1);

  /**
   * Finds the 'startTime' element and returns its content
   *
   * @return time Character string with the actual start time
   *
   * @todo: Make this function return a 'StartTime' object with time as 
   *        specified by the 'type' attribute and element contents.
   */
  const char *getStartTime();
  
  /**
   * Finds the 'Event' elements, processes them and fills the supplied container
   *
   * @param cont Reference to a container to be filled with events
   * @param bUseXPath Boolean flag specifying whether or not to use XPath
   *                  to fill the referenced container.
   *
   * @note If the bUseXPath flag is set to true, the implementation of this
   *       function will use XPath to retrieve Event elements from the 
   *       document. Per LibXML documentation, the order in which said 
   *       elements are retrieved CANNOT be guarateed.
   */
  void fillEventContainer(EventContainerType &cont, bool bUseXPath = false);

  /**
   * Returns a pointer to a new'ed Event based on the next Event element
   * from the XML document
   *
   * @return pEvent Pointer to a new Event object based on the next Event 
   *                element from the document (NULL if the previously returned
   *                value was the last one).
   *
   * @note Caller is responsible for deallocation of the returned Event object
   */
  Event *getNextEvent();

  /**
   * Returns the next available set of locations from the buffer
   *
   * @param[out] sleepTime The difference between last set of locations
   *                       and the one being currently returned
   *
   * @return locations A container of locations from the buffer (LocationListType)
   */
  LocationListType getNextLocations(ACE_Time_Value &sleepTime);

  /**
   * Resets the 'LastEventReached' flag to allow looping
   *
   * @note If constant looping is desired, this function must be called
   *       after every iteration
   */
  void resetLastEventReached();

  /**
   * Returns error status
   *
   * @return true if bError_ flag is true, false otherwise
   */
  bool errorOccurred();

  /**
   * Enables event generator thread cancelation
   */
  void cancel();

  void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);

  void processEvent(const EMANE::EventId&, const EMANE::EventObjectState&);

private:
  /* Private member functions */

  /**
   * Checks for validity of a document against a given schema 
   *
   * @param pczSchemaFilename Name of the file containing the schema
   *
   * @return int 1 if document is valid, value corresponding to specific error,
   *             otherwise
   */
  int isDocumentValid(const char *pczSchemaFilename);

  /**
   * Gets nodes matching the specified xpath expression
   *
   * @param xpath Character string with the expression to find
   *
   * @return pXPath Pointer to an xpath object with the resulting nodeset
   *                (can be NULL if no nodes match the expression)
   */
  xmlXPathObjectPtr getNodesByXPath(const xmlChar *xpath);

  /**
   * Parses the 'Node' element inside of the event
   *
   * @param node Pointer to an xml node corresponding to the 'Node'
   *
   * @return pNode Pointer to a newly created Node object
   */
  Node *parseNodeElement(xmlNodePtr node);

  /**
   * Parses the 'Event' element
   *
   * @param node Pointer to an xml node corresponding to the 'Event'
   *
   * @return pEvent Pointer to a newly created event
   */
  Event *parseEventElement(xmlNodePtr node);

  /**
   * Loads, parses and validates the specified file.
   *
   * @param pczFilename  Name of the file to use as input
   *
   * @return error Success (1) or failure (!= 1) with parsing and/or validation
   *
   */
  int load(const char *pczFilename);

  /**
   * Cleans up the document and XPath context pointers
   *
   *
   */
  void xmlCleanupDocument();

  /**
   * Thread member function for event loading
   */
  ACE_THR_FUNC_RETURN svc();

  /* Private data members */

  /**
   * Name of the file containing the schema to validate against
   */
  std::string sSchemaFile_;

  /**
   * Pointer to the parsed document
   */
  xmlDocPtr pDocument_;

  /**
   * Pointer to the XPath context
   */
  xmlXPathContextPtr pXPathContext_;

  /**
   * Pointer to the current event XML Node - this is only useful when 
   * retrieving 1 event at a time
   */
  xmlNodePtr pCurrentEvent_;

  /**
   * Boolean flag to prevent looping when pCurrentEvent_ is NULL
   */
  bool bLastEventReached_;

  /**
   * Number of times to repeat this gig
   */
  ACE_UINT16 u16RepeatCount_;

  /**
   * Time of last returned entry
   */
  ACE_Time_Value lastTimePeriod_;

  /**
   * Thread responsible for event loading
   */
  ACE_thread_t thread_;
  
  /**
   * Flag to signal an error
   */
  bool bError_;

  /**
   * Flag to signal cancellation
   */
  bool bCancel_;

  /**
   * Mutex for the thread
   */
  ACE_Thread_Mutex mutex_;

  /**
   * Container with filenames
   */
  FileVectorType files_;

  /**
   * Buffer with location updates
   */
  EMANEUtils::ProducerConsumerBuffer<Event*> eventBuffer_;

};

#endif /* EMULATIONSCRIPTLOADER_HEADER_ */
