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

#ifndef EVENT_HEADER_
#define EVENT_HEADER_

#include "node.h"

#include <vector>

#include <ace/Time_Value.h>

/**
 * @class Event
 *
 * @brief Wrapper for the Event element in XML
 */
class Event
{
public:
  /*
   * Typedef for container of node pointers
   */
  typedef std::vector<Node*> NodeContainerType;

  /**
   * Constructor
   *
   * @param pczTime Character string with time value
   *
   * @todo: take care of the type of time (i.e. format)
   */
  Event(const char* pczTime);

  /**
   * Destructor
   *
   */
  virtual ~Event();

  /**
   * Returns the time of this event (relative to start time)
   *
   * @return time_ Time of this event (relative to start time)
   */
  const ACE_Time_Value &getTime();

  /**
   * Returns a reference to the internal node container
   *
   * @return nodeContainer_ Reference to the internal node container
   */
  const NodeContainerType &getNodeContainer();

  /**
   * Adds the passed-in node to the container
   *
   * @param pNode Pointer to the node to be added
   */
  void addNode(Node *pNode);

  /**
   * Prints event's data to the screen
   *
   *
   */
  void dumpToScreen();

private:
  /**
   * The relative timestamp of the current event
   */
  ACE_Time_Value time_;

  /**
   * Container for Node pointers
   */
  NodeContainerType nodeContainer_;
};

#endif /* EVENT_HEADER_ */
