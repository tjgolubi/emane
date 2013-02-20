/*
 * Copyright (c) 2008-2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTAGENTBUILDER_HEADER_
#define EMANEEVENTAGENTBUILDER_HEADER_
 
#include <string>
#include <list> 

#include "emane/emaneeventagentmanager.h"
#include "emane/emaneconfigurationitem.h"
#include "emane/emanebuildexception.h"
#include "emaneutils/factoryexception.h"

namespace EMANE
{
  typedef std::list<EMANE::EventAgent*> EventAgents;

  /**
   * @class EventAgentBuilder
   *
   * @brief Provides methods for constructing event agents and a
   * manager to contain and control them as a group.
   *
   * Reference:
   * Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
   * Design Patterns: Elements of Reusable Object-Oriented Software.
   * Addison-Wesley, Reading MA, 1995
   * Builder, p 97
   */
  class EventAgentBuilder
  {
  public:
    EventAgentBuilder();
    
    ~EventAgentBuilder();
    
    /**
     * Build and return a configured EventAgentManager
     *
     * @param agents the list of agents to manage
     * @param pItems pointer to ConfigurationItems list
     * @return pointer to an initialized and configured EventAgentManager
     */
    EMANE::EventAgentManager *
    buildEventAgentManager(const EventAgents &agents,
                           const ConfigurationItems * pItems = 0)
    throw(BuildException,ConfigureException,InitializeException);

    /**
     * Build and return a configured EventAgent
     *
     * @param nemId  NEM id of the nem for which this event agent is being 
     *               created
     * @param sLibraryFile Name of the dll containing the generator
     * @param pItems pointer to ConfigurationItems list
     * @return pointer to an initialized and configured EventAgent
     */
    EMANE::EventAgent *
    buildEventAgent(EMANE::NEMId nemId,
                    const std::string & sLibraryFile,
                    const ConfigurationItems * pItems = 0)
      throw(EMANEUtils::FactoryException,
            ConfigureException,
            InitializeException);
  };
}

#endif //EMANEEVENTAGENTBUILDER_HEADER_
