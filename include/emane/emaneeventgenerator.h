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

#ifndef EMANEEVENTGENERATOR_HEADER_
#define EMANEEVENTGENERATOR_HEADER_

#include "emane/emanecomponent.h"
#include "emane/emanecomponenttypes.h"
#include "emane/emaneplatformserviceuser.h"
#include "emane/emanebuildable.h"

#include <string>
#include <list>

namespace EMANE
{
  /**
   * @class EventGenerator
   *
   * @brief Base class for all event generators
   */
  class EventGenerator : public Component,
                         public PlatformServiceUser,
                         public Buildable
  {
  public:
  
    typedef std::list<EventId> EventIdList;

    virtual ~EventGenerator(){}

    /**
     * Returns the name of the generator.
     *
     * @retval name Copy of the name of the generator.
     */
    std::string getName();

    /**
     * Returns the mapping container.
     *
     * @retval list Copy of the list containing EventIds for this generator.
     */
    EventIdList getEventIdList();

  protected:
    EventGenerator(const std::string & sName, EMANE::PlatformServiceProvider *pPlatformService) : 
     PlatformServiceUser(pPlatformService),
     sName_(sName) 
    { }

    /**
     * Add event to the list of events produced bu generator
     *
     * @param id Event id of the produced event
     */
    void addEventId(EventId id);

  private:
    /**
     * Holds the name of the Generator.
     */
    std::string sName_;

    /**
     * Holds the EventIds served by this generator.
     */
    EventIdList eventIdList_;

  };
}

#include "emane/emaneeventgenerator.inl"

#define DECLARE_EVENT_GENERATOR(X)                                                \
  extern "C"  EMANE::EventGenerator * create(EMANE::PlatformServiceProvider *p)   \
  {return new X(p);}                                                              \
  extern "C"  void destroy(EMANE::EventGenerator * p)                             \
  {delete p;}                            

#endif //EMANEEVENTGENERATOR_HEADER_
