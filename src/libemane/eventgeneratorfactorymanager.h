/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTGENERATORFACTORYMANAGER_HEADER_
#define EMANEEVENTGENERATORFACTORYMANAGER_HEADER_

#include "eventgeneratorfactory.h"

#include "emane/emaneeventgenerator.h"

#include "emaneutils/factoryexception.h"

#include <map>
#include <string>
#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>

namespace EMANE
{
  /**
   * @class EventGeneratorFactoryManager
   *
   * @brief Factory Manager Singleton cache for EventGeneratorFactory objects.
   * The manager creates and caches the factories and keeps them in scope for the 
   * duration of its lifecycle.
   *
   */

  class EventGeneratorFactoryManager
  {
  public:
    EventGeneratorFactoryManager();
    ~EventGeneratorFactoryManager();
    
    /**
     * Retreive specific EventGeneratorFactory reference
     *
     * @param sLibraryFile DLL file name
     */
    const EventGeneratorFactory  & 
    getEventGeneratorFactory(const std::string & sLibraryFile)
      throw(EMANEUtils::FactoryException);
    
  private:
    typedef std::map<std::string, EventGeneratorFactory *>  EventGeneratorFactoryMap;
    
    EventGeneratorFactoryMap eventGeneratorFactoryMap_;
  };
  
  typedef ACE_Unmanaged_Singleton<EventGeneratorFactoryManager,ACE_Null_Mutex> EventGeneratorFactoryManagerSingleton;
}

#endif //EMANEEVENTGENERATORFACTORYMANAGER_HEADER_
