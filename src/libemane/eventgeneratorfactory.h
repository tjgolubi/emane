/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTGENERATORFACTORY_HEADER_
#define EMANEEVENTGENERATORFACTORY_HEADER_

#include "emane/emaneeventgenerator.h"

#include "emaneutils/factoryexception.h"
#include "emaneutils/componenttypes.h"

#include <ace/OS_NS_dlfcn.h>

namespace EMANE
{
  /**
   * @class EventGeneratorFactory
   *
   * @brief Factory for creating EventGenerators.  The factory
   * manages the DLL allowing for the creation of multiple generators
   *
   * @note DLLs must stay open in order to use their contents
   */
  class EventGeneratorFactory
  {
  public:
    /**
     * Constructor
     *
     * @param sLibraryName Filename of DLL
     */
    EventGeneratorFactory(const std::string & sLibraryName) 
      throw(EMANEUtils::FactoryException);
    
    ~EventGeneratorFactory();
    
    /**
     * Create an EventGenerator
     *
     * @returns EventGenerator reference
     */
    EventGenerator * createEventGenerator(EMANE::PlatformServiceProvider *pPlatformService) const;

    /**
     * Destory an EventGenerator
     *
     * @param pGen Reference to EventGenerator 
     */
    void destoryEventGenerator(EventGenerator * pGen) const;
    
  private:
    typedef EventGenerator * (*CreateEventGeneratorFunc)(EMANE::PlatformServiceProvider *pPlatformService); 
    typedef void (*DestroyEventGeneratorFunc)(EventGenerator*); 

    ACE_SHLIB_HANDLE shlibHandle_;
    CreateEventGeneratorFunc createEventGeneratorFunc_;
    DestroyEventGeneratorFunc destroyEventGeneratorFunc_;
  };
}

#endif //EMANEEVENTGENERATORFACTORY_HEADER_

