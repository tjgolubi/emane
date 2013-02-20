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

#include "eventgeneratorfactory.h"

#include <iostream>
#include <sstream>

EMANE::EventGeneratorFactory::EventGeneratorFactory(const std::string & sLibraryName) 
  throw(EMANEUtils::FactoryException):
  shlibHandle_(0),
  createEventGeneratorFunc_(0),
  destroyEventGeneratorFunc_(0)
{
   if((shlibHandle_ = ACE_OS::dlopen(sLibraryName.c_str(), RTLD_NOW)) == 0)
    {
      throw EMANEUtils::FactoryException(ACE_OS::dlerror());
    }
  
  if((createEventGeneratorFunc_ = reinterpret_cast<CreateEventGeneratorFunc>((void (*)())(ACE_OS::dlsym(shlibHandle_,"create")))) == 0)
    {
      ACE_OS::dlclose(shlibHandle_);
      std::stringstream sstream;
      sstream<<sLibraryName<<" missing create symbol. (Missing DECLARE_EVENT_GENERATOR()?)"<<std::ends;
      throw EMANEUtils::FactoryException(sstream.str());
    }

  if((destroyEventGeneratorFunc_ = reinterpret_cast<DestroyEventGeneratorFunc>((void (*)())ACE_OS::dlsym(shlibHandle_,"destroy"))) == 0)
    {
      ACE_OS::dlclose(shlibHandle_);
      std::stringstream sstream;
      sstream<<sLibraryName<<" missing destory symbol. (Missing DECLARE_EVENT_GENERATOR()?)"<<std::ends;
      throw EMANEUtils::FactoryException(sstream.str());
    }
}

EMANE::EventGeneratorFactory::~EventGeneratorFactory()
{
  if(ACE_OS::dlclose(shlibHandle_) == -1)
    {
      std::cerr<<ACE_OS::dlerror()<<std::endl;
    }
}

EMANE::EventGenerator * EMANE::EventGeneratorFactory::createEventGenerator(EMANE::PlatformServiceProvider *pPlatformService) const
{
  return createEventGeneratorFunc_(pPlatformService);
}

void EMANE::EventGeneratorFactory::destoryEventGenerator(EventGenerator * pGenerator) const
{
  destroyEventGeneratorFunc_(pGenerator);
}
