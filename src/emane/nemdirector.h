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

#ifndef EMANENEMDIRECTOR_HEADER_
#define EMANENEMDIRECTOR_HEADER_

#include <string>

#include "libemane/nembuilder.h"
#include "platformconfiguration.h"
#include "emaneparseexception.h"
#include "emane/emaneconfigureexception.h"

/**
 * @class EMANE::NEMDirector nemdirector.h "nemdirector.h"
 *
 * @brief Director used to build Platform NEM (s) with the NEMBuilder.
 *
 * The NEMDirector class is the implementation of the Director
 * part of the Builder design pattern from the GoF book (see 
 * reference, at end of file). Its task is to manage the creation
 * of NEMs with the use of the NEMBuilder
 *
 * @sa NEMBuilder
 */

namespace EMANE
{
  class NEMDirector
  {
  public:
    /**
     * Constructor
     *
     * @param filename reference to the base XML filename
     * @param builder reference to the NEM builder
     */
    NEMDirector(const std::string &filename,
                EMANE::NEMBuilder &builder)
      throw(ParseException,ValidateException);

    /**
     * Destructor
     */
    ~NEMDirector();
  
    /**
     * Constructs the platform
     */
    EMANE::Platform * construct();

  private:
    /**
     * Uses the passed-in builder to create an NEM object and return
     * a pointer to it.
     *
     * @param pNEMConfig Pointer to the NEMConfiguration object for this NEM
     *
     * @retval a pointer to the newly created NEM object (memory
     *         ownership is trasferred to the caller).
     *
     * @exception ConfigureException FactoryException
     */
    EMANE::NEM* createNEM(EMANE::NEMConfiguration *pNEMConfig)
      throw(ConfigureException,EMANEUtils::FactoryException,PlatformException);

    /**
     * Container for configuration data
     */
    EMANE::PlatformConfiguration configPlatform_;

    /**
     * Holder for the builder
     */
    EMANE::NEMBuilder &rNEMBuilder_;

  };

  /* 
   * Reference:
   *  Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
   *  Design Patterns: Elements of Reusable Object-Oriented Software.
   *  Addison-Wesley, Reading MA, 1995
   *  Builder, p 97
   */
}

#endif //EMANENEMDIRECTOR_HEADER_
