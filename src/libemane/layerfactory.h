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

#ifndef EMANELAYERFACTORY_HEADER_
#define EMANELAYERFACTORY_HEADER_

#include "emaneutils/factoryexception.h"
#include "emaneutils/componenttypes.h"
#include "platformservice.h"

namespace EMANE
{
  /**
   * @class LayerFactory
   *
   * @brief Factory for creating @a T layers .  The factory
   * manages the DLL allowing for the creation of layers
   * of the same type.
   *
   * @note DLLs must stay open in order to use their contents
   */
  template<class T>
  class LayerFactory
  {
  public:
    /**
     * Constructor
     *
     * @param sLibraryName Filename of DLL
     */
    LayerFactory(const std::string & sLibraryName) 
      throw(EMANEUtils::FactoryException);

    ~LayerFactory();
    
    /**
     * Create a @a T layer
     *
     * @param nemId NEM id
     * @param pPlatformService platform service
     *
     * @returns T reference to newly created layer
     */
    T * createLayer(EMANEUtils::NEMId nemId, PlatformService * pPlatformService) const;

    /**
     * Destory a @a T layer
     *
     * @param layer Layer to destory
     *
     */
    void destoryLayer(T * layer) const;
    
  private:
    typedef T * (*createLayerFunc)(EMANEUtils::NEMId, PlatformService *); 
    typedef void (*destroyLayerFunc)(T*); 

    ACE_SHLIB_HANDLE shlibHandle_;
    createLayerFunc  createLayerFunc_;
    destroyLayerFunc destroyLayerFunc_;
  };
}

#include "layerfactory.inl"

#endif //EMANELAYERFACTORY_HEADER_
