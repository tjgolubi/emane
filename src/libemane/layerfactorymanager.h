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

#ifndef EMANELAYERFACTORYMANAGER_HEADER_
#define EMANELAYERFACTORYMANAGER_HEADER_

#include "layerfactory.h"

#include "emane/emanemaclayerimpl.h"
#include "emane/emanephylayerimpl.h"
#include "emane/emaneshimlayerimpl.h"

#include "emaneutils/factoryexception.h"

#include <map>
#include <string>
#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>

namespace EMANE
{
  typedef  LayerFactory<MACLayerImplementor>  MACLayerFactory;
  typedef  LayerFactory<PHYLayerImplementor>  PHYLayerFactory;
  typedef  LayerFactory<ShimLayerImplementor> ShimLayerFactory;
  
  /**
   * @class LayerFactoryManager
   *
   * @brief Factory Manager Singleton cache for LayerFactory objects. The
   * manager creates and caches the factories and keeps them in scope for the 
   * duration of its lifecycle.
   */
  class LayerFactoryManager
  {
  public:
    LayerFactoryManager();
    ~LayerFactoryManager();

    /**
     * Retreive specific a MACLayerFactory reference
     *
     * @param sLibraryFile DLL file name
     */
    const MACLayerFactory  & 
    getMACLayerFactory(const std::string & sLibraryFile)
      throw(EMANEUtils::FactoryException);

    /**
     * Retreive specific a PHYLayerFactory reference
     *
     * @param sLibraryFile DLL file name
     */
    const PHYLayerFactory  &
    getPHYLayerFactory(const std::string & sLibraryFile)
      throw(EMANEUtils::FactoryException);
    
    /**
     * Retreive specific a ShimLayerFactory reference
     *
     * @param sLibraryFile DLL file name
     */
    const ShimLayerFactory &
    getShimLayerFactory(const std::string & sLibraryFile)
      throw(EMANEUtils::FactoryException);
    
  private:
    
    typedef std::map<std::string, MACLayerFactory *>  MACLayerFactoryMap;
    typedef std::map<std::string, PHYLayerFactory *>  PHYLayerFactoryMap;
    typedef std::map<std::string, ShimLayerFactory *> ShimLayerFactoryMap;
    
    MACLayerFactoryMap  macLayerFactoryMap_;
    PHYLayerFactoryMap  phyLayerFactoryMap_;
    ShimLayerFactoryMap shimLayerFactoryMap_;
  };

  typedef ACE_Unmanaged_Singleton<LayerFactoryManager,ACE_Null_Mutex> LayerFactoryManagerSingleton;
}

#endif //EMANELAYERFACTORYMANAGER_HEADER_
