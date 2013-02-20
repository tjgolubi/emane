/*
 * Copyright (c) 2011-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef COMPONENTMAP_HEADER_
#define COMPONENTMAP_HEADER_

#include "emane/emanetypes.h"
#include "emane/emanenemlayer.h"
#include "emane/emanenem.h"
#include "emane/emaneplatform.h"
#include "emane/emanetransportmanager.h"
#include "emane/emaneeventagentmanager.h"
#include "emane/emaneeventgeneratormanager.h"
#include "componentmapentries.h"
#include "componentmapexception.h"
#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>
#include <map>
#include <vector>
#include <list>

namespace EMANE
{
  /**
   * @struct NEMLayerDescriptor
   *
   * @brief Structure containing NEMLayer attributes. These structures
   *        may accessed from the ComponentMap when registration is complete
   */
  struct NEMLayerDescriptor
  {
    BuildId buildId;
    ComponentType type;

  /**
   * NEMLayerDescriptor initializer
   *
   * @param bid the BuildId of the corresponding NEMLayer
   * @param t the ComponentType of the corresponding NEMLayer
   */   
    NEMLayerDescriptor(BuildId bid, ComponentType t) : 
      buildId(bid), type(t)
    {}
  };

  /**
   * @class ComponentMap
   *
   * @brief ComponentMap facilitates tracking objects through 
   *        through the application. It provides a method to
   *        generate unique identifiers (BuildIds),
   *        methods to register objects and their associations against
   *        these Ids, and methods to retrieve this information
   *        once registration is complete.    
   *
   *        For example use, ComponentMap is currently used to register 
   *        statistics to the StatisticsManager before the full system
   *        is assembled.
   */
  class ComponentMap
  {
  public:
    ComponentMap();
    ~ComponentMap();

    /**
     * Get a unique BuildId
     *
     * @return a nem BuildId
     */
    BuildId generateId();

    /**
     * Register a new NEMLayer against it's BuildId
     *
     * @param bid the BuildId of the NEMLayer, from generateId
     * @param type the componenttype of this NEMLayer
     * @param pNemLayer a pointer to the NEMLayer instance
     */
    void registerNEMLayer(const BuildId bid, 
                          const ComponentType type,
                          NEMLayer * pNemLayer);

    /**
     * Register a new NEM
     *
     * @param bid the BuildId of the NEM, from generateId
     * @param nemid the new NEM id
     * @param nem a pointer to the NEM instance
     */
    void registerNEM(const BuildId bid, const NEMId nemid, NEM * nem);

    /**
     * Assign a NEMLayer to a NEM
     *
     * @param nembid the previously registered BuildId of the NEM
     * @param nemlayerbid the previoulsy registered BuildId of the NEMLayer
     */
    void assignNEMLayerToNEM(const BuildId nembid, 
                             const BuildId nemlayerbid);

    /**
     * Register a new Platform
     *
     * @param bid the BuildId of the Platform, from generateId
     * @param platformid the Platform's PlatformId
     * @param platform pointer to the Platform instance
     */
    void registerPlatform(const BuildId bid, 
                          const PlatformId platformid, 
                          Platform * platform);

    /**
     * Assign a NEM to a Platform
     *
     * @param platformbid the previously registered BuildId of the Platform
     * @param nembid the previously registered BuildId of the NEM
     */
    void assignNEMToPlatform(const BuildId platformbid, 
                             const BuildId nembid);    

    /**
     * Number or registered platforms
     *
     * @return the number of registered platforms
     */
    size_t registeredPlatformCount() const;

    /**
     * Register a new EventAgentManager
     *
     * @param bid the BuildId of the EventAgentManager, from generateId
     */
    void registerEventAgentManager(const BuildId bid);

    /**
     * Number or registered EventAgentManagers
     *
     * @return the number of registered EventAgentManagers
     */
    size_t registeredEventAgentManagerCount() const;

    /**
     * Register a new EventGeneratorManager
     *
     * @param bid the BuildId of the EventGeneratorManager, from generateId
     */
    void registerEventGeneratorManager(const BuildId bid);

    /**
     * Number or registered EventAgentManagers
     *
     * @return the number of registered EventAgentManagers
     */
    size_t registeredEventGeneratorManagerCount() const;


    /**
     * Register a new EventGeneratorManager
     *
     * @param bid the BuildId of the EventGeneratorManager, from generateId
     */
    void registerTransportManager(const BuildId bid);

    /**
     * Number or registered TransportManagers
     *
     * @return the number of registered TransportManagers
     */
    size_t registeredTransportManagerCount() const;


    /**
     * Close registration and enable client access to registration 
     * information.
     */
    void closeRegistration();

    /**
     * Close the ComponentMap
     */
    void close();



    /**
     * Determine a valid NEMId
     *
     * @param nemid the NEMId to validate
     * @return true if the nemid is registered, otherwise false
     */
    bool isValid(NEMId nemid) const;

    /**
     * Determine a valid NEMId and index
     *
     * @param nemid the NEMId to validate
     * @param layerindex the index into this NEMs layer stack to validate.
     * @return true if the nemid is registered and the layerindex is in 
     *         the range [0, getLayerCount()-1], otherwise false.
     */
    bool isValid(NEMId nemid, size_t layerindex) const;

    /**
     * Determine a valid NEMId and ComponentType
     *
     * @param nemid the NEMId to validate
     * @param type a NEMLayer type
     * @return true if the nemid is registered and the NEM contains
     *         a NEMLayer of the specified type, otherwise false.
     */
    bool isValid(NEMId nemid, ComponentType type) const;

    /**
     * Determine the number of registered NEMs
     *
     * @return The number or registered NEMs
     */
    size_t getNEMCount() const;

    /**
     * Determine the number of layers in the indexed NEM
     *
     * @param nemindex the NEM index
     */
    size_t getLayerCount(size_t nemindex) const;

    /**
     * Determine the NEM index from the NEMId
     *
     * @param id The NEM index to query
     * @return The index in the range of [0, getNEMCount()-1] that
     *         can be used to reference the NEM with the indicated NEMId
     */
    size_t getNEMIndex(NEMId id) const
      throw(ComponentMapException);

    /**
     * Determine the NEMId from the index
     *
     * @param index an index in the range [0, getNEMCount()-1]
     * @return The index in the range of [0, getNEMCount()-1] that
     *         can be used to reference the NEM with the indicated NEMId
     */
    NEMId getNEMId(size_t index) const
      throw(ComponentMapException);

    /**
     * Fetch the NEMLayerDescriptor for the NEMLayer at the specified
     * indices
     *
     * @param nemindex index of the NEM containing the layer to fetch
     * @param layerindex index of the layer within the specified NEM to fetch
     * @return the NEMLayerDescriptor for the specified location
     */
    NEMLayerDescriptor getLayerDescriptor(size_t nemindex, size_t layerindex) const;

  private:
    BuildId nextId_;
    bool openregistration_;

    // UnassignedIDList contains generated BuildIds that
    // haven't been registered against any object yet
    typedef std::list<BuildId> UnassignedIdList;
    typedef UnassignedIdList::size_type UnassignedIdListSize;
    UnassignedIdList unassignedIdList_;

    // NEMLayerMap is a map of BuildIds to NEMLayerEntry objects
    typedef LeafEntry<NEMLayer> NEMLayerEntry;
    typedef std::map<BuildId, NEMLayerEntry*> NEMLayerMap;
    NEMLayerMap nemlayermap_;

    // NEMMap is a mapping of BuildIds to NEMEntry objects
    typedef ContainerEntry<NEM, NEMLayerEntry, NEMId> NEMEntry;
    typedef std::map<BuildId, NEMEntry*> NEMMap;
    NEMMap nemmap_;

    // PlatformMap is a mapping of BuildIds to PlatformEntry objects
    typedef ContainerEntry<Platform, NEMEntry, PlatformId> PlatformEntry;
    typedef std::map<BuildId, PlatformEntry*> PlatformMap;
    PlatformMap platformmap_;

    // A 2D vector mapping (nemindex,layerindex) pairs
    // to a NEMLayerDescriptor instance. Clients use this
    // to discover the component hierarchy
    typedef std::vector<NEMLayerDescriptor> NEMLayerDescriptorVector;
    typedef std::vector<NEMLayerDescriptorVector > NEMLayerDescriptors;
    NEMLayerDescriptors layerDescriptors_;

    // map and inverse map of NEMId and nemindex - first index
    // into the NEMLayerDescriptors map
    typedef std::map<NEMId, size_t> NEMIdToIndexMap;
    typedef NEMIdToIndexMap::const_iterator NEMIdToIndexMapIter;
    NEMIdToIndexMap nemIdToIndex_;
    typedef std::map<size_t, NEMId> NEMIndexToIdMap;
    typedef NEMIndexToIdMap::const_iterator NEMIndexToIdMapIter;
    NEMIndexToIdMap nemIndexToId_;

    // internal functions used
    bool isRegisteredNEMLayerId(BuildId nemlayerbid);
    bool isRegisteredNEMId(BuildId nembid);
    bool isRegisteredPlatformId(BuildId platformbid);
    void buildtables();
    bool removeUnassignedId(BuildId bid);

    // internal counters - currently these Managers are only counted
    size_t registeredEventAgentManagerCount_;
    size_t registeredEventGeneratorManagerCount_;
    size_t registeredTransportManagerCount_;
  };
  typedef ACE_Unmanaged_Singleton<ComponentMap,ACE_Null_Mutex> ComponentMapSingleton;
}


#endif
