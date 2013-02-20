/*
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
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

#include "componentmap.h"
#include "logservice.h"
#include <string>
#include <sstream>


namespace EMANE
{
  /**
   * @class ispresent
   *
   * @brief functor class searching for build id in a list of ids
   */
  class ispresent
  {
  public:
    ispresent(EMANE::BuildId& bid) : bid_(bid) {}
    bool operator ()(const EMANE::BuildId& bid) { return bid==bid_; }
  private:
    EMANE::BuildId bid_;
  };
}


EMANE::ComponentMap::ComponentMap() : 
  nextId_(1), 
  openregistration_(true),
  registeredEventAgentManagerCount_(0),
  registeredEventGeneratorManagerCount_(0),
  registeredTransportManagerCount_(0)
 {}


EMANE::ComponentMap::~ComponentMap() {}


EMANE::BuildId 
EMANE::ComponentMap::generateId() 
{
  unassignedIdList_.push_back(nextId_);
  return nextId_++;
}


void 
EMANE::ComponentMap::registerNEMLayer(const EMANE::BuildId bid,
                                      const EMANE::ComponentType type,
                                      EMANE::NEMLayer * pNemLayer)
{
  if(openregistration_)
    {
      if(removeUnassignedId(bid))
        {
          nemlayermap_[bid] = new NEMLayerEntry(bid, type, pNemLayer);
        }
    }
}



void 
EMANE::ComponentMap::registerNEM(const BuildId bid, 
                                 const NEMId nemid, 
                                 NEM * nem)
{
  if(openregistration_)
    {
      if(removeUnassignedId(bid))
        {
          nemmap_[bid] = new NEMEntry(bid, nem, nemid);
        }
    }
}


void  
EMANE::ComponentMap::registerPlatform(const BuildId bid, 
                                      const PlatformId platformid, 
                                      Platform * platform) 
{
  if(openregistration_)
    {
      if(removeUnassignedId(bid))
        {
          platformmap_[bid] = new PlatformEntry(bid, platform, platformid);
        }
    }
}


void 
EMANE::ComponentMap::assignNEMLayerToNEM(const BuildId nembid, 
                                         const BuildId nemlayerbid) 
{
  if(openregistration_)
    {
      if(isRegisteredNEMId(nembid) && isRegisteredNEMLayerId(nemlayerbid))
        {
          nemmap_[nembid]->addChild(nemlayermap_[nemlayerbid]);
        }
    }
}


void 
EMANE::ComponentMap::assignNEMToPlatform(const BuildId platformbid, 
                                         const BuildId nembid)
{
  if(openregistration_)
    {

      if(isRegisteredPlatformId(platformbid) && isRegisteredNEMId(nembid)) 
        {
          platformmap_[platformbid]->addChild(nemmap_[nembid]);
        }
    }
}


size_t 
EMANE::ComponentMap::registeredPlatformCount() const
{
  return static_cast<size_t>(platformmap_.size());
}



void 
EMANE::ComponentMap::registerEventAgentManager(const BuildId)
{
  registeredEventAgentManagerCount_++;
}

size_t 
EMANE::ComponentMap::registeredEventAgentManagerCount() const
{
  return registeredEventAgentManagerCount_;
}

void 
EMANE::ComponentMap::registerEventGeneratorManager(const BuildId)
{
  registeredEventGeneratorManagerCount_++;
}

size_t 
EMANE::ComponentMap::registeredEventGeneratorManagerCount() const
{
  return registeredEventGeneratorManagerCount_;
}


void 
EMANE::ComponentMap::registerTransportManager(const BuildId)
{
  registeredTransportManagerCount_++;
}

size_t 
EMANE::ComponentMap::registeredTransportManagerCount() const
{
  return registeredTransportManagerCount_;
}



void 
EMANE::ComponentMap::closeRegistration()
{
  if(openregistration_)
    {
      openregistration_ = false;      
      buildtables();
    }
}


void 
EMANE::ComponentMap::close()
{
  // destroy entries
}


bool 
EMANE::ComponentMap::isValid(NEMId nemid) const
{
  return nemIdToIndex_.find(nemid) != nemIdToIndex_.end();
}


bool 
EMANE::ComponentMap::isValid(NEMId nemid, size_t layerindex) const
{
  if(!isValid(nemid))
    {
      return false;
    }
  size_t nemindex = getNEMIndex(nemid);
  return ((nemindex < layerDescriptors_.size()) &&
          (layerindex < layerDescriptors_[nemindex].size()));
}


size_t 
EMANE::ComponentMap::getNEMCount() const
{
  return layerDescriptors_.size();
}


size_t 
EMANE::ComponentMap::getNEMIndex(NEMId id) const
  throw(EMANE::ComponentMapException)
{
  NEMIdToIndexMapIter pos = nemIdToIndex_.find(id);
  if(pos == nemIdToIndex_.end())
    {
      std::stringstream e;
      e << "Invalid NEM Id ("
        << id
        << ")"
        << std::ends;

      throw ComponentMapException(e.str());
    }
  return pos->second;
}


EMANE::NEMId 
EMANE::ComponentMap::getNEMId(size_t index) const
  throw(EMANE::ComponentMapException)
{
  NEMIndexToIdMapIter pos = nemIndexToId_.find(index);
  if(pos == nemIndexToId_.end())
    {
      std::stringstream e;
      e << "Invalid NEM index ("
        << index
        << ")"
        << std::ends;

      throw ComponentMapException(e.str());
    }
  return pos->second;
}


size_t 
EMANE::ComponentMap::getLayerCount(size_t nemindex) const
{
  return layerDescriptors_[nemindex].size();
}


EMANE::NEMLayerDescriptor 
EMANE::ComponentMap::getLayerDescriptor(size_t nemindex, size_t layerindex) const
{
  return layerDescriptors_[nemindex][layerindex];
}


bool EMANE::ComponentMap::isRegisteredNEMLayerId(BuildId nemlayerbid)
{
  return nemlayermap_.find(nemlayerbid) != nemlayermap_.end();
}


bool 
EMANE::ComponentMap::isRegisteredNEMId(BuildId nembid)
{
  return nemmap_.find(nembid) != nemmap_.end();
}


bool 
EMANE::ComponentMap::isRegisteredPlatformId(BuildId platformbid)
{
  return platformmap_.find(platformbid) != platformmap_.end();
}


void EMANE::ComponentMap::buildtables()
{
  for(PlatformMap::const_iterator pi = platformmap_.begin();
      pi != platformmap_.end();
      ++pi)
    {
      size_t nemindex=0;
      std::vector<NEMEntry*> ne = pi->second->getChildEntries();
      for(std::vector<NEMEntry*>::const_iterator ni = ne.begin();
          ni != ne.end();
          ++ni)
        {
          NEMId nemid = (*ni)->getId();

          layerDescriptors_.push_back(NEMLayerDescriptorVector());
          nemIdToIndex_[nemid] = nemindex;
          nemIndexToId_[nemindex] = nemid;

          std::vector<NEMLayerEntry*> nle = (*ni)->getChildEntries();
          for(std::vector<NEMLayerEntry*>::const_iterator nli=nle.begin(); 
              nli != nle.end();
              ++nli)
            {
              layerDescriptors_[nemindex].push_back(
                             NEMLayerDescriptor((*nli)->getBuildId(),
                                                (*nli)->getType()));
            }
          nemindex++;
        }
    }
}


bool 
EMANE::ComponentMap::removeUnassignedId(EMANE::BuildId bid)
{
  UnassignedIdListSize startsize = unassignedIdList_.size();
  unassignedIdList_.remove_if(EMANE::ispresent(bid));
  return unassignedIdList_.size() < startsize;
}
