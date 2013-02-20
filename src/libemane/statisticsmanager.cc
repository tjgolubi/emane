/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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
#include "statisticsmanager.h"
#include "logservice.h"
#include <sstream>


EMANE::StatisticsManager::StatisticsManager(){}
 

EMANE::StatisticsManager::~StatisticsManager(){}


void 
EMANE::StatisticsManager::registerStatistic(EMANE::BuildId bid,
                                            std::string name, 
                                            EMANE::Statistic *value)
{
  statmap_[bid][name] = value;
}


void 
EMANE::StatisticsManager::unregisterStatistic(EMANE::BuildId bid,
                                              std::string name)
{
  StatMapIter si = statmap_.find(bid);

  if(si != statmap_.end())
    {
      StatEntryIter sei = si->second.find(name);

      if(sei != si->second.end())
        {
          si->second.erase(sei);
        }
    }
}


EMANE::returnList 
EMANE::StatisticsManager::getPlatformStatistics()
{
  returnList retlist;
  for(size_t i=0;
      i<ComponentMapSingleton::instance()->getNEMCount();
      i++)
    {
      NEMId nemid = ComponentMapSingleton::instance()->getNEMId(i);
      std::stringstream s;
      s << nemid;
      retlist.push_back(StatValue("NEM", s.str()));
      returnList tmp = getNEMStatistics(nemid);
      retlist.splice(retlist.end(), tmp);
    }
  return retlist;
}  


EMANE::returnList 
EMANE::StatisticsManager::getNEMStatistics(NEMId id)
{
  returnList retlist;

  if(ComponentMapSingleton::instance()->isValid(id))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      for(size_t i=0;
          i<ComponentMapSingleton::instance()->getLayerCount(nemindex);
          ++i)
        {
          returnList tmp = getIndexStatistics(id, i);
          retlist.splice(retlist.end(), tmp);
        }
    }

  return retlist;
}


EMANE::returnList 
EMANE::StatisticsManager::getIndexStatistics(NEMId id, size_t layerindex)
{
  returnList retlist;

  if(ComponentMapSingleton::instance()->isValid(id, layerindex))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      NEMLayerDescriptor ld = 
        ComponentMapSingleton::instance()->getLayerDescriptor(nemindex, 
                                                              layerindex);
      std::stringstream s;
      s << layerindex;
      retlist.push_back(StatValue("Index", s.str()));
      retlist.push_back(StatValue("Layer Type", 
                                  componentTypeToStr(ld.type)));
      returnList tmp = getStatistics(ld.buildId);
      retlist.splice(retlist.end(), tmp);
      retlist.push_back(StatValue("",""));
    }
  return retlist;
}


EMANE::returnList 
EMANE::StatisticsManager::getLayerStatistics(NEMId id, ComponentType type)
{
  returnList retlist;

  if(ComponentMapSingleton::instance()->isValid(id))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      for(size_t i=0; 
          i<ComponentMapSingleton::instance()->getLayerCount(nemindex);
          ++i)
        {      
          NEMLayerDescriptor ld = 
            ComponentMapSingleton::instance()->getLayerDescriptor(nemindex, i);

          if(ld.type == type)
            {
              returnList tmp;
              tmp = getIndexStatistics(id, i);
              retlist.splice(retlist.end(), tmp);
            }
        }
    }

  return retlist;
}


EMANE::returnList 
EMANE::StatisticsManager::getStatistic(std::string name,
                                       NEMId id,
                                       size_t index)
{
  if(ComponentMapSingleton::instance()->isValid(id, index))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      NEMLayerDescriptor ld = 
        ComponentMapSingleton::instance()->getLayerDescriptor(nemindex, index);

      return getStatistic(ld.buildId, name);
    }

  return returnList();
}


void 
EMANE::StatisticsManager::clearPlatformStatistics()
{
  for(StatMapIter smi = statmap_.begin();
      smi != statmap_.end(); 
      ++smi)
    {
      clearStatistics(smi->first);
    }
}


void 
EMANE::StatisticsManager::clearNEMStatistics(NEMId id)
{
  if(ComponentMapSingleton::instance()->isValid(id))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      returnList retlist;
      for(size_t i=0;
          i<ComponentMapSingleton::instance()->getLayerCount(nemindex);
          ++i)
        {
          NEMLayerDescriptor ld = 
            ComponentMapSingleton::instance()->getLayerDescriptor(nemindex, i);
          clearStatistics(ld.buildId);
        }
    }
}


void 
EMANE::StatisticsManager::clearIndexStatistics(NEMId id, size_t index)
{
  if(ComponentMapSingleton::instance()->isValid(id, index))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      NEMLayerDescriptor ld = 
        ComponentMapSingleton::instance()->getLayerDescriptor(nemindex, index);
      clearStatistics(ld.buildId);
    }
}


void 
EMANE::StatisticsManager::clearLayerStatistics(NEMId id, ComponentType type)
{
  if(ComponentMapSingleton::instance()->isValid(id))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      for(size_t i=0; 
          i<ComponentMapSingleton::instance()->getLayerCount(nemindex);
          ++i)
        {      
          NEMLayerDescriptor ld = 
            ComponentMapSingleton::instance()->getLayerDescriptor(nemindex, i);
          
          if(ld.type == type)
            {
              clearStatistics(ld.buildId);
            }
        }
    }
}


void 
EMANE::StatisticsManager::clearStatistic(std::string name,
                                         NEMId id,
                                         size_t index)
{
  if(ComponentMapSingleton::instance()->isValid(id, index))
    {
      size_t nemindex = ComponentMapSingleton::instance()->getNEMIndex(id);
      NEMLayerDescriptor ld = 
        ComponentMapSingleton::instance()->getLayerDescriptor(nemindex, index);

      clearStatistic(ld.buildId, name);
    }
}


void 
EMANE::StatisticsManager::dumpStatisticsToLog()
{
  EMANE::returnList retlist;
  EMANE::returnList::iterator pos;

  retlist = getPlatformStatistics();

  for(pos = retlist.begin(); pos != retlist.end();pos++)
  {
    // remove pretty-printing separators
    if((*pos).name.size() > 0) 
      {
    LogServiceSingleton::instance()->log(STATS_LEVEL,"%s: %s: %s",__func__, (*pos).name.c_str(),(*pos).value.c_str()); 
      }
  }
}


EMANE::returnList 
EMANE::StatisticsManager::getStatistics(EMANE::BuildId bid) const
{
  returnList retlist;
  
  StatMapConstIter pos = statmap_.find(bid);
  if(pos != statmap_.end())
    {
      for(StatEntryConstIter sei = pos->second.begin();
          sei != pos->second.end();
          ++sei)
        {
          retlist.push_back(StatValue(sei->first, sei->second->toString()));
        }
    }

  return retlist;
}


EMANE::returnList 
EMANE::StatisticsManager::getStatistic(EMANE::BuildId bid, 
                                       std::string name) const
{
  returnList retlist;

  StatMapConstIter pos = statmap_.find(bid);
  if(pos != statmap_.end())
    {
      StatEntryConstIter sei = pos->second.find(name);
      if(sei != pos->second.end())
        {
          retlist.push_back(StatValue(sei->first, sei->second->toString()));
        }
    }

  return retlist;
}


void 
EMANE::StatisticsManager::clearStatistics(EMANE::BuildId bid)
{
  StatMapIter pos = statmap_.find(bid);
  if(pos != statmap_.end())
    {
      for(StatEntryIter sei = pos->second.begin();
          sei != pos->second.end();
          ++sei)
        {
          sei->second->clear();
        }
    }
}


void 
EMANE::StatisticsManager::clearStatistic(EMANE::BuildId bid, 
                                         std::string name)
{
  StatMapIter pos = statmap_.find(bid);
  if(pos != statmap_.end())
    {
      StatEntryIter sei = pos->second.find(name);
      if(sei != pos->second.end())
        {
          sei->second->clear();
        }
    }
}


EMANE::returnList 
EMANE::StatisticsManager::getIndexMapping(NEMId id)
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"getIndexMapping(%d)",id); 

  returnList retlist;

  if(ComponentMapSingleton::instance()->isValid(id))
    {
      size_t nemindex =
        ComponentMapSingleton::instance()->getNEMIndex(id);
      std::stringstream s;
      s << id;
      retlist.push_back(StatValue("NEM", s.str()));
      for(size_t i=0;
          i<ComponentMapSingleton::instance()->getLayerCount(nemindex);
          i++)
        {
          retlist.push_back(StatValue("LAYER","INDEX"));
          returnList tmp;
          tmp = walkLayers(nemindex);
          retlist.splice(retlist.end(), tmp);
          retlist.push_back(StatValue("",""));
        }
    }
  else
    {
      for(size_t i=0;
          i<ComponentMapSingleton::instance()->getNEMCount();
          i++)
        {
          NEMId nemid =
            ComponentMapSingleton::instance()->getNEMId(i);

          std::stringstream s;
          s << nemid;
          retlist.push_back(StatValue("NEM",s.str()));
          retlist.push_back(StatValue("LAYER","INDEX"));
          returnList tmp;
          tmp = walkLayers(i);
          retlist.splice(retlist.end(), tmp);
          retlist.push_back(StatValue("",""));
        }
    }
  return retlist;
}


EMANE::returnList
EMANE::StatisticsManager::walkLayers(size_t nemindex) const
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,"walkLayers(%u)",
                                       static_cast<unsigned int>(nemindex)); 

  returnList retlist;
  
  for(size_t i=0;
      i < ComponentMapSingleton::instance()->getLayerCount(nemindex);
      ++i)
    {
      NEMLayerDescriptor nld = 
        ComponentMapSingleton::instance()->getLayerDescriptor(nemindex,i);
      std::stringstream s;
      s << i;
      retlist.push_back(StatValue(componentTypeToStr(nld.type),
                                   s.str()));
    }

  return retlist;
}


std::string 
EMANE::StatisticsManager::componentTypeToStr(ComponentType type) const
{
  std::string retVal;

  if (type == COMPONENT_PHYILAYER)
    {
      retVal = "phy";
    }
  else if (type == COMPONENT_MACILAYER)
    {
      retVal = "mac";
    }
  else if (type == COMPONENT_SHIMILAYER)
    {
      retVal = "shim";
    }
  if (type == COMPONENT_PHYLAYER)
    {
      retVal = "phy";
    }
  else if (type == COMPONENT_MACLAYER)
    {
      retVal = "mac";
    }
  else if (type == COMPONENT_SHIMLAYER)
    {
      retVal = "shim";
    }
  return retVal;
}
