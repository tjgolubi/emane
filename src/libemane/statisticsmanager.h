/*
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANESTATISTICMANAGER_HEADER_
#define EMANESTATISTICMANAGER_HEADER_

#include "emane/emanetypes.h"
#include "emane/emanecomponenttypes.h"
#include "emane/emanestatistic.h"
#include <string>
#include <map>
#include <list>

#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>

namespace EMANE
{
  /**
   * @struct StatValue
   *
   * @brief A structure containing a name and value string members to hold
   * the name and current value of a registered statistic.
   */
  struct StatValue
  {
    std::string name;
    std::string value;
    StatValue(const std::string n, const std::string v) : name(n), value(v) {}
  };

  typedef std::list<StatValue> returnList;

  typedef std::map<std::string, EMANE::Statistic *> StatEntry;
  typedef StatEntry::iterator StatEntryIter;
  typedef StatEntry::const_iterator StatEntryConstIter;

  typedef std::map<BuildId, StatEntry> StatMap;
  typedef StatMap::iterator StatMapIter;
  typedef StatMap::const_iterator StatMapConstIter;

/**
 *
 * @class  StatisticsManager
 *
 * @brief Defines the Statistics Manager.
 * Allows for registering and unregistering statistics and methods for polling and clearing statistics.
 *
 */
  class StatisticsManager
  {
  public:
    StatisticsManager();
    ~StatisticsManager();

    void registerStatistic(BuildId bid, 
                           std::string name, 
                           EMANE::Statistic *value);
    void unregisterStatistic(BuildId,
                             std::string name);

    returnList getPlatformStatistics();
    returnList getNEMStatistics(NEMId id);
    returnList getIndexStatistics(NEMId id, size_t index);
    returnList getLayerStatistics(NEMId id, ComponentType type);
    returnList getStatistic(std::string name, NEMId id, size_t index);
    returnList getIndexMapping(NEMId id);
    void clearPlatformStatistics();
    void clearNEMStatistics(NEMId id);
    void clearIndexStatistics(NEMId id, size_t index);
    void clearLayerStatistics(NEMId id, ComponentType type);
    void clearStatistic(std::string name, NEMId id, size_t index);
    void dumpStatisticsToLog();

  private:
    StatMap statmap_;

    returnList getStatistics(BuildId bid) const;
    returnList getStatistic(BuildId bid, std::string name) const;
    void clearStatistics(BuildId bid);
    void clearStatistic(BuildId bid, std::string name);
    returnList walkLayers(size_t nemindex) const;
    std::string componentTypeToStr(ComponentType type) const;

  };
  typedef ACE_Unmanaged_Singleton<StatisticsManager,ACE_Null_Mutex> StatisticsManagerSingleton;
}

#endif
