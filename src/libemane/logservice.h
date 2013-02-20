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

#ifndef EMANELOGSERVICE_HEADER_
#define EMANELOGSERVICE_HEADER_

#include "logserviceproxy.h"

#include "emane/emanelogserviceprovider.h"
#include "emane/emanecomponenttypes.h"

#include "emaneutils/componenttypes.h"

#include <map>

#include <ace/SOCK_Dgram.h>
#include <ace/Singleton.h>
#include <ace/Null_Mutex.h>
#include <ace/Thread.h>

namespace EMANE
{
  /**
   * @class LogService
   *
   * @brief Platform log service
   *
   * @details Handles all logging tasks at the platform
   * level
   *
   */
  class LogService : public LogServiceProvider
  {
  public:
    LogService();
    ~LogService();

    void log(LogLevel level,const char *format, ...)
      __attribute__ ((format (printf, 3, 4)));

    void vlog(LogLevel level,const char *format,va_list ap);

    void setLogLevel(LogLevel level);

    void redirectLogsToSysLog(const ACE_TCHAR *program);

    void redirectLogsToFile(const char* file); 

    void redirectLogsToRemoteLogger(const ACE_TCHAR *program, const char* addr);
    
    void registerProxy(EMANEUtils::NEMId id, LogServiceProxy *, ComponentType type);

    void open(const ACE_INET_Addr & logControlAddr);

  private:
    /* logger to component mapping */
    typedef std::multimap<ComponentType, LogServiceProxy *> ComponentTypeLogServiceProxyMMap;
    typedef ComponentTypeLogServiceProxyMMap::iterator ComponentTypeLogServiceProxyMMapIter;
    typedef std::pair<ComponentTypeLogServiceProxyMMapIter, ComponentTypeLogServiceProxyMMapIter> ComponentTypeLogServiceProxyMMapIterPair;

    /* component(s) logger to nem mapping */
    typedef std::map<EMANEUtils::NEMId, ComponentTypeLogServiceProxyMMap> NemComponentTypeLogServiceProxyMap;
    typedef NemComponentTypeLogServiceProxyMap::iterator NemComponentTypeLogServiceProxyMapIter;

    ACE_thread_t controlThread_;
    ACE_SOCK_Dgram udp_;
    LogLevel level_;
    bool bOpenControl_;
    bool bACELogging_;

    NemComponentTypeLogServiceProxyMap nemComponentTypeLogServiceProxyMap_;

    ACE_INET_Addr addr_;
    ACE_THR_FUNC_RETURN processControlMessages();
  };

  typedef ACE_Unmanaged_Singleton<LogService,ACE_Null_Mutex> LogServiceSingleton;
}

#endif //EMANELOGSERVICE_HEADER_
