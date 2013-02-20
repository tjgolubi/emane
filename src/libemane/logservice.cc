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

#include "logservice.h"
#include "logmessage.h"
#include "logserviceproxy.h"

#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/recvcancelable.h"

#include <cstdarg>
#include <cstdio>

#include <ace/OS_NS_sys_time.h>
#include <ace/OS_NS_string.h>
#include <ace/OS_NS_Thread.h>
#include <ace/OS_NS_time.h>
#include <ace/Log_Msg.h>
#include <ace/streams.h>

namespace
{
  const char * LEVELSTRING[] = 
    { 
      "NONE",  // 0
      "ABORT", // 1
      "ERROR", // 2
      "STATS", // 3
      "DEBUG", // 4
    };
}

#define TOTAL_LEVELS (static_cast<int>(sizeof(LEVELSTRING) / sizeof(char *)))
#define MAX_LEVEL    (TOTAL_LEVELS - 1)

EMANE::LogService::LogService():
  controlThread_(0),
  level_(ABORT_LEVEL),
  bOpenControl_(false),
  bACELogging_(false)
{
};

EMANE::LogService::~LogService()
{
  if(bOpenControl_)
    {
      ACE_OS::thr_cancel(controlThread_);

      ACE_OS::thr_join(controlThread_,0,0);
    }

  for(NemComponentTypeLogServiceProxyMapIter nemIter = nemComponentTypeLogServiceProxyMap_.begin();
      nemIter != nemComponentTypeLogServiceProxyMap_.end();
      ++nemIter)
   {
     for(ComponentTypeLogServiceProxyMMapIter mmIter = nemIter->second.begin();
         mmIter != nemIter->second.end();
         ++mmIter)
      {
        delete mmIter->second;
      }
   }
};

void EMANE::LogService::log(LogLevel level, const char *fmt, ...)
{
  /*
   * Note on synchronization:
   *
   *  There is no synchronization mechanism used to protect the level_
   *  attribute.  This variable is ~virtually~ constant, meaning that
   *  even if it is dynamically changed when compared to the amount of 
   *  times it is read it is as if it is always fixed.  Originally 
   *  an rwlock was employed for synchronization but profiling showed that
   *  it was extremely costly (think logs in the packet go path).
   *
   *  Worst case scenario is that either a handful of logs will be 
   *  displayed that should not or a few that should won't.
   */
  if(level <= level_) 
    {
      va_list ap;
      
      va_start(ap, fmt);
      
      vlog(level,fmt,ap);
      
      va_end(ap);
    }
}

void EMANE::LogService::vlog(LogLevel level, const char *fmt, va_list ap)
{
  // See above: Note on synchronization
  
  if(level <= level_) 
    {
      ACE_Time_Value tv = ACE_OS::gettimeofday();
      
      ACE_TCHAR buff[1024];
      size_t len = 0;

      struct tm ltm;
      
      time_t t = tv.sec();
      
      ACE_OS::localtime_r(&t,&ltm);
      
      len = snprintf(buff,sizeof(buff),"%02d:%02d:%02d.%06lu %5s ",
                     ltm.tm_hour,
                     ltm.tm_min,
                     ltm.tm_sec,
                     tv.usec(),
                     level >= 0 && level < TOTAL_LEVELS ? LEVELSTRING[level] : "?");
      
      len += vsnprintf(buff + len ,sizeof(buff) - len,fmt,ap);
   
      if(bACELogging_)
      {
        // Map the EMANE Log Level to the appropriate ACE Log Level
        switch(level)
        {
         case ABORT_LEVEL:   
           ACE_DEBUG((LM_CRITICAL, ACE_TEXT("%s\n"),buff));
           break;
         case ERROR_LEVEL:
           ACE_DEBUG((LM_ERROR, ACE_TEXT("%s\n"),buff));
           break;
         case STATS_LEVEL:
           ACE_DEBUG((LM_INFO, ACE_TEXT("%s\n"),buff));
           break;
         case DEBUG_LEVEL:
           ACE_DEBUG((LM_DEBUG, ACE_TEXT("%s\n"),buff));
           break;
         default:
           ACE_DEBUG((LM_ERROR, ACE_TEXT("Log Message wasn't a valid log level\n")));
           break;
        }
      }
      else
      {
        printf("%s\n",buff);
      }
      

    }
}

void EMANE::LogService::redirectLogsToSysLog(const ACE_TCHAR *program)
{
  if(ACE_LOG_MSG->open(program, ACE_Log_Msg::SYSLOG, NULL) == -1)
  {
   ACE_DEBUG((LM_ERROR, ACE_TEXT ("SystemLogging Failed: %s\n"),ACE_OS::strerror(errno)));
  }
  else
  {
    bACELogging_ = true;
  }
}

void EMANE::LogService::redirectLogsToFile(const char* file)
{
  ofstream *log_stream_;
  ACE_OSTREAM_TYPE * outputStream;

  log_stream_ = new ofstream();
  log_stream_->open(file, ios::out | ios::app);
  outputStream = ((ACE_OSTREAM_TYPE *)log_stream_);
  ACE_LOG_MSG->msg_ostream(outputStream);
  ACE_LOG_MSG->clr_flags(ACE_Log_Msg::STDERR | ACE_Log_Msg::LOGGER);
  ACE_LOG_MSG->set_flags(ACE_Log_Msg::OSTREAM);

  bACELogging_ = true;
}

void EMANE::LogService::redirectLogsToRemoteLogger(const ACE_TCHAR *program, const char* addr)
{
  if(ACE_LOG_MSG->open(program, ACE_Log_Msg::LOGGER,ACE_TEXT (addr)) == -1)
  {
    ACE_DEBUG((LM_ERROR, ACE_TEXT ("RemoteLogging Failed: %s\n"),ACE_OS::strerror(errno)));
  }
  else
  {
    bACELogging_ = true;
  }  
}

void EMANE::LogService::setLogLevel(LogLevel level)
{
  // See above: Note on synchronization
  level_ = level;
}

void  EMANE::LogService::registerProxy(EMANEUtils::NEMId id, LogServiceProxy * proxy, EMANE::ComponentType type)
{
  ComponentTypeLogServiceProxyMMap mmap;

  mmap.insert(std::make_pair(type, proxy));
  
  nemComponentTypeLogServiceProxyMap_.insert(std::make_pair(id, mmap));

  proxy->setLogLevel(level_);
}


void EMANE::LogService::open(const ACE_INET_Addr & logControlAddr)
{
  if(!bOpenControl_)
    {
      udp_.open(logControlAddr,ACE_PROTOCOL_FAMILY_INET,0,1);
  
      EMANEUtils::spawn(*this,&EMANE::LogService::processControlMessages,&controlThread_);
      
      bOpenControl_ = true;
    }
}

ACE_THR_FUNC_RETURN EMANE::LogService::processControlMessages(void)
{
  ACE_UINT8 buf[65536];
  ssize_t len = 0;
  
  LogMessageHeader * pLogMessageHeader = 0;
  
  while(1)
    {
      if((len =  EMANEUtils::recvCancelable(udp_,buf,sizeof(buf),addr_)) > 0)
        {
          size_t messageLength = len;

          if(messageLength >= sizeof(LogMessageHeader))
            {
              pLogMessageHeader = logMessageHeaderToHost(reinterpret_cast<LogMessageHeader *>(buf));

              switch(pLogMessageHeader->u16Id_)
                {
                case LOG_COMMAND_LEVEL_CONTROL:

                  if(pLogMessageHeader->u16Length_ == sizeof(LogLevelControl))
                    {
                      LogLevelControl * pLevelControl =
                        logMessageToHost(reinterpret_cast<LogLevelControl *>(pLogMessageHeader->data_));

                      // See above: Note on synchronization
                      if(pLevelControl->u16Level_ > MAX_LEVEL)
                        {
                          level_ = static_cast<EMANE::LogLevel>(MAX_LEVEL);
                        }
                      else
                        {
                          level_ = static_cast<EMANE::LogLevel>(pLevelControl->u16Level_);
                        }

                      // for all nems
                      for(NemComponentTypeLogServiceProxyMapIter nemIter = nemComponentTypeLogServiceProxyMap_.begin();
                          nemIter != nemComponentTypeLogServiceProxyMap_.end();
                          ++nemIter)
                       {
                         // for all components within the nem
                         for(ComponentTypeLogServiceProxyMMapIter mmIter = nemIter->second.begin();
                             mmIter != nemIter->second.end();
                             ++mmIter)
                          {
                            mmIter->second->setLogLevel(level_);
                          }

                         // TODO set log level for specific component type(s)
                       }
                    }
                  break;

                case NEM_LOG_COMMAND_LEVEL_CONTROL:
                  
                  if(pLogMessageHeader->u16Length_ == sizeof(NEMLogLevelControl))
                    {
                      NEMLogLevelControl * pNEMLevelControl =
                        NEMLogMessageToHost(reinterpret_cast<NEMLogLevelControl *>(pLogMessageHeader->data_));

                      const NemComponentTypeLogServiceProxyMapIter nemIter = 
                            nemComponentTypeLogServiceProxyMap_.find(pNEMLevelControl->u16NEMId_);

                      // for this nem
                      if(nemIter != nemComponentTypeLogServiceProxyMap_.end())
                        {
                         // for all components within the nem
                         for(ComponentTypeLogServiceProxyMMapIter mmIter = nemIter->second.begin();
                             mmIter != nemIter->second.end();
                             ++mmIter)
                          {
                            // See above: Note on synchronization
                            if(pNEMLevelControl->u16Level_ > MAX_LEVEL)
                              {
                                mmIter->second->setLogLevel(static_cast<EMANE::LogLevel>(MAX_LEVEL));
                              }
                            else
                              {
                                mmIter->second->setLogLevel(static_cast<EMANE::LogLevel>(pNEMLevelControl->u16Level_));
                              }
                             // TODO set log level for specific component type(s)
                           }
                        }
                    }
                  break;
                  
                default:
                  break;
                }
            }
        }
      else
        {
          // socket read error
          break;
        }
    }
  
  return 0;
}
