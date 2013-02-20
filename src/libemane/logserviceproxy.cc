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

#include "logserviceproxy.h"

EMANE::LogServiceProxy::LogServiceProxy(LogServiceProvider * pLogServiceProvider, 
                                        LogLevel level):
  pLogServiceProvider_(pLogServiceProvider),
  level_(level){};

EMANE::LogServiceProxy::~LogServiceProxy(){};

void EMANE::LogServiceProxy::vlog(LogLevel level, const char *fmt, va_list ap)
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
      pLogServiceProvider_->vlog(level,fmt,ap);
    }
}

void EMANE::LogServiceProxy::log(LogLevel level, const char *fmt, ...)
{
  // See above: Note on synchronization
  if(level <= level_)
    {
      va_list ap;
      
      va_start(ap, fmt);
      
      pLogServiceProvider_->vlog(level,fmt,ap);   
      
      va_end(ap);
    }
}

void EMANE::LogServiceProxy::setLogLevel(LogLevel level)
{
  // See above: Note on synchronization
  level_ = level;
}
