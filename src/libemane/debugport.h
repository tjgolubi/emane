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

#ifndef EMANEDEBUGPORT_HEADER_
#define EMANEDEBUGPORT_HEADER_

#include "statisticsmanager.h"

#include <string>
#include <vector>

#include <ace/Thread_Mutex.h>
#include <ace/SOCK_Stream.h>

namespace EMANE
{
 /**
  *   
  * @class DebugPort
  *
  * @brief Debug port
  *
  */
  class DebugPort
  {
  public:
   /**
    *
    * Constructor
    *
    * @param u16Port debug port number
    * @param statmanager pointer to the StatisticsManager
    *
    */
    DebugPort(ACE_UINT16 u16Port, StatisticsManager* statmanager);
 
   /**
    * 
    * Destructor
    *
    */
    ~DebugPort();

   /**
    * 
    * start method
    *
    */
    void start();

  private:
    typedef std::vector<std::string> CommandArguements;
    typedef std::vector<std::string> CommandHistory;

    ACE_THR_FUNC_RETURN telnetServer();
    void getStats(CommandArguements arguments);
    void clearStats(CommandArguements arguments);
    void sendPrompt();
    void sendOptionMessage(ACE_UINT8 u8Code, ACE_UINT8 u8Option);
    void sendASCIIData(const char *fmt,...);
    int  processArguements(CommandArguements arguments);
    void displayHelp(CommandArguements arguments);
    void processOptionCode(ACE_UINT8 u8Data);
    CommandArguements parse(const std::string &sCmdLine);

    ACE_UINT16     u16Port_;
    CommandHistory commandHistory_;  
    ACE_SOCK_Stream peer_;
    ACE_thread_t thread_;
    bool bConnected_;
    ACE_Thread_Mutex mutex_;
    StatisticsManager* statsManager_;  
    bool bCommandMode;
    bool bOptionMode;
    bool bEcho;
    ACE_UINT8 u8OptionCode;
  };
}

#endif
