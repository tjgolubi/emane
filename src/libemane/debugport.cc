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
#include "debugport.h"
#include <emane/emanestatistic.h>
#include "logservice.h"

#include <emaneutils/spawnmemberfunc.h>

#include <ace/SOCK_Acceptor.h>
#include <stdlib.h>
#include <ace/Thread_Mutex.h>


namespace
{

  const ACE_UINT8 TELNET_CODE_IAC                     = 0xff;
  const ACE_UINT8 TELNET_CODE_WILL                    = 0xfb;
  const ACE_UINT8 TELNET_CODE_WONT                    = 0xfc;
  const ACE_UINT8 TELNET_CODE_DO                      = 0xfd;
  const ACE_UINT8 TELNET_CODE_DONT                    = 0xfe;

  const ACE_UINT8 OPTION_SUPPRESS_GO_AHEAD            = 0x03;
  const ACE_UINT8 OPTION_TERMINAL_TYPE                = 0x18;
  const ACE_UINT8 OPTION_NEGOTIATE_WINDOW_SIZE        = 0x1f;
  const ACE_UINT8 OPTION_TERMINAL_SPEED               = 0x20;
  const ACE_UINT8 OPTION_TERMINAL_REMOTE_FLOW_CONTROL = 0x21;
  const ACE_UINT8 OPTION_TERMINAL_LINEMODE            = 0x22;
  const ACE_UINT8 OPTION_NEW_ENVIRONMENT              = 0x27;
  const ACE_UINT8 OPTION_STATUS                       = 0x05;
  const ACE_UINT8 OPTION_X_DISPLAY_LOCATION           = 0x23;
  const ACE_UINT8 OPTION_ECHO                         = 0x01;

  static const char * stats_usage()
  {
    return "stats [all|nem|layer|index|stat]";
  }

  static const char * clearstats_usage()
  {
    return "clearstats [all|nem|layer|index|stat]";
  }

  static const char * showstacks_usage()
  {
    return "showstacks [nem]";
  }
}

EMANE::DebugPort::DebugPort(ACE_UINT16 u16Port, 
                                                    EMANE::StatisticsManager *statmanager) :
  u16Port_(u16Port),
  statsManager_(statmanager),
  bCommandMode(false),
  bOptionMode(false),
  bEcho(false),
  u8OptionCode(0)
{}

EMANE::DebugPort::~DebugPort()
{}

void EMANE::DebugPort::start()
{
  EMANEUtils::spawn(*this,&EMANE::DebugPort::telnetServer, &thread_);
}

int EMANE::DebugPort::processArguements(CommandArguements arguments)
{
  int retVal = 0;

  if(arguments[0] == "stats")
  {
    getStats(arguments);
  }
  else if(arguments[0] == "clearstats")
  {
    clearStats(arguments);
  }
  else if(arguments[0] == "quit")
  {
    retVal = -1;
  }
  else if(arguments[0] == "exit")
  {
    retVal = -1;
  }
  else if(arguments[0] == "clear")
  {
    peer_.send_n("\r\x1b[2J\x1b[0;0H",11);
  }
  else if(arguments[0] == "showstacks")
  {
    EMANE::returnList::iterator pos;
    EMANE::returnList returnVal;
    NEMId Id = 0;
    bool helpNeeded = false;

    if(arguments.size() == 2)
      {
        if (arguments[1] == "help")
        {
          sendASCIIData("%s",showstacks_usage());
          helpNeeded = true;         
        }
        else
        {
          Id = strtol(arguments[1].c_str(),NULL,0);
        }
      }

    if(helpNeeded == false)
    {
      returnVal = statsManager_->getIndexMapping(Id);
  
      for(pos = returnVal.begin(); pos != returnVal.end();pos++)
      {
        sendASCIIData("%-15s %s",(*pos).name.c_str(),(*pos).value.c_str());
      }
    }
  }
  else if(arguments[0] == "version")
  {
    sendASCIIData("%s %s",PACKAGE,PACKAGE_VERSION);
  }
  else if(arguments[0] == "help")
  {
    displayHelp(arguments);
  }
  else
  {
    sendASCIIData("unknown command: %s",arguments[1].c_str());
    sendASCIIData("Try 'help'");
  }

  return retVal;
}

void EMANE::DebugPort::displayHelp(CommandArguements arguments)
{
  if(arguments.size() == 1)
  {
    sendASCIIData("");
    sendASCIIData("Form more information: `help <command>`");
    sendASCIIData("");
    sendASCIIData("command list:");
    sendASCIIData(" clear");
    sendASCIIData(" exit");
    sendASCIIData(" help");
    sendASCIIData(" stats");
    sendASCIIData(" clearstats");
    sendASCIIData(" showstacks");
    sendASCIIData(" quit");
    sendASCIIData(" version");
    sendASCIIData("");
  }
  else if(arguments[1] == "clear")
  {
    sendASCIIData("");
    sendASCIIData("command    : clear");
    sendASCIIData("description: Clear screen");
    sendASCIIData("usage      : clear");
    sendASCIIData("");
  }
  else if(arguments[1] == "exit")
  {
    sendASCIIData("");
    sendASCIIData("command    : exit");
    sendASCIIData("description: Exit telnet session");
    sendASCIIData("usage      : exit");
    sendASCIIData("");
  }
  else if(arguments[1] == "quit")
  {
    sendASCIIData("");
    sendASCIIData("command    : quit");
    sendASCIIData("description: Exit telnet session");
    sendASCIIData("usage      : quit");
    sendASCIIData("");
  }
  else if(arguments[1] == "help")
  {
    sendASCIIData("");
    sendASCIIData("command    : help");
    sendASCIIData("description: Print comand usage and descripition");
    sendASCIIData("usage      : help | help <command>");
    sendASCIIData("");
  }
  else if(arguments[1] == "version")
  {
    sendASCIIData("");
    sendASCIIData("command    : version");
    sendASCIIData("description: Print version information");
    sendASCIIData("usage      : version");
    sendASCIIData("");
  }
  else if(arguments[1] == "whoami")
  {
    sendASCIIData("");
    sendASCIIData("command    : whoami");
    sendASCIIData("description: Print NEM id");
    sendASCIIData("usage      : whoami");
    sendASCIIData("");
  }
  else if(arguments[1] == "stats")
  {
    sendASCIIData("");
    sendASCIIData("command    : stats");
    sendASCIIData("description: Show statistics");
    sendASCIIData("usage      : %s",stats_usage());
    sendASCIIData("");
  }
  else if(arguments[1] == "clearstats")
  {
    sendASCIIData("");
    sendASCIIData("command    : clearstats");
    sendASCIIData("description: Clear statistics");
    sendASCIIData("usage      : %s",clearstats_usage());
    sendASCIIData("");
  }
  else if(arguments[1] == "showstacks")
  {
    sendASCIIData("");
    sendASCIIData("command    : showstacks");
    sendASCIIData("description: Show NEM stack");
    sendASCIIData("usage      : %s",showstacks_usage());
    sendASCIIData("");
  }
  else
  {
    sendASCIIData("unknown command: %s",arguments[1].c_str());
    sendASCIIData("Try 'help'");
  }
}



ACE_THR_FUNC_RETURN EMANE::DebugPort::telnetServer()
{
  ACE_INET_Addr endpointAddr(u16Port_);

  ACE_SOCK_Acceptor acceptor;
  ACE_SOCK_Stream peer;

  if(acceptor.open(endpointAddr,1) == -1)
    {
     EMANE::LogServiceSingleton::instance()->log(EMANE::ERROR_LEVEL, "DebugPort::telnetServer acceptor open");
      return (ACE_THR_FUNC_RETURN) 1;
    } 

  while(1)
    {
      int len;
      std::string sCommandLine;
      bool bEscapeSequence1  = false;
      bool bEscapeSequence2  = false;
      ACE_UINT8 u8Data       = 0;
      int iCommandHistoryIndex = -1;

      commandHistory_.clear();
      
      if(acceptor.accept(peer) == -1)
        {
          EMANE::LogServiceSingleton::instance()->log(EMANE::ERROR_LEVEL, "DebugPort::telnetServer acceptor accept");
          break;
        }

      mutex_.acquire();
      bConnected_ = true;
      peer_       = peer;

      mutex_.release();

      sendOptionMessage(TELNET_CODE_WILL,OPTION_SUPPRESS_GO_AHEAD);
      sendOptionMessage(TELNET_CODE_WILL,OPTION_ECHO);

      sendPrompt();

      while((len = peer_.recv(&u8Data,sizeof(u8Data))) > 0)
        {
          /*EMANE::LogServiceSingleton::instance()->log(EMANE::DEBUG_LEVEL,
              "read: %hhx (%hhu) com:%d opt:%d e1:%d e2:%d",
              u8Data,
              u8Data,
              bCommandMode,
              bOptionMode,
              bEscapeSequence1,
              bEscapeSequence2);*/

          if(!bCommandMode)
            {
              if(u8Data == 0xFF)
                {
                  bCommandMode = true;
                }
              else if (u8Data == '\n' || u8Data == 0)
                {
                  if(bEcho)
                    {
                      peer_.send_n("\r\n",2);
                    }
                  
                  if(!sCommandLine.empty())
                    {
                      CommandArguements arguments = parse(sCommandLine);
                      if(arguments.empty())
                        {
                          sendPrompt();
                        }
                      else
                        {
                          iCommandHistoryIndex = -1;

                          commandHistory_.push_back(sCommandLine);
                          if(processArguements(arguments) == -1)
                          {
                             break;
                          }
                        }
                    }
                    sCommandLine.clear();
                    sendPrompt();
                }
              else if(u8Data != '\r')
                {
                  bool bProcess = false;
                  
                  switch(u8Data)
                    {
                    case 0xc:
                      // clear 
                      peer_.send_n("\r\x1b[2J\x1b[0;0H",11);
                      sendPrompt();
                      break;

                    case 0x1b:
                      // escape seq char 1
                      bEscapeSequence1 = true;
                      break;

                    case 0x5b:
                      // escape seq char 2
                      if(bEscapeSequence1)
                        {
                          bEscapeSequence2 = true;
                          bEscapeSequence1 = false;
                        }
                      else
                        {
                          bProcess = true;
                        }

                      break;
                      
                    case 0x7f:
                      // backspace

                      if(!sCommandLine.empty())
                        {
                          sCommandLine.erase(sCommandLine.end() -1);
                          peer_.send_n("\b \b",3);
                        }

                      break;
                  
                    case 0x41:
                      // up arrow
                      if(bEscapeSequence2 == true)
                        {
                      if(!commandHistory_.empty())
                        {
                          if(iCommandHistoryIndex  == -1)
                            {
                              iCommandHistoryIndex = commandHistory_.size();
                            }
                          
                          if(iCommandHistoryIndex > 0)
                            {
                              --iCommandHistoryIndex;

                              sendPrompt();

                              peer_.send_n(commandHistory_[iCommandHistoryIndex].c_str(),
                                          commandHistory_[iCommandHistoryIndex].size());

                              sCommandLine = commandHistory_[iCommandHistoryIndex];
                            }
                        }
                        }
                      else
                        {
                          if(isprint(u8Data))
                            {
                              bProcess = true;
                            }
                        }
                      bEscapeSequence2 = false;
                      break;

                    case 0x42:
                      // down arrow
                      if(bEscapeSequence2 == true)
                        {
                      if(!commandHistory_.empty())
                        {
                          if(iCommandHistoryIndex  != -1)
                            {
                              
                              if(static_cast<size_t>(iCommandHistoryIndex) < commandHistory_.size() - 1)
                                {
                                  ++iCommandHistoryIndex;

                                  sendPrompt();

                                  peer_.send_n(commandHistory_[iCommandHistoryIndex].c_str(),
                                              commandHistory_[iCommandHistoryIndex].size());
                                  
                                  sCommandLine = commandHistory_[iCommandHistoryIndex];
                                }
                              else
                                {
                                  sendPrompt();
                                  sCommandLine.clear();
                                }
                            }
                        }
                        }
                      else
                        {
                          if(isprint(u8Data))
                            {
                              bProcess = true;
                            }
                        }
                      bEscapeSequence2 = false;
                      break;
                      
                    default:
                      bEscapeSequence1 = false;

                      if(bEscapeSequence2)
                        {
                          bEscapeSequence2 = false;
                        }
                      else
                        {
                          if(isprint(u8Data))
                            {
                              bProcess = true;
                            }
                        }

                      break;
                    }

                  if(bProcess)
                    {
                      sCommandLine.push_back(u8Data);
                      
                      if(bEcho)
                        {
                          peer_.send_n(&u8Data,1);
                        }

                      bEscapeSequence1 = false;
                      bEscapeSequence2 = false;
                    }
                  
                  continue;
                }
              else if(u8Data == '\r')
                {
                  continue;
                }
              else
                {
                  sendPrompt();
                }
            }
          else
            {
              processOptionCode(u8Data);
            }
        }
      mutex_.acquire();
      bConnected_ = false;
      peer_.close();
      mutex_.release();
    }
  return (ACE_THR_FUNC_RETURN) 0;
              
}

void EMANE::DebugPort::sendPrompt()
{
  char prompt[256];
  size_t len = snprintf(prompt,sizeof(prompt),"\r\x1b\x5bKdebugport %%%% ");
  peer_.send_n(prompt,len);
}

void EMANE::DebugPort::processOptionCode(ACE_UINT8 u8Data)
{
              if(!bOptionMode)
                {
                  switch(u8Data)
                    {
                    case 240:
                    case 241:
                    case 242:
                    case 243:
                    case 244:
                    case 245:
                    case 246:
                    case 247:
                    case 248:
                    case 249:
                    case 250:
                      
                      bCommandMode = false;
                      break;
                      
                    case 251:
                    case 252:
                    case 253:
                    case 254:
                      u8OptionCode = u8Data;
                      bOptionMode = true;
                      break;
                    }
                }
              else
                {
                  if(u8OptionCode == TELNET_CODE_WILL)
                    {
                      switch(u8Data)
                        {
                        case OPTION_SUPPRESS_GO_AHEAD:
                          sendOptionMessage(TELNET_CODE_DO,u8Data);
                          break;

                        case OPTION_TERMINAL_LINEMODE:
                          sendOptionMessage(TELNET_CODE_DONT,OPTION_TERMINAL_LINEMODE);
                          sendOptionMessage(TELNET_CODE_WILL,OPTION_ECHO);
                          break;

                        default:
                          sendOptionMessage(TELNET_CODE_DONT,u8Data);
                        }
                    }

                  if(u8OptionCode == TELNET_CODE_DO)
                    {
                      switch(u8Data)
                        {
                        case OPTION_SUPPRESS_GO_AHEAD:
                        case OPTION_ECHO:
                          sendOptionMessage(TELNET_CODE_WILL,u8Data);
                          bEcho = true;
                          break;
                        default:
                          sendOptionMessage(TELNET_CODE_WONT,u8Data);
                        }
                    }

                  if(u8OptionCode == TELNET_CODE_DONT)
                    {
                      switch(u8Data)
                        {
                        case OPTION_ECHO:
                          bEcho = false;
                          break;
                        case OPTION_TERMINAL_LINEMODE:
                          sendOptionMessage(TELNET_CODE_WONT,OPTION_TERMINAL_LINEMODE);
                          break;
                        }
                    }
                   
                  bOptionMode = false;
                  bCommandMode = false;
                  u8OptionCode = 0;
                }

}

void EMANE::DebugPort::sendOptionMessage(ACE_UINT8 u8Code, ACE_UINT8 u8Option)
{
  ACE_UINT8 buf[3] = {0xff,u8Code,u8Option};
  peer_.send_n(buf,3);
}

void EMANE::DebugPort::sendASCIIData(const char *fmt,...)
{
  char buff[256];

  va_list ap;

  va_start(ap, fmt);
  
  size_t length = vsnprintf(buff, sizeof(buff) - 2, fmt, ap);

  va_end(ap);      

  buff[length++] = '\r';
  buff[length++] = '\n';

  peer_.send_n(buff,length);
}

EMANE::DebugPort::CommandArguements 
EMANE::DebugPort::parse(const std::string &sCmdLine)
{
  CommandArguements argVector;
  size_t pos1 = 0;
  size_t pos2 = 0;

  while((pos2 = sCmdLine.find(" ",pos1)) != std::string::npos)
    {
      if(pos1 != pos2)
        {
          argVector.push_back(sCmdLine.substr(pos1,pos2 - pos1));
        }

      pos1 = pos2+1;
    }

  if(!sCmdLine.empty())
    {
      std::string s = sCmdLine.substr(pos1,std::string::npos);

      if(s.find_first_not_of(" ",0) != std::string::npos)
        {
          argVector.push_back(s);
        }
    }

  return argVector;  
}


void EMANE::DebugPort::getStats(CommandArguements arguments)
{
  // process stats call here
  EMANE::returnList returnVal;
  EMANE::returnList::iterator pos;
  EMANE::ComponentType type = COMPONENT_ALL;
  NEMId Id = 0;
  size_t index = 0;
  std::string name;


  if (arguments.size() > 1)
    {
      if(arguments[1] == "all")
        {
          returnVal = statsManager_->getPlatformStatistics();
        }
      else if(arguments[1] == "nem")
        {
          if(arguments.size() == 3)
          {
            Id = strtol(arguments[2].c_str(),NULL,0);
            returnVal = statsManager_->getNEMStatistics(Id);
          }
          else
          {
            sendASCIIData("stats nem nemvalue");
          }
        }
      else if(arguments[1] == "layer")
        {
          if(arguments.size() == 4)
          {
            Id = strtol(arguments[2].c_str(),NULL,0);
            if (arguments[3] == "phy")
            {
              type = COMPONENT_PHYILAYER;
            }
            else if (arguments[3] == "mac")
            {
              type = COMPONENT_MACILAYER;
            }
            else if (arguments[3] == "shim")
            {
             type = COMPONENT_SHIMILAYER;
            }
            returnVal = statsManager_->getLayerStatistics(Id,type);          
          }
          else
          {
            sendASCIIData("stats layer nemvalue layername");
          }
        }
      else if(arguments[1] == "stat")
        {
          if(arguments.size() == 5)
          {
            Id = strtol(arguments[2].c_str(),NULL,0);
            index = strtol(arguments[3].c_str(),NULL,0);
            name = arguments[4];
            returnVal = statsManager_->getStatistic(name,Id,index);
          }
          else
          {
            sendASCIIData("stats stat nemvalue indexvalue statname");
          }
        }
      else if(arguments[1] == "index")
        {
          if(arguments.size() == 4)
          {
            Id = strtol(arguments[2].c_str(),NULL,0);
            index = strtol(arguments[3].c_str(),NULL,0);
            returnVal = statsManager_->getIndexStatistics(Id,index);
          }
          else
          {
            sendASCIIData("stats index nemvalue indexvalue");
          }
        }
    }
  else
    {
      //stats usage
      sendASCIIData("%s",stats_usage());
    }

  for(pos = returnVal.begin(); pos != returnVal.end();pos++)
  {
    sendASCIIData("%-30s %s",(*pos).name.c_str(),(*pos).value.c_str());
  }
}

void EMANE::DebugPort::clearStats(CommandArguements arguments)
{
 EMANE::ComponentType type = COMPONENT_ALL;
  NEMId Id = 0;
  size_t index = 0;
  std::string name;


  if (arguments.size() > 1)
    {
      if(arguments[1] == "all")
        {
          statsManager_->clearPlatformStatistics();
        }
      else if(arguments[1] == "nem")
        {
          Id = strtol(arguments[2].c_str(),NULL,0);
          statsManager_->clearNEMStatistics(Id);
        }
      else if(arguments[1] == "layer")
        {
          if (arguments.size() > 3)
            {
              Id = strtol(arguments[2].c_str(),NULL,0);
              if (arguments[3] == "phy")
                {
                  type = COMPONENT_PHYILAYER;
                }
              else if (arguments[3] == "mac")
                {
                  type = COMPONENT_MACILAYER;
                }
              else if (arguments[3] == "shim")
                {
                  type = COMPONENT_SHIMILAYER;
                }
              statsManager_->clearLayerStatistics(Id,type);          
            }
        }
      else if(arguments[1] == "stat")
        {
          if (arguments.size() > 4)
            {
              Id = strtol(arguments[2].c_str(),NULL,0);
              index = strtol(arguments[3].c_str(),NULL,0);
              name = arguments[4];
              statsManager_->clearStatistic(name,Id,index);
            }
        }
      else if(arguments[1] == "index")
        {
          if (arguments.size() > 3)
            {
              Id = strtol(arguments[2].c_str(),NULL,0);
              index = strtol(arguments[3].c_str(),NULL,0);
              statsManager_->clearIndexStatistics(Id,index);
            }
        }
    }
  else
    {
      //stats usage
      sendASCIIData("%s",clearstats_usage());
    }

}
