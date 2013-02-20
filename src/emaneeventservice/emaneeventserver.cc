/*
 * Copyright (c) 2008-2013 - DRS CenGen, LLC, Columbia, Maryland
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

#include "libemane/eventgeneratorbuilder.h"
#include "libemane/logger.h"
#include "eventdirector.h"
#include "erroreventreceiver.h"

#include "emane/emaneexception.h"
#include "emane/emaneeventerrorevent.h"

#include "emaneutils/terminator.h"

#include <sstream>
#include <iostream>

#include <ace/Get_Opt.h>
#include <ace/Reactor.h>
#include <ace/Sched_Params.h>
#include <ace/OS_NS_time.h>
#include <ace/OS_NS_sys_time.h>
#include <ace/OS_NS_stdio.h>
#include <ace/OS_NS_unistd.h>
#include <ace/OS_NS_sys_uio.h>
#include <ace/OS_NS_fcntl.h>
#include <ace/ACE.h>

void usage();

int ACE_TMAIN(int argc, ACE_TCHAR * argv[])
{
  EMANE::Logger logger;

  try
    {
      const ACE_Time_Value currentTimeValue = ACE_OS::gettimeofday();

      const ACE_TCHAR options[] = ACE_TEXT("f:dhnrl:s:L:");

      std::string sConfigFile;

      ACE_Get_Opt cmd_opts(argc,argv,options);
  
      if(cmd_opts.long_option(ACE_TEXT("help"),'h') == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: help")),EXIT_FAILURE);
        }
      
      if(cmd_opts.long_option(ACE_TEXT("loglevel"),'l',ACE_Get_Opt::ARG_REQUIRED) == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("config long option: loglevel")),EXIT_FAILURE);
        }

      if(cmd_opts.long_option(ACE_TEXT("nextday"),'n') == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: nextday")),EXIT_FAILURE);
        }

      if(cmd_opts.long_option(ACE_TEXT("realtime"),'r') == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: realtime")),EXIT_FAILURE);
        }

      if(cmd_opts.long_option(ACE_TEXT("version"),'v') == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: version")),EXIT_FAILURE);
        }
 
      if(cmd_opts.long_option(ACE_TEXT("starttime"),'s', ACE_Get_Opt::ARG_REQUIRED) == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: starttime")),EXIT_FAILURE);
        }
      if(cmd_opts.long_option(ACE_TEXT("daemonize"),'d') == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: daemonize")),EXIT_FAILURE);
        }

      if(cmd_opts.long_option(ACE_TEXT("syslog"),1)== -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: syslog")),EXIT_FAILURE);
        }

      if(cmd_opts.long_option(ACE_TEXT("logfile"),'f',ACE_Get_Opt::ARG_REQUIRED) == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),ACE_TEXT("config long option: logfile")),EXIT_FAILURE);
        }

      if(cmd_opts.long_option(ACE_TEXT("logserver"),'L',ACE_Get_Opt::ARG_REQUIRED) == -1)
        {
          ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("config long option: logserver")),EXIT_FAILURE);
        }
      int option;

      EMANE::LogLevel level = EMANE::ERROR_LEVEL;

      bool bRealtime = false;
      bool bNextDay  = false;
      bool bDaemonize = false;
      bool bSysLog = false;
      std::string sLogServer;
      std::string sLogFile;

      // no wait time unless specified below
      ACE_Time_Value waitTime = ACE_Time_Value::zero;

      while((option = cmd_opts()) != EOF)
        {
          switch(option)
            {
            case 'l':
              // --level
              {
                int iLevel = atoi(cmd_opts.opt_arg());
                
                if(iLevel >= EMANE::NOLOG_LEVEL  && iLevel <= EMANE::DEBUG_LEVEL)
                  {
                    level = static_cast<EMANE::LogLevel>(iLevel);
                  }
                else
                  {
                    std::cerr<<"Bad log level"<<std::endl; 
                    return EXIT_FAILURE; 
                  }
              }
              break;
            case 'd':
              // --daemonize
              bDaemonize = true;
              break;
             
            case 'h':
              // --help
              usage();
              return 0;
      
            case 'r':
              // --realtime
              bRealtime = true;
              break;

            case 1:
              // --syslogging
              bSysLog = true;
              break;

            case 'L':
              // --logserver
              sLogServer = cmd_opts.opt_arg();
              break;

            case 'f':
              // --logfile
              sLogFile = cmd_opts.opt_arg();
              break;

            case 'n':
              // --nextday
              bNextDay = true;
              break;
        
            case 'v':
              // -- version
              std::cout<<VERSION<<std::endl;
              return EXIT_SUCCESS;

            case 's':
              // --starttime
              {
                unsigned hour, min, sec;
                if(sscanf(cmd_opts.opt_arg(), "%02u:%02u:%02u", &hour, &min, &sec) == 3)
                  {
                    // desired start time today in seconds
                    time_t startTimeSeconds = (hour * 3600) + (min * 60) + sec;

                    // our current absoulute time in seconds
                    time_t absoluteTimeSeconds = currentTimeValue.sec();

                    // current time broken down to mon,day,year,hr,min,sec ...
                    struct tm *tm_ptr = ACE_OS::localtime(&absoluteTimeSeconds);

                    // number of seconds passed so far today
                    time_t passedTimeSeconds = (tm_ptr->tm_hour * 3600) + (tm_ptr->tm_min * 60) + tm_ptr->tm_sec;

                    // has the start time has passed
                    if(startTimeSeconds < passedTimeSeconds)
                      {
                        // allow next day
                        if(bNextDay)
                          {
                            // add one day of seconds to start time
                            startTimeSeconds += 24 * 3600;
                          }
                        else
                          {
                            std::cerr<<"Start time "<<cmd_opts.opt_arg()<<" has past current time; use \"--nextday --starttime HH:MM:SS\" if you want to start after midnight."<<std::endl; 
                            return EXIT_FAILURE; 
                          }
                      }

                    // time to wait in seconds
                    waitTime.set(startTimeSeconds - passedTimeSeconds, 0);
                  }
                else
                  {
                    std::cerr<<"Error in format of start time "<<cmd_opts.opt_arg()<<" --starttime HH:MM:SS"<<std::endl; 
                    return EXIT_FAILURE; 
                  }
              }
              break;

            case ':':
              // missing arguement
              std::cerr<<"-"<<cmd_opts.opt_opt()<<"requires an argument"<<std::endl;
              return EXIT_FAILURE;
              
            default:
              std::cerr<<"Unknown option: "<<cmd_opts.last_option()<<std::endl;
              return EXIT_FAILURE;
            }
        }

      if(cmd_opts.opt_ind() < cmd_opts.argc())
        {
          sConfigFile = argv[cmd_opts.opt_ind()];
        }
      else
        {
          std::cerr<<"Missing CONFIG_URI"<<std::endl;
          return EXIT_FAILURE;
        }
     
      // create the event generator build director prior to daemonizing.
      // Part of the daemonization entails changing the current 
      // working directory.  If the CONFIG URI is a relative 
      // file path the change of directory will prevent finding
      // any included XML definitions.
      EMANE::EventGeneratorBuilder builder;

      EMANE::EventDirector director(sConfigFile, builder);

 
      if(bDaemonize) 
        {

          // Logging cannot be sent to STDERR while running daemonized 
          if(sLogServer.empty() && sLogFile.empty() && !bSysLog && level != 0)
            {
              logger.log(EMANE::ERROR_LEVEL,"Debug level needs to be set to 0 when stderr is the output log type and EMANE is running in the background");
              level =  EMANE::NOLOG_LEVEL;
            }
          if(ACE::daemonize(ACE_TEXT("/"),false) == -1) 
            {
              ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("emane:could not daemonize\n")),-1);
            }
        }
      // Redirect to the logtype specified.  Default is stderr
      if(bSysLog)
        {
          logger.redirectLogsToSysLog(argv[0]);
        }

      if(!sLogServer.empty())
        {
          logger.redirectLogsToRemoteLogger(argv[0],sLogServer.c_str());
        }

      if(!sLogFile.empty())
        {
          logger.redirectLogsToFile(sLogFile.c_str());
        }

      if(level > 0)
        {
          logger.setLogLevel(level);
        }
      else
        {
          logger.setLogLevel(EMANE::NOLOG_LEVEL);
        }


      ACE_Sched_Params params(ACE_SCHED_RR,ACE_THR_PRI_FIFO_DEF,ACE_SCOPE_PROCESS);

      if(bRealtime)
        {
          if(ACE_OS::sched_params(params) == -1)
            {
              logger.log(EMANE::ERROR_LEVEL,
                         "could not set scheduler to realtime");
              return EXIT_FAILURE; 
            }
        }

      EMANE::EventGeneratorManager * pEventGeneratorManager = director.construct();
      
      EMANE::ErrorEventReceiver errorEventReceiver;
      
      pEventGeneratorManager->registerEventHandler(EMANE::EventErrorEvent::EVENT_ID,&errorEventReceiver);

      if(waitTime > ACE_Time_Value::zero)
        {      
          logger.log(EMANE::DEBUG_LEVEL,
                     "Delay start for %ld seconds", 
                     waitTime.sec());

          ACE_OS::sleep(waitTime);
        }

      pEventGeneratorManager->start();

      pEventGeneratorManager->postStart();

      // if start was successfull close standard file descriptors
      // this is delayed to allow debug messages to stderr on exception
      // processing
      if(level == 0 && bDaemonize)
        {
          logger.setLogLevel(level);

          ACE_OS::close(ACE_STDIN);
          ACE_OS::close(ACE_STDOUT);
          ACE_OS::close(ACE_STDERR);
          
          ACE_HANDLE fd = ACE_OS::open("/dev/null", O_RDWR, 0);
          
          if(fd != ACE_INVALID_HANDLE)
            {
              ACE_OS::dup2(fd, ACE_STDIN);
              ACE_OS::dup2(fd, ACE_STDOUT);
              ACE_OS::dup2(fd, ACE_STDERR);
              
              if(fd > ACE_STDERR)
                {
                  ACE_OS::close(fd);
                }
            }
        }

      EMANEUtils::Terminator terminator;

      terminator.reactor(ACE_Reactor::instance());

      ACE_Reactor::instance()->run_reactor_event_loop();

      pEventGeneratorManager->stop();
      
      pEventGeneratorManager->destroy();

      delete pEventGeneratorManager;

      logger.log(EMANE::DEBUG_LEVEL,"bye"); 
    }

  catch(const EMANE::ConfigureException & exp)
    {
      logger.log(EMANE::ABORT_LEVEL,"%s: %s (item:%s, value:%s)",
                 exp.type(),
                 exp.what(),
                 exp.item().getName().c_str(),
                 exp.item().getValue().c_str());
      return EXIT_FAILURE;
    }
  catch(const EMANE::EMANEException & exp)
    {
      logger.log(EMANE::ABORT_LEVEL,"%s: %s",exp.type(),exp.what());
      return EXIT_FAILURE;
    }
  catch(const std::exception & exp)
    {
      logger.log(EMANE::ABORT_LEVEL,"%s",exp.what());
      return EXIT_FAILURE;
    }
  
  return EXIT_SUCCESS;
}

void usage()
{
  std::cout<<"usage: emaneeventservice [OPTIONS]... CONFIG_URI"<<std::endl;
  std::cout<<std::endl;
  std::cout<<" CONFIG_URI                   URI of XML configuration file."<<std::endl;
  std::cout<<std::endl;
  std::cout<<"options:"<<std::endl;
  std::cout<<"  -d, --daemonize                Run EMANE in the background"<<std::endl;
  std::cout<<"  -h, --help                    Print this message and exit."<<std::endl;
  std::cout<<"  -v, --version                 Print version and exit."<<std::endl;
  std::cout<<"  -l, --loglevel LEVEL          Set initial log level. [0,4]"<<std::endl;
  std::cout<<"  -L, --logserver ADDR:PORT      Log server endpoint."<<std::endl;
  std::cout<<"  -f, --logfile FILE             Log to a file instead of stderr"<<std::endl;
  std::cout<<"  --syslog                       Log to syslog instead of stderr"<<std::endl;
  std::cout<<"  -r, --realtime                Set realtime scheduling"<<std::endl;
  std::cout<<"  -s, --starttime               Set the start time HH:MM:SS"<<std::endl;
  std::cout<<"  -n, --nextday                 Set the start time to the next day, after midnight"<<std::endl;
  std::cout<<std::endl;
}
