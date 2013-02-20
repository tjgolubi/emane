/*
 * Copyright (c) 2012 - DRS CenGen LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen LLC nor the names of its
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

#include "antennaprofilegenerator.h"
#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/parameterconvert.h"

#include <ace/OS_NS_sys_time.h>

#include <sstream>

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      {true,false,1,"inputfileformat", 0,0,"Antenna Profile input file format"},
      {true,false,1,"inputfilecount",  0,0,"Antenna Profile input file count"},
      {true,false,1,"totalnodes",      0,0,"Total number of nodes in model"},
      {false,true,1,"repeatcount",   "1",0,"Number of repetitions of the entire model"},
      {0,0,0,0,0,0,0},
    };

}

AntennaProfileEventGenerator::AntennaProfileEventGenerator(EMANE::PlatformServiceProvider *pPlatformService):
  EventGenerator("Antenna Profile Event Generator", pPlatformService),
  loader_(pPlatformService_),
  cond_(mutex_),
  bCancel_(false)
  
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
  addEventId(AntennaProfileEvent::EVENT_ID);
}

AntennaProfileEventGenerator::~AntennaProfileEventGenerator()
{
}

void AntennaProfileEventGenerator::initialize()
  throw(EMANE::InitializeException)
{
}

void AntennaProfileEventGenerator::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  Component::configure(items);
}

void AntennaProfileEventGenerator::start()
  throw(EMANE::StartException)
{
  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();

  AntennaProfileLoader::FileVector fileVector;
  std::string sInputFileFormat;
  int iInputFileCount = 0;
  ACE_UINT16 u16RepeatCount = 1;

  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "inputfileformat")
                {
                  sInputFileFormat = iter->second.item_.getValue();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator:start Input File Format: %s",sInputFileFormat.c_str());
                }
              else if(iter->first == "inputfilecount")
                {
                  iInputFileCount = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINT32();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator:start Input File Count: %d",iInputFileCount);
                }
              else if(iter->first == "totalnodes")
                {
                  u16TotalNodes_ =
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINT16();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator:start Total Nodes: %d",u16TotalNodes_);
                }
              else if(iter->first == "repeatcount")
                {
                  u16RepeatCount =
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINT16();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator:start Repeat Count: %d",u16RepeatCount);
                }
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"AntennaProfileEventGenerator: Missing configuartion item "<<iter->first<<std::ends;
              throw EMANE::StartException(ssDescription.str());
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"AntennaProfileEventGenerator: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }

  char buf[MAXPATHLEN];

  for(int i = 0; i < iInputFileCount; ++i)
    {
      snprintf(buf,sizeof(buf),sInputFileFormat.c_str(),i);
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileGenerator loading file %s",buf);
      fileVector.push_back(buf);
    }

  loader_.open(u16TotalNodes_,fileVector,u16RepeatCount);

  EMANEUtils::spawn(*this,&AntennaProfileEventGenerator::svc,&thread_);
}

void AntennaProfileEventGenerator::stop()
  throw (EMANE::StopException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator: stop");

  if(thread_)
    {
      mutex_.acquire();
      bCancel_ = true;
      cond_.signal();
      mutex_.release();
      ACE_OS::thr_join(thread_,0,0);
    }
}

void AntennaProfileEventGenerator::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator: destroy");
}

void AntennaProfileEventGenerator::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void*)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator: processTimedEvent");
}

void AntennaProfileEventGenerator::processEvent(const EMANE::EventId&, const EMANE::EventObjectState&)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator: processEvent");
}

ACE_THR_FUNC_RETURN AntennaProfileEventGenerator::svc()
{
  ACE_Time_Value tvWait = ACE_OS::gettimeofday();
  const ACE_Time_Value tvDelta(1,0);
  int iRet = 0;

  while(1)
    {


      mutex_.acquire();
      iRet = 0;

      tvWait += tvDelta;

      while(!bCancel_ && iRet != -1)
      {
         iRet = cond_.wait(&tvWait);
      }

      if(bCancel_)
      {
         mutex_.release();
         break;
      } 
      mutex_.release();

      AntennaProfileLoader::AntennaProfileList list = loader_.getAntennaProfileList();

      if(!list.empty())
        {
          AntennaProfileEvent AntennaEvent(&list[0],u16TotalNodes_);
          pPlatformService_->sendEvent(0,
                                       0,
                                       EMANE::COMPONENT_ALL,
                                       AntennaEvent);
        }
      else
        {
          pPlatformService_->log(EMANE::DEBUG_LEVEL,"AntennaProfileEventGenerator: List is empty.  Finished");
          break;
        }

    }

  return 0;
}

DECLARE_EVENT_GENERATOR(AntennaProfileEventGenerator);
