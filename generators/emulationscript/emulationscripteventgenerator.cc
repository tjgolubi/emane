/*
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "emulationscripteventgenerator.h"

#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/parameterconvert.h"

#include <ace/OS_NS_sys_time.h>

#include <sstream>

namespace
{
  /* ConfigurationDefinition to be filled-in here */
  EMANE::ConfigurationDefinition defs[] = 
    {
      {true,false,1,"schemalocation", 0,0, "Path to schema to validate input files"},
      {true,false,0,"inputfile",      0,0, "Name of a file used as input"},
      {false,true,1,"repeatcount",   "1",0,"Number of repetitions of the entire model"},
      {0, 0, 0, 0, 0, 0, 0},
    };
}

EmulationScriptEventGenerator::EmulationScriptEventGenerator(EMANE::PlatformServiceProvider *pPlatformService)
  : EventGenerator("Emulation Script Event Generator [MITRE]",pPlatformService),
    thread_(0),
    cond_(mutex_),
    bCancel_(false)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);

  //addEventId(PathlossEvent::EVENT_ID); <-- not handled now @TODO!
  addEventId(LocationEvent::EVENT_ID);
}

EmulationScriptEventGenerator::~EmulationScriptEventGenerator()
{
  if(thread_)
    {
      mutex_.acquire();
      bCancel_ = true;
      loader_.cancel();
      cond_.signal();
      mutex_.release();
      ACE_OS::thr_join(thread_,0,0);
    }
}

void EmulationScriptEventGenerator::initialize()
  throw(EMANE::InitializeException)
{
  /* Empty */
}

void EmulationScriptEventGenerator::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  Component::configure(items);
}

void EmulationScriptEventGenerator::start()
  throw(EMANE::StartException)
{
  EmulationScriptLoader::FileVectorType fileVector;
  std::string sSchemaFile = "";
  ACE_UINT16  u16RepCnt = 1;

  /* Go through available configuration items */
  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();

  try
    {
      for (; iter != configRequirements_.end(); ++iter)
        {
          if (iter->second.bPresent_)
            {
              if (iter->first == "schemalocation")
                {
                  sSchemaFile = iter->second.item_.getValue();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"EmulationScriptEventGenerator:start Schema Location: %s",sSchemaFile.c_str());
                }
              else if (iter->first == "inputfile")
                {
                  fileVector.push_back(iter->second.item_.getValue());
                }
              else if (iter->first == "repeatcount")
                {
                  u16RepCnt = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"EmulationScriptEventGenerator:start Repeat Count: %d",u16RepCnt);
                }
              
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"EmulationScriptEventGenerator: Missing configuration item "<<iter->first<<std::ends;
              throw EMANE::StartException(ssDescription.str());
            }
        }
    }
   catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"EmulationScriptEventGenerator: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }

  /* Pass necessary values to the loader */
  loader_.open(fileVector, sSchemaFile, u16RepCnt);

  /* Start the Event thread */
  EMANEUtils::spawn(*this, &EmulationScriptEventGenerator::svc, &thread_);
}

void EmulationScriptEventGenerator::stop()
  throw(EMANE::StopException)
{
  /* Empty */
}

void EmulationScriptEventGenerator::destroy()
  throw()
{
  /* Empty */
}


void EmulationScriptEventGenerator::processTimedEvent(ACE_UINT32, long , const ACE_Time_Value &, const void *)
{

}

void EmulationScriptEventGenerator::processEvent(const EMANE::EventId&, const EMANE::EventObjectState&)
{

}


ACE_THR_FUNC_RETURN EmulationScriptEventGenerator::svc()
{
  /* Time to sleep, default is 1, but will be overwritten by getNextLocations */
  ACE_Time_Value tvWait = ACE_OS::gettimeofday();
  int iRet = 0;

  /* Loop through and get available events */
  while (1)
    {
      /* Get Available location list from buffer */
      ACE_Time_Value tvDelta;
      EmulationScriptLoader::LocationListType locations = loader_.getNextLocations(tvDelta);

      /* If we've got locations, send them out */
      if (!locations.empty())
        {
          mutex_.acquire();
          iRet = 0;  
          tvWait += tvDelta;
          /* Wait for the time between events */
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

          /* Generate event */
          LocationEvent locationEvent(&locations[0], locations.size());

          pPlatformService_->sendEvent(0,                       // all platforms
                                       0,                       // all nems
                                       EMANE::COMPONENT_ALL,    // all components
                                       locationEvent);

        }// end if locations and server
      else if (locations.empty() && loader_.errorOccurred())
        {
          pPlatformService_->log(EMANE::ERROR_LEVEL,"EmulationScriptEventGenerator: Loader Error Occurred!");

          break;
        }
      else if (locations.empty())
        {
          pPlatformService_->log(EMANE::DEBUG_LEVEL,"EmulationScriptEventGenerator: Loader Completed.");

          break;
        }
      else
        {
          pPlatformService_->log(EMANE::DEBUG_LEVEL,"EmulationScriptEventGenerator: Event Server NOT set!");

          break;
        }

    }

  return 0;
}

DECLARE_EVENT_GENERATOR(EmulationScriptEventGenerator);
