/*
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "mobilitymodeleventgenerator.h"

#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/parameterconvert.h"

#include <ace/OS_NS_string.h>
#include <ace/OS_NS_sys_time.h>

#include <sstream>

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      {true,false,1,"inputfileformat",   0,0,"Mobility Model input file format"},
      {true,false,1,"inputfilecount",    0,0,"Mobility Model input file count"},
      {true,false,1,"totalnodes",        0,0,"Total number of nodes in mobility model"},
      {true,false,1,"maxnemidpresent",   0,0,"MAX NEM Id Present"},
      {false,true,1,"repeatcount",      "1",0,"Number of repetitions of the entire model"},
      {true,false,1,"utmzone",           0,0,"UTM Zone designation for coordinates in model"},
      {false,true,1,"entryreplay",       0,0,"Number of repetitions of an entry"},
      {false,true,1,"publishlocationevents","on",0,"Publish location events"},
      {false,true,1,"publishpathlossevents","on",0,"Publish pathloss events"},
      {0,0,0,0,0,0,0},
    };
}

MobilityModelEventGenerator::MobilityModelEventGenerator(EMANE::PlatformServiceProvider *pPlatformService):
  EventGenerator("Mobility Model Event Generator", pPlatformService),
  loader_(pPlatformService_),
  u16TotalNodes_(0),
  u16MaxNEMPresent_(0),
  bPublishLocationEvents_(true),
  bPublishPathlossEvents_(true),
  cond_(mutex_),
  bCancel_(false)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
  addEventId(PathlossEvent::EVENT_ID);
  addEventId(LocationEvent::EVENT_ID);
}

MobilityModelEventGenerator::~MobilityModelEventGenerator()
{
}

 void MobilityModelEventGenerator::initialize()
   throw(EMANE::InitializeException)
{
}

void MobilityModelEventGenerator::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  Component::configure(items);
}

void MobilityModelEventGenerator::start()
  throw(EMANE::StartException)
{
  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();

  MobilityModelLoader::FileVector fileVector;
  MobilityModelLoader::EntryReplayMap entryReplayMap;
  int iInputFileCount = 0;
  std::string sInputFileFormat;
  std::string sReplayEntryFormat = "";
  ACE_UINT16 u16RepCnt = 1;

  /* Default to DirtyJersey */
  ACE_UINT8    u8ZoneUTM   = 18;  
  const char * pzLetterUTM = "T";

  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "inputfileformat")
                {
                  sInputFileFormat = iter->second.item_.getValue();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start Input File Format: %s",sInputFileFormat.c_str());
                }
              else if(iter->first == "inputfilecount")
                {
                  iInputFileCount = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toINT32();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start Input File Count: %d",iInputFileCount);
                }
              else if(iter->first == "totalnodes")
                {
                  u16TotalNodes_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start Total Nodes: %d",u16TotalNodes_);
                }
              else if(iter->first == "maxnemidpresent")
                {
                  u16MaxNEMPresent_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start Max NEM ID: %d",u16MaxNEMPresent_);
                }
              else if(iter->first == "repeatcount")
                {
                  u16RepCnt = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start Repeat Count: %d",u16RepCnt);
                }
              else if(iter->first == "utmzone")
                {
                  u8ZoneUTM = static_cast<ACE_UINT8>(ACE_OS::strtoul(iter->second.item_.getValue().c_str(), 
                                                                     const_cast<char**>(&pzLetterUTM), 10));
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start UTM Zone: %d",u8ZoneUTM);
                }
              else if(iter->first == "entryreplay")
                {
                  sReplayEntryFormat = iter->second.item_.getValue();
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start Entry Replay: %s",sReplayEntryFormat.c_str());
                }
              else if(iter->first == "publishlocationevents")
                {
                  bPublishLocationEvents_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start publishlocationevents: %d", bPublishLocationEvents_);
                }
              else if(iter->first == "publishpathlossevents")
                {
                  bPublishPathlossEvents_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator:start publishpathlossevents: %d", bPublishPathlossEvents_);
                }

              
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"MobilityModelEventGenerator: Missing configuration item "<<iter->first<<std::ends;
              throw EMANE::StartException(ssDescription.str());
              
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"MobilityModelEventGenerator: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }
  
  char buf[MAXPATHLEN];

  for(int i = 0; i < iInputFileCount; ++i)
    {
      snprintf(buf,sizeof(buf),sInputFileFormat.c_str(),i);
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator loading file %s",buf);
      fileVector.push_back(buf);
    }

  char *pzToken;
  int iTokenCount = 0;

  while ((pzToken = ACE_OS::strtok (iTokenCount == 0 ? &sReplayEntryFormat[0] : NULL, " \t\n")) != NULL) 
    {
      ++iTokenCount;
      int iIndex, iReplayCount;
      if(sscanf(pzToken,"%d:%d", &iIndex, &iReplayCount) == 2)
        {
          pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator replay entry %d, count %d", iIndex, iReplayCount);
          entryReplayMap.insert(std::make_pair(iIndex,iReplayCount));
        }
      else
        {
          pPlatformService_->log(EMANE::ERROR_LEVEL,"MobilityModelEventGenerator error loading replay entry %s",pzToken);
        }
    }

  loader_.open(u16TotalNodes_,fileVector, u8ZoneUTM, *pzLetterUTM, entryReplayMap, u16RepCnt);

  EMANEUtils::spawn(*this,&MobilityModelEventGenerator::svc,&thread_);
}

void MobilityModelEventGenerator::stop()
  throw(EMANE::StopException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator: stop");

  if(thread_)
    {
      mutex_.acquire();
      bCancel_ = true;
      cond_.signal();
      mutex_.release();
      ACE_OS::thr_join(thread_,0,0);
    }
}

void MobilityModelEventGenerator::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator: destroy");
}

void MobilityModelEventGenerator::processTimedEvent(ACE_UINT32, long , const ACE_Time_Value &, const void *)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MobilityModelEventGenerator: processTimedEvent");
}


void MobilityModelEventGenerator::processEvent(const EMANE::EventId&, const EMANE::EventObjectState&)
{

}

ACE_THR_FUNC_RETURN MobilityModelEventGenerator::svc()
{
  ACE_Time_Value tvWait;
  const ACE_Time_Value tvDelta(1,0);
  int iRet = 0;
  PathlossEvent::PathlossEntry entries[u16TotalNodes_ - 1];

  while(1)
    {
      mutex_.acquire();

      iRet = 0;

      tvWait = ACE_OS::gettimeofday() + tvDelta;

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
      
      MobilityModelLoader::ConnectivityMatrix matrix = loader_.getConnectivityMatrix();

      MobilityModelLoader::LocationList locations = loader_.getLocationList();

      if(!matrix.empty())
        {
          if(bPublishPathlossEvents_)
            {
              for(int i = 0; i < u16TotalNodes_ ; ++i)
                {
                  // build a pathloss event containing all entries _except_ the self entry
                  // copy from beginning of matrix slice to self entry
                  memcpy(&entries[0],&matrix[u16TotalNodes_*i],i * sizeof(PathlossEvent::PathlossEntry));
                  
                  // copy from position after self entry to end og matrix slice
                  memcpy(&entries[i],&matrix[u16TotalNodes_*i + i + 1],(u16TotalNodes_ - 1 - i ) * sizeof(PathlossEvent::PathlossEntry));
                  
                  PathlossEvent pathlossEvent(entries,u16TotalNodes_ - 1);
                  
                  if(i < u16MaxNEMPresent_)
                    {
                      pPlatformService_->sendEvent(0,                          // all platform(s)
                                                   i+1,                        // nem id
                                                   EMANE::COMPONENT_PHYILAYER, 
                                                   pathlossEvent);
                    }
                }
            }
        }
      else
        {
          break;
        }
      
      if(bPublishLocationEvents_)
        {
          if(!locations.empty())
            {
              LocationEvent locationEvent(&locations[0],u16TotalNodes_);

              pPlatformService_->sendEvent(0,                            // platform(s)
                                           0,                            // all nem(s)
                                           EMANE::COMPONENT_ALL,
                                           locationEvent);
            }
        }
    }


  return 0;
}

DECLARE_EVENT_GENERATOR(MobilityModelEventGenerator);
