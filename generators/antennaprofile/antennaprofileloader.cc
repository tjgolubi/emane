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

#include "antennaprofileloader.h"
#include "emaneutils/spawnmemberfunc.h"

#define NEM_TO_INDEX(x) ((x) - 1)

AntennaProfileLoader::AntennaProfileLoader(EMANE::PlatformServiceProvider * pPlatformService) :
  thread_(0),
  u16NumberNodes_(0),
  u32ValidEntry_(0),
  u16RepeatCount_(1),
  pAntennaList_(0),
  AntennaProfileBuffer_(60),
  pPlatformService_(pPlatformService)
 { }

AntennaProfileLoader::~AntennaProfileLoader()
 { }

void AntennaProfileLoader::open(ACE_UINT16 u16NumberNodes,const FileVector & fileVector, ACE_UINT16 u16RepeatCount)
{
  fileVector_     = fileVector;
  u16NumberNodes_ = u16NumberNodes;
  u16RepeatCount_ = u16RepeatCount;

  pAntennaList_ = new AntennaProfileEvent::AntennaProfileEntry[u16NumberNodes];

  EMANEUtils::spawn(*this,&AntennaProfileLoader::svc,&thread_);
}

AntennaProfileLoader::AntennaProfileList AntennaProfileLoader::getAntennaProfileList()
{
  return AntennaProfileBuffer_.consume();
}

ACE_THR_FUNC_RETURN AntennaProfileLoader::svc()
{
  ACE_UINT32 u32EntryTime = 0;
  ACE_UINT32 u32CurrentTime = 1;
  ACE_UINT16 u16NodeId = 0;
  ACE_UINT32 u32Count = 1;
  u32ValidEntry_ = 0;


  if(u16RepeatCount_ > 0)
  {
    u32Count = u16RepeatCount_;
  }

  while(u32Count)
  {
    memset(pAntennaList_, 0, sizeof(AntennaProfileEvent::AntennaProfileEntry) * u16NumberNodes_);

    FileVector::const_iterator iter;

    for(iter = fileVector_.begin(); iter != fileVector_.end(); ++iter)
    {
      size_t szLineCount = 0;
      FILE *pFd = 0;

      if((pFd = fopen(iter->c_str(),"r")) != NULL)
        {
          while(!feof(pFd))
            {
              float    fElevationDegrees = 0;
              float    fAzimuthDegrees   = 0;
              ACE_UINT16 u16ProfileId    = 0;

              // Keep the line count for error reporting
              ++szLineCount;

              // Scan each line (time NEM profile az el)
              if(fscanf(pFd, "%u %hu %hu %f %f",
                        &u32EntryTime,
                        &u16NodeId,
                        &u16ProfileId,
                        &fAzimuthDegrees,
                        &fElevationDegrees) == 5)
                {
                  
                  // Check the parameters
                  pAntennaList_[NEM_TO_INDEX(u16NodeId)].u16Node_ = u16NodeId;

                  if((-90.0 <= fElevationDegrees) && (fElevationDegrees <= 90.0)  &&  // el [-90.0 to 90.0]
                       (0.0 <= fAzimuthDegrees)   && (fAzimuthDegrees   < 360.0)  &&   // az [  0.0 to 360.0)
                       (u16ProfileId > 0))                                             // profile > 0
                    {
                      // If the time has shifted then send out the current batch
                      if((u32CurrentTime != u32EntryTime) && u32ValidEntry_ > 0)
                       {
                         AntennaProfileBuffer_.produce(AntennaProfileList(pAntennaList_, pAntennaList_ + u16NumberNodes_));

                         memset(pAntennaList_, 0, sizeof(AntennaProfileEvent::AntennaProfileEntry) * u16NumberNodes_);

                         u32CurrentTime = u32EntryTime;
                       }

                      pAntennaList_[NEM_TO_INDEX(u16NodeId)].u16Node_           = u16NodeId;
                      pAntennaList_[NEM_TO_INDEX(u16NodeId)].u16ProfileId_      = u16ProfileId;
                      pAntennaList_[NEM_TO_INDEX(u16NodeId)].fElevationDegrees_ = fElevationDegrees;
                      pAntennaList_[NEM_TO_INDEX(u16NodeId)].fAzimuthDegrees_   = fAzimuthDegrees;
                      u32ValidEntry_++; 
                    }
                  else
                    {
                      pPlatformService_->log(EMANE::ERROR_LEVEL, "Entry out of range near line %zu", szLineCount); 
                    }

                }
              else
                {
                  if(feof(pFd))
                   {
                      break;
                   }
                   pPlatformService_->log(EMANE::ERROR_LEVEL, "bad entry file %s, near line %zu", iter->c_str(), szLineCount);
                   break;

                }
            }
          fclose(pFd);
        }
        else
         {
           pPlatformService_->log(EMANE::DEBUG_LEVEL, "Cannot Open File"); 
         }
        if(u32ValidEntry_)
          {
            pPlatformService_->log(EMANE::DEBUG_LEVEL, "Valid Entry to Produce");
            AntennaProfileBuffer_.produce(AntennaProfileList(pAntennaList_,pAntennaList_ + u16NumberNodes_));
          }
    }
    if(u16RepeatCount_ > 0)
    {
      u32Count--;
    }

 
  } // End Repeat Count
  AntennaProfileBuffer_.produce(AntennaProfileList());
  return 0;
}

