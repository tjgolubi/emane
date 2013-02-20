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

#include "mobilitymodelloader.h"
#include "emaneutils/spawnmemberfunc.h"
#include "emane/emaneconstants.h"
#include <cmath>

#define NEM_TO_INDEX(x) ((x) - 1)

namespace
{
  const double PI                    = 3.14159265;
  const double DEG_IN_RAD            = PI / 180.0;
  const double RAD_IN_DEG            = 180.0 / PI;
  const double SCALE                 = 0.9996;
  const double RADIUS_METERS         = 6378137.0;
  const double ECCENTRICITY_SQ       = 0.00669438;
  const double ECCENTRICITY_PRIME_SQ = ECCENTRICITY_SQ / (1.0 - ECCENTRICITY_SQ);
  const double E1                    = (1.0 - sqrt(1.0 - ECCENTRICITY_SQ)) / (1.0 + sqrt(1.0 - ECCENTRICITY_SQ));
}

MobilityModelLoader::MobilityModelLoader(EMANE::PlatformServiceProvider * pPlatformService):
  u16NumberNodes_(0),
  matrixBuffer_(60),
  locationBuffer_(60),
  u16RepeatCount_(0),
  u8ZoneUTM_(18),
  u8LetterUTM_('T'),
  u32ValidEntry_(0),
  u32InvalidEntry_(0),
  u32InvalidNEMNode_(0),
  u32NumProduced_(0),
  ppConnectivityMatrix_(0),
  pLocationList_(0),
  pPlatformService_(pPlatformService),
  thread_(0)
{}

MobilityModelLoader::~MobilityModelLoader()
{
  if(thread_)
    {
      ACE_OS::thr_cancel(thread_);
      ACE_OS::thr_join(thread_,0,0);
    }

  if(ppConnectivityMatrix_)
    {
      delete [] *ppConnectivityMatrix_;
      delete [] ppConnectivityMatrix_;
    }

  if(pLocationList_)
    {
      delete [] pLocationList_;
    }
}


void MobilityModelLoader::open(ACE_UINT16 u16NumberNodes,
                               const FileVector & fileVector, 
                               ACE_UINT8 u8ZoneUTM, 
                               ACE_UINT8 u8LetterUTM, 
                               EntryReplayMap entryReplayMap, 
                               ACE_UINT16 u16RepCnt)
{
  u16NumberNodes_ = u16NumberNodes;
  fileVector_ = fileVector;
  entryReplayMap_  = entryReplayMap;
  u16RepeatCount_ = (u16RepCnt == 0) ? ACE_UINT16_MAX : u16RepCnt;
  
  u8ZoneUTM_   = u8ZoneUTM;
  u8LetterUTM_ = u8LetterUTM;

  // create the connectivity matrix
  ppConnectivityMatrix_ = new PathlossEvent::PathlossEntry * [u16NumberNodes_];
  
  PathlossEvent::PathlossEntry *  pConnectivityMatrix = 
    new PathlossEvent::PathlossEntry [u16NumberNodes_ * u16NumberNodes_];
  
  for(int i = 0; i < u16NumberNodes_; ++i, pConnectivityMatrix += u16NumberNodes_)
    {
      *(ppConnectivityMatrix_ +i) = pConnectivityMatrix;
    }
  
  // create the location list
  pLocationList_ = new LocationEvent::LocationEntry[u16NumberNodes_];

  EMANEUtils::spawn(*this,&MobilityModelLoader::svc,&thread_);
}

MobilityModelLoader::ConnectivityMatrix MobilityModelLoader::getConnectivityMatrix()
{
  return matrixBuffer_.consume();
}

MobilityModelLoader::LocationList  MobilityModelLoader::getLocationList()
{
  return locationBuffer_.consume();
}

ACE_THR_FUNC_RETURN MobilityModelLoader::svc(void)
{
  ACE_UINT32 u32TxXUTM  = 0;
  ACE_UINT32 u32TxYUTM  = 0;
  ACE_INT32  i32TXAntennaHeightMeters = 0;
  ACE_UINT32 u32RxXUTM  = 0;
  ACE_UINT32 u32RxYUTM  = 0;
  ACE_INT32  i32RXAntennaHeightMeters = 0;
  ACE_UINT32 u32EntryTime;
  ACE_UINT16 u16RxNode;

  for (ACE_UINT16 u16Index = 0; u16Index < u16RepeatCount_; ++u16Index)
    {
      //resetting
      u32ValidEntry_     = 0;
      u32InvalidEntry_   = 0;
      u32InvalidNEMNode_ = 0;
      
      PathlossEvent::PathlossEntry e;
      memset(&e,0,sizeof(e));

      LocationEvent::LocationEntry l;
      memset(&l,0,sizeof(l));
       
      // clear the matrix 
      memset(&ppConnectivityMatrix_[0][0],
             0,
             sizeof(PathlossEvent::PathlossEntry) * u16NumberNodes_ * u16NumberNodes_);

      // clear the location list
      memset(pLocationList_,0,sizeof(LocationEvent::LocationEntry) * u16NumberNodes_);

      unsigned long ulTimeSeconds = 0;

      FileVector::const_iterator iter;

      for(iter = fileVector_.begin(); iter != fileVector_.end(); ++iter)
        {
          size_t szLineCount = 0;
          FILE * pFd = 0;

          if((pFd = fopen(iter->c_str(),"r")) != NULL)
            {
              while(!feof(pFd))
                {
                  char cbPathLossFormat[128] = {0};

                  float fForwardPathLoss, fReversePathLoss, fDistanceMeters;
                  
                  // clear entry
                  memset(&e,0,sizeof(e));
                  
                  u32EntryTime = 0;
                  u16RxNode = 0;

                  // keep track of lines
                  ++szLineCount;
                  
                  // scan each line
                  if(fscanf(pFd, "%u %hu %hu %s %f %u %u %d %u %u %d",
                            &u32EntryTime,
                            &e.u16TxNode_,
                            &u16RxNode,
                            cbPathLossFormat,
                            &fDistanceMeters,
                            &u32TxXUTM,
                            &u32TxYUTM,
                            &i32TXAntennaHeightMeters,
                            &u32RxXUTM,
                            &u32RxYUTM,
                            &i32RXAntennaHeightMeters) == 11)
                    {
                      // check for asymetric path loss
                      if(sscanf(cbPathLossFormat,"%f/%f", &fForwardPathLoss, &fReversePathLoss) == 2) 
                        {
                          // set forward pathloss
                          e.i32PathLossDBScaled_ =
                            static_cast<ACE_INT32>(fForwardPathLoss * PathlossEvent::PATHLOSSSCALE);
                          
                          // set reverse pathloss
                          e.i32RevPathLossDBScaled_ = 
                            static_cast<ACE_INT32>(fReversePathLoss *  PathlossEvent::PATHLOSSSCALE);
                        }
                      // check for symetric path loss
                      else if(sscanf(cbPathLossFormat,"%f", &fForwardPathLoss) == 1) 
                        {
                          // set forward pathloss
                          e.i32PathLossDBScaled_ = 
                            static_cast<ACE_INT32>(fForwardPathLoss * PathlossEvent::PATHLOSSSCALE);

                          // reverse path loss is the same as forward
                          e.i32RevPathLossDBScaled_ = 
                            static_cast<ACE_INT32>(fForwardPathLoss * PathlossEvent::PATHLOSSSCALE);
                        }
                      // bad path loss format
                      else
                        {
                          pPlatformService_->log(EMANE::ERROR_LEVEL,"bad path loss format [%s], file %s, line %zu", 
                                  cbPathLossFormat, iter->c_str(), szLineCount);
                          ++u32InvalidNEMNode_; 
                          continue;
                        }
                      
                      ++u32ValidEntry_;

                      // when the current time referenced matrix we are building is complete
                      // the current mobility entry will have a time Tn + 1.  This difference
                      // indicates we have a completed matrix.
                      if(ulTimeSeconds != u32EntryTime)
                        {
                          int iProduceCount = 1;

                          // check if this entry is to be replayed                      
                          EntryReplayMap::iterator entryReplayMapIter = entryReplayMap_.find(ulTimeSeconds);
  
                          if(entryReplayMapIter != entryReplayMap_.end())
                            {
                              iProduceCount = entryReplayMapIter->second;
                            }

                          for(int i = 0; i < iProduceCount; ++i)
                            {
                              ++u32NumProduced_;
                              matrixBuffer_.produce(ConnectivityMatrix(&ppConnectivityMatrix_[0][0],&ppConnectivityMatrix_[0][0] + u16NumberNodes_ * u16NumberNodes_));

                              locationBuffer_.produce(LocationList(pLocationList_,pLocationList_ + u16NumberNodes_));
                            }
                          
                          memset(&ppConnectivityMatrix_[0][0],
                                 0,
                                 sizeof(PathlossEvent::PathlossEntry) * u16NumberNodes_ * u16NumberNodes_);
                          
                          memset(pLocationList_,0,sizeof(LocationEvent::LocationEntry) * u16NumberNodes_);

                          
                          ulTimeSeconds = u32EntryTime;
                        }
                      
                      // the mobility model is condensed.  all data is two way A -> B and B <- A so if we have read in
                      // A -> B for time Tn we also know B->A for Tn.  The only thing we care about is storing row
                      // information only if both A or B are locally attached.
                      if(NEM_TO_INDEX(e.u16TxNode_) < u16NumberNodes_ && NEM_TO_INDEX(u16RxNode)< u16NumberNodes_)
                        {
                          memcpy(&ppConnectivityMatrix_[NEM_TO_INDEX(u16RxNode)][NEM_TO_INDEX(e.u16TxNode_)],
                                 &e,
                                 sizeof(PathlossEvent::PathlossEntry));

                          /* Lat/Lon */
                          ACE_INT32 i32LatScaled = 0;
                          ACE_INT32 i32LonScaled = 0;

                          /* Sender */
                          pLocationList_[NEM_TO_INDEX(e.u16TxNode_)].u16Node_ = e.u16TxNode_;
                          
                          utmToArcSec(u32TxXUTM, u32TxYUTM, i32LatScaled, i32LonScaled);

                          pLocationList_[NEM_TO_INDEX(e.u16TxNode_)].i32LatitudeMARCS_  = i32LatScaled;
                          pLocationList_[NEM_TO_INDEX(e.u16TxNode_)].i32LongitudeMARCS_ = i32LonScaled;
                          pLocationList_[NEM_TO_INDEX(e.u16TxNode_)].i32AltitudeMeters_ = i32TXAntennaHeightMeters;
                          
                          pLocationList_[NEM_TO_INDEX(u16RxNode)].u16Node_ = u16RxNode;

                          utmToArcSec(u32RxXUTM, u32RxYUTM, i32LatScaled, i32LonScaled);

                          pLocationList_[NEM_TO_INDEX(u16RxNode)].i32LatitudeMARCS_  = i32LatScaled;
                          pLocationList_[NEM_TO_INDEX(u16RxNode)].i32LongitudeMARCS_ = i32LonScaled;
                          pLocationList_[NEM_TO_INDEX(u16RxNode)].i32AltitudeMeters_ = i32RXAntennaHeightMeters;

                          // swap src/dst
                          ACE_UINT16 u16Tmp = e.u16TxNode_;
                          e.u16TxNode_ = u16RxNode;
                          u16RxNode = u16Tmp;
                          
                          // swap pathloss
                          ACE_INT32 i32Tmp          = e.i32PathLossDBScaled_;
                          e.i32PathLossDBScaled_    = e.i32RevPathLossDBScaled_;
                          e.i32RevPathLossDBScaled_ = i32Tmp;
                          
                          memcpy(&ppConnectivityMatrix_[NEM_TO_INDEX(u16RxNode)][NEM_TO_INDEX(e.u16TxNode_)],
                                 &e,
                                 sizeof(PathlossEvent::PathlossEntry));
                        }
                      else
                        {
                          ++u32InvalidNEMNode_; 
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
                      
                      ++u32InvalidEntry_;
                    }
                }
              
              fclose(pFd);
            }
      
          // provided we have read in at least 1 valid entry there is one matrix left
          // to publish  after file processing is complete
          if(u32ValidEntry_)
            {
              int iProduceCount = 1;

              // check if this entry is to be replayed                      
              EntryReplayMap::iterator entryReplayMapIter = entryReplayMap_.find(ulTimeSeconds);
  
              if(entryReplayMapIter != entryReplayMap_.end())
                {
                  iProduceCount = entryReplayMapIter->second;
                }

              for(int i = 0; i < iProduceCount; ++i)
                {
                  ++u32NumProduced_;
                  matrixBuffer_.produce(ConnectivityMatrix(&ppConnectivityMatrix_[0][0],&ppConnectivityMatrix_[0][0] + u16NumberNodes_ * u16NumberNodes_));

                  locationBuffer_.produce(LocationList(pLocationList_,pLocationList_ + u16NumberNodes_));
                }
            }
        }
      
      // signify we are processing data.  produce and empty matrix.
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"Iteration            : %u",u16Index+1);
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"Valid Entry          : %u",u32ValidEntry_);
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"Entries Produced     : %u",u32NumProduced_);
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"Invalid Entry        : %u",u32InvalidEntry_);
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"Invalid NEM Node     : %u",u32InvalidNEMNode_);

    }//end for (repeatcount)
  
  matrixBuffer_.produce(ConnectivityMatrix());
  locationBuffer_.produce(LocationList());
  
  return 0;
}

void MobilityModelLoader::utmToArcSec(ACE_UINT32 utmEasting, ACE_UINT32 utmNorthing, ACE_INT32 &i32Latitude, ACE_INT32 &i32Longitude)
{
  /* Center meridian is at 500000, get true easting */
  int iX = utmEasting - 500000;
  
  int iY = utmNorthing;
  
  /* Check which hemisphere we're in - 'N' is first zone in Northern */
  if ((u8LetterUTM_ - 'N') < 0)
    {
      /* In southern hemisphere, need to subract northing from 10000km */
      iY -= 10000000;
    }
  
  /* Get center meridian of zone */
  int iCenterMeridian = int(u8ZoneUTM_ - 1) * 6 - 180 + 3;
  
  /* Get Meridional Arc */
  double dMerArc = iY / SCALE;
  
  /* Get distance from point to the polar axis */
  double dMu = dMerArc / (RADIUS_METERS * (1 - ECCENTRICITY_SQ / 4 - 3 * 
                                           ECCENTRICITY_SQ * ECCENTRICITY_SQ / 64 -
                                           5 * ECCENTRICITY_SQ * ECCENTRICITY_SQ *
                                           ECCENTRICITY_SQ / 256));
  
  /* Calculate Footprint Latitude */
  double dPhiRadians = dMu + (3 * E1 / 2 - 27 * E1 * E1 * E1 / 32) * sin(2 * dMu) +
    (21 * E1 * E1 / 16 - 55 * E1 * E1 * E1 * E1 / 32) * sin(4 * dMu) +
    (151 * E1 * E1 * E1 / 96) * sin(6 * dMu);// +
  //        (1097 * E1 * E1 * E1 * E1 / 512) * sin(8 * dMu);
  
  
  /* Intermediate variables for calculating Lat and Lon */
  double dC1 = ECCENTRICITY_PRIME_SQ * cos(dPhiRadians) * cos(dPhiRadians);
  double dT1 = tan(dPhiRadians) * tan(dPhiRadians);
  double dR1 = RADIUS_METERS * (1 - ECCENTRICITY_SQ) / 
    pow(1 - ECCENTRICITY_SQ * sin(dPhiRadians) * sin(dPhiRadians), 1.5);
  double dN1 = RADIUS_METERS / sqrt(1 - ECCENTRICITY_SQ * sin(dPhiRadians) * sin(dPhiRadians));
  double dD = iX / (dN1 * SCALE);
  
  /* Latitude */
  double dLatitude = dPhiRadians - (dN1 * tan(dPhiRadians) / dR1) * 
    ((dD * dD / 2) - ((5 + 3 * dT1 + 10 * dC1 - 4 * dC1 * dC1 - 9 * ECCENTRICITY_PRIME_SQ) *
                      dD * dD * dD * dD / 24) + 
     ((61 + 90 * dT1 + 298 * dC1 + 45 * dT1 * dT1 - 252 * ECCENTRICITY_PRIME_SQ - 3 * dC1 * dC1) * 
      dD * dD * dD * dD * dD * dD / 720));
  
  dLatitude = dLatitude * RAD_IN_DEG;
  
  /* Longitude */
  double dLongitude = (dD - (1 + 2 * dT1 * dC1) * dD * dD * dD / 6 + 
                 (5 - 2 * dC1 + 28 * dT1 - 3 * dC1 * dC1 + 8 * 
                  ECCENTRICITY_PRIME_SQ + 24 * dT1 * dT1) * dD * dD * dD * dD * dD / 120) / cos(dPhiRadians) ;
  
  dLongitude = iCenterMeridian + dLongitude * RAD_IN_DEG;

  /* Set return values */
  i32Latitude  = static_cast<ACE_INT32>(dLatitude  * EMANE::MILLI_ARC_SECONDS_PER_DEGREE);
  i32Longitude = static_cast<ACE_INT32>(dLongitude * EMANE::MILLI_ARC_SECONDS_PER_DEGREE);


  pPlatformService_->log(EMANE::DEBUG_LEVEL,"utm easting %u, utm northing %u, lat %8.6lf deg, lon %8.6lf deg, lat %d marcs, lon %d marcs",
                         utmEasting, 
                         utmNorthing, 
                         dLatitude, 
                         dLongitude, 
                         i32Latitude, 
                         i32Longitude);
}

