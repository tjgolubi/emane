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

#ifndef MOBILITYMODELLOADER_HEADER_
#define MOBILITYMODELLOADER_HEADER_

#include "emaneevents/pathlossevent.h"
#include "emaneevents/locationevent.h"

#include "emaneutils/producerconsumerbuffer.h"
#include "emane/emaneplatformserviceprovider.h"

#include <ace/Thread.h>

#include <vector>
#include <map>

/**
 * @class MobilityModelLoader
 *
 * @brief Loads the mobility model
 */
class MobilityModelLoader
{
public:
  typedef std::vector<PathlossEvent::PathlossEntry> ConnectivityMatrix;
  typedef std::vector<LocationEvent::LocationEntry> LocationList;
  
  typedef std::vector<std::string> FileVector;
  typedef std::map<ACE_UINT32,ACE_UINT16> EntryReplayMap;
  typedef EntryReplayMap::iterator EntryReplayMapIter;

  MobilityModelLoader(EMANE::PlatformServiceProvider * pPlatformService);

  ~MobilityModelLoader();

  void open(ACE_UINT16 u16NUmberNodes,const FileVector & fileVector, ACE_UINT8 u8ZoneUTM, 
            ACE_UINT8 u8LetterUTM, EntryReplayMap entryReplayMap, ACE_UINT16 u16RepCnt = 1);

  /**
   * Get the current connectivit matrix. This call may block if there
   * is no matrix available. An empty EntryVector indicates there is 
   * no more data available.
   * 
   * @return Flat vector representing the connectivity matrix.
   * @verbatim
   * The below matrix:
   *
   *         0  1  2  3  4  5  6  7 
   *      -------------------------
   *      A| A0 A1 A2 A3 A4 A5 A6 A7
   *      B| B0 B1 B2 A3 B4 B5 B6 B7
   *      C| C0 C1 C2 A3 C4 C5 C6 C7
   *      D| D0 D1 D2 A3 D4 D5 D6 D7
   *
   * would be returned as:
   *
   * A0 A1 A2 A3 A4 A5 A6 A7 B0 B1 B2 B3 B4...D6 D7
   * @endverbatim
   */  
  ConnectivityMatrix getConnectivityMatrix();

  LocationList getLocationList();

private:
  ACE_UINT16 u16NumberNodes_;
  FileVector fileVector_;
  EntryReplayMap  entryReplayMap_;

  EMANEUtils::ProducerConsumerBuffer<ConnectivityMatrix> matrixBuffer_;
  EMANEUtils::ProducerConsumerBuffer<LocationList> locationBuffer_;
  
  ACE_UINT16 u16RepeatCount_;

  ACE_UINT8  u8ZoneUTM_;
  ACE_UINT8  u8LetterUTM_;

  ACE_UINT32 u32ValidEntry_;
  ACE_UINT32 u32InvalidEntry_;
  ACE_UINT32 u32InvalidNEMNode_;
  ACE_UINT32 u32NumProduced_;
  
  PathlossEvent::PathlossEntry ** ppConnectivityMatrix_; 
  LocationEvent::LocationEntry *  pLocationList_;

  EMANE::PlatformServiceProvider * pPlatformService_;

  ACE_thread_t thread_;

  ACE_THR_FUNC_RETURN svc();

  void utmToArcSec(ACE_UINT32, ACE_UINT32, ACE_INT32 &, ACE_INT32 &);
};

#endif // MOBILITYMODELLOADER_HEADER_
