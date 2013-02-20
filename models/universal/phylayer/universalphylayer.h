/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef UNIVERSALPHY_UNIVERSALPHYLAYER_HEADER_
#define UNIVERSALPHY_UNIVERSALPHYLAYER_HEADER_

#include "emane/emanephylayerimpl.h"
#include "emane/emanephytypes.h"
#include "emane/emanenemlist.h"
#include "emane/emanecommonphyheader.h"
#include "emane/emanecommonphyheaderexception.h"
#include "emane/emanestatisticunsignedinteger32.h"

#include "emane/emanecommonphyheaderantennaoption.h"
#include "emane/emanecommonphyheadertransmitteroption.h"

#include "emanecontrolmessages/universalphycontrolsendmessage.h"
#include "emanecontrolmessages/universalphycontrolrecvmessage.h"


#include "universalphyheader.h"

#include "pathlossmanager.h"
#include "noisemanager.h"
#include "antennaprofilemanager.h"

#include <set>

#include <ace/Basic_Types.h>

namespace UniversalPHY {

/**
 * @class UniversalPHYLayer
 *
 * @brief Implementation of the universal PHY layer
 */
class UniversalPHYLayer : public EMANE::PHYLayerImplementor
 {
 public:
  /**
   *
   * @brief constructor
   *
   */
  UniversalPHYLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService);

  /**
   *
   * @brief destructor
   *
   */
  ~UniversalPHYLayer();

  /**
   *
   * @brief initialize
   *
   */
  void initialize()
    throw(EMANE::InitializeException);

  /**
   *
   * @brief configure
   *
   * @param rItems config items
   *
   */
  void configure(const EMANE::ConfigurationItems & rItems)
    throw(EMANE::ConfigureException);
 
  /**
   *
   * @brief start
   *
   */
  void start()
    throw(EMANE::StartException);
  
  /**
   *
   * @brief stop
   *
   */
  void stop()
    throw(EMANE::StopException);
  
  /**
   *
   * @brief destroy
   *
   */
  void destroy()
    throw();

  /**
  *
  * @brief process upstream packet
  *
  * @param rHeader header
  * @param rPacket packet
  * @param rControlMessage control
  *
  */
   void processUpstreamPacket(const EMANE::CommonPHYHeader & rHeader,
                              EMANE::UpstreamPacket & rPacket,
                              const EMANE::ControlMessage & rControlMessage);  

  /**
  *
  * @brief process downstream control
  *
  * @param rControlMessage control
  *
  */
  void processDownstreamControl(const EMANE::ControlMessage & rControlMessage);
  
  /**
  *
  * @brief process downstream packet
  *
  * @param rPacket packet
  * @param rControlMessage control
  *
  */
  void processDownstreamPacket(EMANE::DownstreamPacket & rPacket,
                               const EMANE::ControlMessage & rControlMessage);
  /**
  *
  * @brief process event
  *
  * @param rEventId event id
  * @param rState object state
  *
  */
  void processEvent(const EMANE::EventId & rEventId, const EMANE::EventObjectState & rState);
  
    
  /**
  *
  * @brief processTimedEvent
  *
  * @param taskType  task type
  * @param eventId   event id
  * @param tv        timeout
  * @param arg       opaque data
  *
  */
  void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);


 private:
   typedef std::set <std::string>  StringSet;
   typedef StringSet::iterator     StringSetIter;

   StringSet registeredStatistics_;

   enum ANTENNA_TYPE { ANTENNA_TYPE_NONE = 0, ANTENNA_TYPE_OMNI = 1, ANTENNA_TYPE_UNI = 2 };

   PathLossManager  pathLossManager_;

   NoiseManager  noiseManager_;

   AntennaProfileManager  antennaProfileManager_;

   ACE_UINT16 u16TxSequenceNumber_;

   ACE_UINT16 u16RegistrationId_;

   ACE_UINT16 u16SubId_;

   ACE_UINT64 u64FrequencyHz_;

   NoiseManager::FrequencyOfInterestSet foi_;

   float fSystemNoiseFiguredB_;

   float fReceiverSensitivitydBm_;

   double dReceiverSensitivityMilliWatt_;

   EMANE::UniversalPhyControlSendMessage defaultTxControl_;

   bool bFOIFilterEnable_;

   bool checkModulation(const EMANE::PHYTxFrequencyInfoItems & items, ACE_UINT16 u16PhySubId);

   void prependUniversalPhyHeader(EMANE::DownstreamPacket & rPacket, ACE_UINT16 u16MacId) const;

   void getUniversalPhyHeader (EMANE::UpstreamPacket & rPacket, UniversalPhyHeader & hdr) const;

   bool checkCommonPHYHeader(EMANE::CommonPHYHeader & rCommonPHYHeader);

   void getAdditionTransmittersSet (const EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & vec, EMANE::NEMIdSet & set) const;

   void setPHYAdditionalTransmitters(EMANE::CommonPHYHeader & phyHeader,
          const EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & vec) const;

   void setPHYFrequencyInfo(EMANE::CommonPHYHeader & phyHeader, const EMANE::PHYTxFrequencyInfoItems & vec) const
          throw(EMANE::CommonPHYHeaderException);

   void setPHYAntennaProfile(EMANE::CommonPHYHeader & phyHeader, const EMANE::UniversalPhyControlSendMessage & ctrl);

   void updateAntennaProfile(const ACE_UINT16 u16ProfileId, const float fAzimuthDegrees, const float fElevationDegrees);

   void getAntennaProfile(const EMANE::CommonPHYHeader & phyHeader, UniversalPHY::AntennaProfile & antennaProfile) const;

   void getAllTransmitters(const EMANE::CommonPHYHeader & phyHeader, 
                          const EMANE::PacketInfo & pinfo, EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & items) const;

   void registerAllStatistics();

   void unregisterAllStatistics();

   
   EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToReceiverSensitivity_;
   EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToRegistrationId_;
   EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToInvalidSubId_;
   EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToSubIdMismatch_;
   EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToNoPathLossInfo_;
   EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToFoi_;
   EMANE::StatisticUnsignedInteger32 numUpstreamDiscard_;
   EMANE::StatisticUnsignedInteger32 numUpstreamPacketFromOTA_;
   EMANE::StatisticUnsignedInteger32 numDownstreamPacketToOTA_;
   EMANE::StatisticUnsignedInteger32 numUpstreamPacketToMAC_;
   EMANE::StatisticUnsignedInteger32 numDownstreamPacketFromMAC_;
 };
}

#endif //UNIVERSALPHY_UNIVERSALPHYLAYER_HEADER_
