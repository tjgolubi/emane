/*
 * Copyright (c) 2008,2009,2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef RFPIPEMAC_MACLAYER_HEADER_
#define RFPIPEMAC_MACLAYER_HEADER_

#include "emane/emanemaclayerimpl.h"
#include "emane/emanemactypes.h"
#include "emane/emanestatisticunsignedinteger32.h"

#include "emaneutils/flowcontrolmanager.h"

#include "rfpipemacdownstreamqueue.h"
#include "pcrmanager.h"

#include <ace/Time_Value.h>

namespace RFPIPEMAC {

/**
 *
 * @class MacLayer
 *
 * @brief Implementation of the rf pipe mac layer.
 *
 */

 class MacLayer : public EMANE::MACLayerImplementor
 {
 public:
   /**
    * constructor
    *
    * @param id this NEM id.
    * @param pPlatformService reference to the platform service provider
    *
    */
   MacLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService);

   /**
    *
    * destructor
    *
    */
  ~MacLayer();


  // mac layer implementor api below
  
  void initialize()
    throw(EMANE::InitializeException);
  
  void configure(const EMANE::ConfigurationItems & items)
    throw(EMANE::ConfigureException);
  
  void start()
    throw(EMANE::StartException);
  
  void postStart();

  void stop()
    throw(EMANE::StopException);
  
  void destroy()
    throw();

  void processUpstreamControl(const EMANE::ControlMessage &msg);


  void processUpstreamPacket(const EMANE::CommonMACHeader & hdr,
                             EMANE::UpstreamPacket & pkt,
                             const EMANE::ControlMessage &msg);  
  
  void processDownstreamControl(const EMANE::ControlMessage &msg);
 

  void processDownstreamPacket(EMANE::DownstreamPacket &pkt,
                               const EMANE::ControlMessage &msg);


  void processEvent(const EMANE::EventId &eventId,
                    const EMANE::EventObjectState &state);

  void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);

 private:

  /**
   *
   * @brief  the emane rf pipe registration id
   *
   */
 static const EMANE::RegistrationId type_ = EMANE::REGISTERED_EMANE_MAC_RF_PIPE;


  /**
   *
   * @brief  tx control data
   *
   */

 struct TxControlData
  { 
    ACE_UINT64 u64DataRatebps_;
    bool       bDataRateSet_;

    ACE_UINT64 u64FrequencyHz_;
    bool       bFrequencySet_;

    float      fTransmitPowerdBm_;
    bool       bTxPowerSet_;


     /**
      *
      * @brief  tx control data initializer.
      *
      * @param   u64DataRatebps datarate in bps
      * @param   u64FrequencyHz center frequency in Hz
      * @param   fTransmitPowerdBm transmit power in dBm
      *
      */
    TxControlData(ACE_UINT64 u64DataRatebps,
                  ACE_UINT64 u64FrequencyHz,
                  float      fTransmitPowerdBm):
      u64DataRatebps_(u64DataRatebps),
      bDataRateSet_(true),
      u64FrequencyHz_(u64FrequencyHz),
      bFrequencySet_(true),
      fTransmitPowerdBm_(fTransmitPowerdBm),
      bTxPowerSet_(true)
    {}
 
     /**
      *
      * @brief  tx control data initializer.
      *
      * @param   u64DataRatebps datarate in bps
      *
      */
    TxControlData(ACE_UINT64 u64DataRatebps) :
      u64DataRatebps_(u64DataRatebps),
      bDataRateSet_(true),
      u64FrequencyHz_(0),
      bFrequencySet_(false),
      fTransmitPowerdBm_(0),
      bTxPowerSet_(false)
    {}
   

   /**
    *
    * @brief  overide method for tx control data
    *
    * @param  rhs reference to tx control data used to overide existing values.
    *
    */

    void overide(const TxControlData & rhs)
     {
        // overide datarate if set
        if(rhs.bDataRateSet_ == true)
          {
            u64DataRatebps_ = rhs.u64DataRatebps_;

            bDataRateSet_ = true;
          }

        // overide frequency if set
        if(rhs.bFrequencySet_ == true)
          {
            u64FrequencyHz_ = rhs.u64FrequencyHz_;

            bFrequencySet_ = true;
          }

        // overide txpower if set
        if(rhs.bTxPowerSet_ == true)
          {
            fTransmitPowerdBm_ = rhs.fTransmitPowerdBm_;

            bTxPowerSet_ = true;
          }
     }
   };

  typedef std::map<EMANE::NEMId, TxControlData> TxControlDataMap;
  typedef TxControlDataMap::iterator TxControlDataMapIter;

  TxControlDataMap txControlDataMap_;

  ACE_thread_t downstreamThread_;
  
  RFPIPEMAC::DownstreamQueue downstreamQueue_;
  
  bool bRunning_;
  
  ACE_UINT16 u16TxSequenceNumber_;

  EMANEUtils::FlowControlManager flowControlManager_;

  RFPIPEMAC::PCRManager  pcrManager_;

  // config items
  bool            bPromiscuousMode_;

  ACE_UINT64      u64DataRatebps_;

  ACE_Time_Value  tvJitter_;

  ACE_Time_Value  tvDelay_;

  bool            bFlowControlEnable_;

  ACE_UINT16      u16FlowControlTokens_;

  bool            bEnableTightTiming_;

  std::string     sPCRCurveURI_;


  ACE_THR_FUNC_RETURN processDownstreamQueue();  

  ACE_Time_Value randomize(const ACE_Time_Value &rTimeValue);

  ACE_Time_Value getDuration(ACE_UINT64 u64DataRatebps, size_t lengthInBytes);

  TxControlData getTxControlData(EMANE::NEMId id);

  bool checkPOR(float fSINR, size_t packetSize);

  EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToSinr_;
  EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToMacId_;
  EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToQueueRejection_;
  EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToNotForThisNEM_;
  EMANE::StatisticUnsignedInteger32 numUpstreamDiscardDueToLackOfRxCtrlInfo_;
  EMANE::StatisticUnsignedInteger32 numUpstreamPacketFromPHY_;
  EMANE::StatisticUnsignedInteger32 numDownstreamPacketToPHY_;
  EMANE::StatisticUnsignedInteger32 numDownstreamDiscardDueToFlowControl_;
  EMANE::StatisticUnsignedInteger32 numUpstreamPacketToTransport_;
  EMANE::StatisticUnsignedInteger32 numDownstreamPacketFromTransport_;

 };
}

#endif //RFPIPEMAC_MACLAYER_HEADER_
