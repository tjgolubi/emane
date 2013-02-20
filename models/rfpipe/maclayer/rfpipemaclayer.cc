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

#include "rfpipemaclayer.h"

#include "emane/emaneconfigurationrequirement.h"

#include "emaneutils/parameterconvert.h"
#include "emaneutils/spawnmemberfunc.h"

#include "emanecontrolmessages/universalphycontrolsendmessage.h"
#include "emanecontrolmessages/universalphycontrolrecvmessage.h"

#include <ace/OS_NS_time.h>
#include <ace/OS_NS_sys_select.h>

#include <sstream>


namespace
{
  const EMANE::ConfigurationDefinition defs[] =
    {
      /* required, default, count, name,                       value,               type, description */
      {false,      true,    1,     "enablepromiscuousmode",     "off",              0,    "enable promiscuous mode"},
      {false,      false,   0,     "transmissioncontrolmap",    "" ,                0,    "transmission control data"},
      {true,       true ,   1,     "datarate",                  "1M",               0,    "datarate bps"},
      {true,       true,    1,     "jitter",                    "0.0",              0,    "jitter in usec"},
      {true,       true,    1,     "delay",                     "0.0",              0,    "delay in usec"},
      {false,      true,    1,     "flowcontrolenable",         "off",              0,    "flow control enable"},
      {false,      true,    1,     "flowcontroltokens",         "10",               0,    "number of flow control tokens"},
      {false,      true,    1,     "enabletighttiming",         "off",              0,    "enable tight timing for pkt delay"},
      {true,       false ,  1,     "pcrcurveuri",               "",                 0,    "pcr curve uri"},
      {0,0,0,0,0,0,0},
    };

    const char * pzLayerName = "RFPipeMACLayer";
}



RFPIPEMAC::MacLayer::MacLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService):
  MACLayerImplementor(id, pPlatformService),
  downstreamThread_(0),
  downstreamQueue_(pPlatformService_),
  bRunning_(false),
  u16TxSequenceNumber_(0),
  flowControlManager_(*this),
  pcrManager_(id, pPlatformService_)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);

  pPlatformService_->registerStatistic("numUpstreamDiscardDueToSinr",             &numUpstreamDiscardDueToSinr_);
  pPlatformService_->registerStatistic("numUpstreamDiscardDueToMacId",            &numUpstreamDiscardDueToMacId_);
  pPlatformService_->registerStatistic("numUpstreamDiscardDueToQueueRejection",   &numUpstreamDiscardDueToQueueRejection_);
  pPlatformService_->registerStatistic("numUpstreamDiscardDueToNotForThisNEM",    &numUpstreamDiscardDueToNotForThisNEM_);
  pPlatformService_->registerStatistic("numUpstreamDiscardDueToLackOfRxCtrlInfo", &numUpstreamDiscardDueToLackOfRxCtrlInfo_);
  pPlatformService_->registerStatistic("numDownstreamDiscardDueToFlowControl",    &numDownstreamDiscardDueToFlowControl_);
  pPlatformService_->registerStatistic("numUpstreamPacketFromPHY",                &numUpstreamPacketFromPHY_);
  pPlatformService_->registerStatistic("numDownstreamPacketToPHY",                &numDownstreamPacketToPHY_);
  pPlatformService_->registerStatistic("numUpstreamPacketToTransport",            &numUpstreamPacketToTransport_);
  pPlatformService_->registerStatistic("numDownstreamPacketFromTransport",        &numDownstreamPacketFromTransport_);
}


RFPIPEMAC::MacLayer::~MacLayer()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);

  pPlatformService_->unregisterStatistic("numUpstreamDiscardDueToSinr");
  pPlatformService_->unregisterStatistic("numUpstreamDiscardDueToMacId");
  pPlatformService_->unregisterStatistic("numUpstreamDiscardDueToQueueRejection");
  pPlatformService_->unregisterStatistic("numUpstreamDiscardDueToNotForThisNEM");
  pPlatformService_->unregisterStatistic("numUpstreamDiscardDueToLackOfRxCtrlInfo");
  pPlatformService_->unregisterStatistic("numDownstreamDiscardDueToFlowControl");
  pPlatformService_->unregisterStatistic("numUpstreamPacketFromPHY");
  pPlatformService_->unregisterStatistic("numDownstreamPacketToPHY");
  pPlatformService_->unregisterStatistic("numUpstreamPacketToTransport");
  pPlatformService_->unregisterStatistic("numDownstreamPacketFromTransport");
}



void 
RFPIPEMAC::MacLayer::processUpstreamControl(const EMANE::ControlMessage &)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s, ignore", id_, pzLayerName, __func__);
#endif
}



void 
RFPIPEMAC::MacLayer::processDownstreamControl(const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s msg 0x%08X:0x%08X", 
      id_, pzLayerName, __func__, msg.getMajorIdentifier(), msg.getMinorIdentifier());
#endif

  if(msg.getMajorIdentifier() == EMANE::EMANE_FLOWCONTROL_MAJOR_ID)
   {
     flowControlManager_.processFlowControlMessage(msg);
   }
  else
   {
     sendDownstreamControl(msg);
   }
}


void 
RFPIPEMAC::MacLayer::processUpstreamPacket(const EMANE::CommonMACHeader & hdr,
                                           EMANE::UpstreamPacket & pkt,
                                           const EMANE::ControlMessage & msg)
{
  // bump statistic               
  ++numUpstreamPacketFromPHY_;

  if(hdr.getRegistrationId() != type_)
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: MAC Registration Id does not match, drop.",
                             id_, pzLayerName, __func__);


      // bump statistic
      ++numUpstreamDiscardDueToMacId_;

      // drop
      return;
    }

  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();

  // get packet info
  const EMANE::PacketInfo pinfo = pkt.getPacketInfo();
 
  // check for recv control info
  if((msg.getMajorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_MAJOR_ID) &&
     (msg.getMinorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_RECV_MINOR_ID))
    {
      // phy control recv message
      EMANE::UniversalPhyControlRecvMessage ctrl(msg);

      // get the frequency info for the pkt (a list is returned but we are only interested in the first value)
      const EMANE::PHYRxFrequencyInfoItems freqInfoVec = ctrl.getFrequencyInfo ();

      // get the transmitter info (a list is returned but we are only interested in the first value)
      const EMANE::UniversalPhyControlRecvMessage::TransmitterInfoItems transmitters = ctrl.getTransmitterInfo ();

      const float fRxPowerdBm = transmitters[0].getRxPowerdBm();

#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL,
          "MACI %03hu %s::%s: origin %hu, dst %hu, len %zu, %s",
            id_, pzLayerName, __func__,
            pinfo.source_,
            pinfo.destination_,
            pkt.length(),
            ctrl.format(tvCurrentTime).c_str());
#endif

      // check sinr
      if(checkPOR(fRxPowerdBm - freqInfoVec[0].getNoiseFloordBm(), pkt.length()) == false)
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL,
             "MACI %03hu %s::%s: origin %hu, dst %hu, rxpwr %3.2f dBm, noise floor %3.2f, drop",
             id_, pzLayerName, __func__,
             pinfo.source_,
             pinfo.destination_,
             fRxPowerdBm,
             freqInfoVec[0].getNoiseFloordBm());
#endif

         // bump statistic
         ++numUpstreamDiscardDueToSinr_;

         // drop
         return;
       }
      // check promiscous mode, destination is this nem or to all nem's
      else if(bPromiscuousMode_ || (pinfo.destination_ == id_) || (pinfo.destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS))
       {
         // set timeout = delay + jitter + pkt duration
         ACE_Time_Value tvTimeOut = tvDelay_ + randomize(tvJitter_) + freqInfoVec[0].getDuration();

         // if tight timing mode enabled subtract the ota time (now - tx time)
         if(bEnableTightTiming_)
           {
             tvTimeOut -= (tvCurrentTime - ctrl.getTxTime());
           }

         // if any delay is needed
         if(tvTimeOut > ACE_Time_Value::zero)
          {
            // create a copy of the pkt, the callback will take ownership of the pkt and itself
            const EMANE::UpstreamPacket *p = new EMANE::UpstreamPacket (pkt);

            // schedule the event id, pkt data, timeout (absolute time), one shot
            const long eventId = pPlatformService_->scheduleTimedEvent(0, p, tvTimeOut + tvCurrentTime);

             // event was scheduled
            if(eventId >= 0)
             {
#ifdef VERBOSE_LOGGING
               pPlatformService_->log(EMANE::DEBUG_LEVEL,
                   "MACI %03hu %s::%s: origin %hu, dst %hu, timed event %ld",
                    id_, pzLayerName, __func__,
                    pinfo.source_,
                    pinfo.destination_,
                    eventId);
#endif

                // done
                return;
             }
           else
             {
               // cleanup
               delete p;

#ifdef VERBOSE_LOGGING
               pPlatformService_->log(EMANE::DEBUG_LEVEL,
                   "MACI %03hu %s::%s: failed to add timed event for origin %hu, dst %hu, drop",
                    id_, pzLayerName, __func__,
                    pinfo.source_,
                    pinfo.destination_);
#endif


               // bump statistic               
               ++numUpstreamDiscardDueToQueueRejection_;

               // drop
               return;
             }
          }
         else
          {
#ifdef VERBOSE_LOGGING
            pPlatformService_->log(EMANE::DEBUG_LEVEL,
                "MACI %03hu %s::%s: origin %hu, dst %hu, forward upstream",
                id_, pzLayerName, __func__,
                pinfo.source_,
                pinfo.destination_);
#endif
            // send upstream now
            sendUpstreamPacket(pkt);

            // done
            return;
          }
       }
      else
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s: not for this nem, ignore pkt origin %hu, dst %hu, drop",
             id_, pzLayerName, __func__,
             pinfo.source_,
             pinfo.destination_);
#endif

         // bump statistic               
         ++numUpstreamDiscardDueToNotForThisNEM_;

         // drop
         return;
       }
    }
  else
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s: universal phy recv control message not provided from origin %hu, drop",
           id_, pzLayerName, __func__, pinfo.source_);
#endif

        // bump statistic               
        ++numUpstreamDiscardDueToLackOfRxCtrlInfo_;

       // drop
       return;
    }
}


void 
RFPIPEMAC::MacLayer::processDownstreamPacket(EMANE::DownstreamPacket & pkt,
                                             const EMANE::ControlMessage &)
{
  // bump statistic               
  ++numDownstreamPacketFromTransport_;

  // check flow control
  if(bFlowControlEnable_ == true)
    {
       const EMANEUtils::FlowControlManager::FlowControlStatus flowControlStatus = flowControlManager_.removeToken();

       if(flowControlStatus.bStatus_ == false)
        {
          pPlatformService_->log(EMANE::ERROR_LEVEL,
              "MACI %03hu %s::%s: failed to remove token, %hu/%hu flow control tokens available, drop packet",
               id_, pzLayerName, __func__, flowControlStatus.ui16TokensAvailable_, flowControlStatus.ui16ShadowTokenCount_);

           // bump statistic               
           ++numDownstreamDiscardDueToFlowControl_;

           // drop
           return;
        }
       else
        {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log(EMANE::DEBUG_LEVEL,
              "MACI %03hu %s::%s: removed token, %hu/%hu flow control tokens available",
              id_, pzLayerName, __func__, flowControlStatus.ui16TokensAvailable_, flowControlStatus.ui16ShadowTokenCount_);
#endif
        }
    }

  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();

  // get packet info
  const EMANE::PacketInfo pinfo = pkt.getPacketInfo();

  // get tx control data for this dst
  const TxControlData txCtrl = getTxControlData(pinfo.destination_);

  // get duration
  const ACE_Time_Value tvDuration = getDuration(txCtrl.u64DataRatebps_, pkt.length());

  // universal phy control send message
  EMANE::UniversalPhyControlSendMessage msg;

  // set freq info values
  EMANE::PHYTxFrequencyInfo freqInfo((txCtrl.bFrequencySet_ == true) ? // if tx frequency is known for the dst
                                       txCtrl.u64FrequencyHz_ : 0,     // set it otherwise use 0 and the phy will use its default
                                     ACE_Time_Value::zero,             // tx offset (T = 0) no offset
                                     tvDuration);                      // duration

  // set the freq info (we only use 1 frequency, hence 1 value)
  msg.setFrequencyInfo(EMANE::PHYTxFrequencyInfoItems(1, freqInfo));

  // if tx power is known for this dst
  if(txCtrl.bTxPowerSet_ == true)
   {
     // set tx power 
     msg.setTxPowerdBm (txCtrl.fTransmitPowerdBm_);
   }

#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,
      "MACI %03hu %s::%s: origin %hu, dst %hu, len %zu, %s",
       id_, pzLayerName, __func__,
       pinfo.source_,
       pinfo.destination_,
       pkt.length(),
       msg.format().c_str());
#endif

  // create downstream entry
  RFPIPEMAC::DownstreamQueueEntry entry(pkt,                       // pkt
                                        msg.buildControlMessage(), // ctrl
                                        u16TxSequenceNumber_,      // sequence number
                                        tvCurrentTime,             // current time (SOT)
                                        tvDuration);               // duration

  // bump tx sequence number
  ++u16TxSequenceNumber_;

  // enqueue entry
  downstreamQueue_.enqueue(entry);
}



void 
RFPIPEMAC::MacLayer::initialize()
  throw(EMANE::InitializeException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);
}



void 
RFPIPEMAC::MacLayer::configure(const EMANE::ConfigurationItems &items)
  throw(EMANE::ConfigureException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);

  Component::configure(items);
}


void 
RFPIPEMAC::MacLayer::start()
  throw(EMANE::StartException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);

  EMANE::ConfigurationRequirements::const_iterator iter = configRequirements_.begin();

  try
    {
      for(;iter != configRequirements_.end(); ++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "enablepromiscuousmode")
                {
                  bPromiscuousMode_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s %s = %s", 
                      id_, pzLayerName, __func__, iter->first.c_str(), bPromiscuousMode_ ? "on" : "off");
                }
              else if(iter->first == "transmissioncontrolmap")
                {
                  // scan transmission ctrl values    
                  const std::vector<std::string> tokens = EMANEUtils::getTokens(iter->second.item_.getValue(), ":");

                  // check num tokens
                  if(tokens.size() == 4) 
                    {
                      const ACE_UINT16 u16DestNEMId      = EMANEUtils::ParameterConvert(tokens[0]).toUINT16();
                      const ACE_UINT64 u64DataRatebps    = EMANEUtils::ParameterConvert(tokens[1]).toUINT64();
                      const ACE_UINT64 u64FrequencyHz    = EMANEUtils::ParameterConvert(tokens[2]).toUINT64();
                      const float      fTransmitPowerdBm = EMANEUtils::ParameterConvert(tokens[3]).toFloat();

                      // dst 0 not allowed
                      if(u16DestNEMId != 0)
                        {
                          // tx control data all params
                          const TxControlData txCtrl (u64DataRatebps, u64FrequencyHz, fTransmitPowerdBm);

                          // check for duplicates
                          if(txControlDataMap_.insert(std::make_pair(u16DestNEMId, txCtrl)).second == false)
                           {
                             throw EMANE::StartException("configuration item 'transmissioncontrolmap' duplicate destination");
                           }
                          else
                           {
                             pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s %s: dst = %hu, datarate = %ju, freq = %ju Hz, txpwr %4.2f dBm", 
                                 id_, pzLayerName, __func__, iter->first.c_str(), u16DestNEMId, u64DataRatebps, u64FrequencyHz, fTransmitPowerdBm);
                           }
                        }
                      else
                        {
                          throw EMANE::StartException("configuration item 'transmissioncontrolmap' destination must be non zero");
                        }
                    }
                  else
                    {
                      pPlatformService_->log(EMANE::ERROR_LEVEL, "MACI %03hu %s::%s %s:%s", 
                                             id_, pzLayerName, __func__, iter->first.c_str(), iter->second.item_.getValue().c_str());

                      throw EMANE::StartException("bad configuration item 'transmissioncontrolmap' format '<Dst>:<bps>:<Hz>:<dBm>'");
                    }
                }
              else if(iter->first == "datarate")
                {
                  u64DataRatebps_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT64(1);
                    
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s %s = %ju",
                      id_, pzLayerName, __func__, iter->first.c_str(), u64DataRatebps_);
                }
              else if(iter->first == "jitter")
                {
                  const float fValueUsec = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat();

                  tvJitter_.set(fValueUsec / 1.0e6);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s %s = %ld:%06ld", 
                      id_, pzLayerName, __func__, iter->first.c_str(), tvJitter_.sec(), tvJitter_.usec());
                }
              else if(iter->first == "delay")
                {
                  const float fValueUsec = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(0);

                  tvDelay_.set(fValueUsec / 1.0e6);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s %s = %ld:%06ld", 
                      id_, pzLayerName, __func__, iter->first.c_str(), tvDelay_.sec(), tvDelay_.usec());
                }
              else if(iter->first == "flowcontrolenable")
                {
                  bFlowControlEnable_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s %s = %s", 
                      id_, pzLayerName, __func__, iter->first.c_str(), bFlowControlEnable_ ? "on" : "off");
                }
              else if(iter->first == "flowcontroltokens")
                {
                  u16FlowControlTokens_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16(1, 0xFFFF);
                    
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s %s = %hu",
                      id_, pzLayerName, __func__, iter->first.c_str(), u16FlowControlTokens_);
                }
              else if(iter->first == "enabletighttiming")
                {
                  bEnableTightTiming_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();
                  
                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s %s = %s", 
                      id_, pzLayerName, __func__, iter->first.c_str(), bEnableTightTiming_ ? "on" : "off");
                }
             else if(iter->first == "pcrcurveuri")
               {
                 sPCRCurveURI_ = iter->second.item_.getValue();

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s %s = %s",
                     id_, pzLayerName, __func__, iter->first.c_str(), sPCRCurveURI_.c_str());
               }
              else if(iter->second.bRequired_)
                {
                  pPlatformService_->log(EMANE::ERROR_LEVEL,"MACI %03hu %s::%s missing %s",
                      id_, pzLayerName, __func__, iter->first.c_str());
              
                  std::stringstream ssDescription;
                  ssDescription << pzLayerName << ": missing configuration item " << iter->first << std::ends;
                  throw EMANE::StartException(ssDescription.str());
                }
            }
        }
    }
   catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream << pzLayerName << ": parameter " << iter->first << ": " << exp.what() << std::ends;
      throw EMANE::StartException(sstream.str());
    }

  // load pcr curve
  pcrManager_.load(sPCRCurveURI_);

  // running
  bRunning_ = true;

  // start the downstream thread for queue processing
  EMANEUtils::spawn(*this, &MacLayer::processDownstreamQueue, &downstreamThread_);
}



void 
RFPIPEMAC::MacLayer::postStart()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);

  // check flow control enabled 
  if(bFlowControlEnable_ == true)
    {
      // start flow control 
      flowControlManager_.start(u16FlowControlTokens_);
    }
}



void 
RFPIPEMAC::MacLayer::stop()
  throw(EMANE::StopException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);

  // set running flag to false
  bRunning_ = false;

  // check flow control enabled
  if(bFlowControlEnable_ == true)
    {
      // stop the flow control manager
      flowControlManager_.stop();
    }

  // unblock the downstream queue
  downstreamQueue_.cancel();

  // check thread id
  if(downstreamThread_ != 0)
   {
      // join downstream thread
      ACE_OS::thr_join(downstreamThread_, NULL, NULL);

      // reset value
      downstreamThread_ = 0;
   }
}



void 
RFPIPEMAC::MacLayer::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);
}




void 
RFPIPEMAC::MacLayer::processEvent(const EMANE::EventId &, const EMANE::EventObjectState &)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s", id_, pzLayerName, __func__);
#endif
}




ACE_THR_FUNC_RETURN 
RFPIPEMAC::MacLayer::processDownstreamQueue()
{
  // last end of transmission time
  ACE_Time_Value tvLastEOTTime = ACE_Time_Value::zero;

  // while in the running mode
  while(bRunning_ == true) 
    {
      // get downstream entry, blocking call
      RFPIPEMAC::DownstreamQueueEntry entry = downstreamQueue_.dequeue();
      
      // if flow control enabled 
      if(bFlowControlEnable_ == true)
        {
          // add token
          flowControlManager_.addToken();
        }

      // no longer running
      if (bRunning_ == false)
        {
          break;
        }

      // send packet then wait the packet duration
      sendDownstreamPacket(EMANE::CommonMACHeader(type_), entry.pkt_, entry.ctrl_);

      // bump statistic               
      ++numDownstreamPacketToPHY_;

      // if current packet arrived while you were ~transmitting~ the previous packet
      // then reset the start-of-transmission time to the end-of-transmission of 
      // the previous sent packet.
      if(tvLastEOTTime > entry.tvSOT_)
        {
          entry.tvSOT_ = tvLastEOTTime;
        }
 
      // calculate the effective EOT time - add the calculated total duration to the
      // start-of-transmission time
      const ACE_Time_Value tvEOTTime = entry.tvSOT_ + entry.tvDuration_;

      // time to wait (relative time)
      ACE_Time_Value tvWaitTime;

      // wait until EOT
      while((tvWaitTime = (tvEOTTime - ACE_OS::gettimeofday())) > ACE_Time_Value::zero)
        {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s: packet seq: %hu delay remaining %ld:%06ld", 
              id_, pzLayerName, __func__,
              entry.seq_,
              tvWaitTime.sec(),
              tvWaitTime.usec());
#endif
     
          // wait     
          if(ACE_OS::select(0, 0, 0, 0, tvWaitTime) < 0)
           {
              pPlatformService_->log(EMANE::ERROR_LEVEL,"MACI %03hu %s::%s: packet seq: %hu invalid delay %ld:%06ld", 
                  id_, pzLayerName, __func__,
                  entry.seq_,
                  tvWaitTime.sec(),
                  tvWaitTime.usec());
           }
        }

      // finished waiting for the EOT
      tvLastEOTTime = tvEOTTime;
    }
 
  // thread terminated 
  return 0;
}



ACE_Time_Value 
RFPIPEMAC::MacLayer::randomize(const ACE_Time_Value & rInterval)
{
  // the result is +/- the interval
  ACE_Time_Value tvResult = ACE_Time_Value::zero;

  // if interval is greater then zero
  if(rInterval > ACE_Time_Value::zero) 
   {
     // scale up and roll the dice
     const ACE_UINT64 u64usec = pPlatformService_->getRandomNumber() % (2 * ((rInterval.sec() * 1000000) + rInterval.usec()));

     // set the time
     tvResult.set(static_cast<double>(u64usec / 1.0e6));

     // scale back
     tvResult -= rInterval;
   }

  // return result
  return tvResult;
}



ACE_Time_Value 
RFPIPEMAC::MacLayer::getDuration(ACE_UINT64 u64DataRatebps, size_t lengthInBytes)
{
  // the result
  ACE_Time_Value tvResult = ACE_Time_Value::zero;

  // if data rate is greater than zero
  if (u64DataRatebps > 0)
   {
     // seconds and fraction of seconds
     const double dDuration = ((lengthInBytes * 8.0) / (double) u64DataRatebps);

     // set the result
     tvResult.set(dDuration);
   }

  // return result
  return tvResult;
}



 
RFPIPEMAC::MacLayer::TxControlData 
RFPIPEMAC::MacLayer::getTxControlData(EMANE::NEMId id)
{
  // default tx control data 1 param
  TxControlData txCtrl(u64DataRatebps_);

  // check for tx control data
  const TxControlDataMapIter iter = txControlDataMap_.find(id);

  // transmission ctrl data found
  if(iter != txControlDataMap_.end())
    {
      // override transmission control data
      txCtrl.overide(iter->second);
    }

  // return tx ctrl data
  return txCtrl;
}




bool 
RFPIPEMAC::MacLayer::checkPOR(float fSINR, size_t packetSize)
{
   // find por
   const float fPCR = pcrManager_.getPCR(fSINR, packetSize);

   // random value from 0.0 to 1.0 inclusive
   const float fRandomValue = pPlatformService_->getRandomProbability();

   // pcr >= random value
   const bool bResult = (fPCR >= fRandomValue);

#ifdef VERBOSE_LOGGING
   pPlatformService_->log(EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: sinr %3.2f, pcr %3.2f %s rand %3.3f",
      id_, pzLayerName, __func__, fSINR, fPCR, bResult == true ? ">=" : "<", fRandomValue);
#endif

   // return result
   return bResult;       
}

 

 
void 
RFPIPEMAC::MacLayer::processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value & tvEventTime, const void *arg)
{
  // get processing delay
  const ACE_Time_Value tvProcessingDelay = ACE_OS::gettimeofday() - tvEventTime;

  // upstream packet
  EMANE::UpstreamPacket * p = (EMANE::UpstreamPacket *) arg;

  if(p != NULL)
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s task type %u, delay %06ld:%06ld, event id %ld, origin %hu, dst %hu, len %zd", 
         id_, pzLayerName, __func__,
         taskType,
         tvProcessingDelay.sec(),
         tvProcessingDelay.usec(),
         eventId,
         p->getPacketInfo().source_, 
         p->getPacketInfo().destination_, 
         p->length());
#else
   (void) taskType;
   (void) eventId;
#endif

     // send packet upstream
     sendUpstreamPacket(*p);

     // bump statistic               
     ++numUpstreamPacketToTransport_;

     // cleanup packet
     delete p;
   }
  else
   {
     pPlatformService_->log(EMANE::ERROR_LEVEL,"MACI %03hu %s::%s NULL pkt", id_, pzLayerName, __func__);
   }

   return;
}


DECLARE_MAC_LAYER(RFPIPEMAC::MacLayer);
