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

#include "phyapitestshim.h"

#include "emaneutils/parameterconvert.h"

#include <sstream>

#include <ace/OS_NS_sys_time.h>

namespace
{
  EMANE::ConfigurationDefinition defs[] = 
  {
   
    /* required, default, count, name,                value,     type, description */
    {false,      true,    1,     "packetsize",        "128",     0,    "pkt size"},
    {false,      true,    1,     "packetrate",        "1",       0,    "pkt rate"},
    {false,      true,    1,     "destination",       "0xFFFF",  0,    "destination nem"},
    {false,      false,   1,     "bandwidth",         "",        0,    "bandwidth in Hz"},
    {false,      false,   1,     "antennaprofileid",  "",        0,    "antenna profile id"},
    {false,      false,   1,     "antennaazimuth",    "",        0,    "antenna azimuth in deg"},
    {false,      false,   1,     "antennaelevation",  "",        0,    "antenna elevation in deg"},
    {false,      false,   1,     "txpower",           "",        0,    "tx powwer in dBm"},
    {false,      false,   255,   "transmitter",       "",        0,    "additional transmitter"},
    {false,      false,   255,   "frequency",         "",        0,    "tx frequency info"},

    {0,0,0,0,0,0,0},
  };

  const char * pzLayerName = "PHYAPITestShimLayer";

  const ACE_UINT32 EVENT_TYPE_TX = 1;

  static ACE_UINT8 PAYLOAD_BUFFER[0xffff] = { 0xBE };
}

EMANE::PHYAPITestShimLayer::PHYAPITestShimLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService) :
  ShimLayerImplementor(id, pPlatformService),
  u16PacketSize_(0),
  tvTxInterval_(ACE_Time_Value::zero),
  txTimedEventId_(0),
  dst_(EMANE::NEM_BROADCAST_MAC_ADDRESS)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}


EMANE::PHYAPITestShimLayer::~PHYAPITestShimLayer() 
{ }


void EMANE::PHYAPITestShimLayer::initialize() 
  throw(EMANE::InitializeException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s", id_, pzLayerName, __func__);
}


void EMANE::PHYAPITestShimLayer::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s", id_, pzLayerName, __func__);

  Component::configure(items);
}


void EMANE::PHYAPITestShimLayer::start()
  throw(EMANE::StartException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s", id_, pzLayerName, __func__);

  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();

  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "packetsize")
                {
                  // set pkt payload size
                  u16PacketSize_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %hu bytes", 
                                         id_, pzLayerName, __func__, iter->first.c_str(), u16PacketSize_);
                }
              else if(iter->first == "packetrate")
                {
                  // get rate (pps)
                  const float fPacketRate = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %3.2f pps", 
                                         id_, pzLayerName, __func__, iter->first.c_str(), fPacketRate);

                  if(fPacketRate > 0.0)
                   {
                     // set the pkt interval
                     tvTxInterval_.set(1.0 / fPacketRate);
                  }
                }
              else if(iter->first == "destination")
                {
                  // set the NEM dest
                  dst_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16(1);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %hu", 
                                         id_, pzLayerName, __func__, iter->first.c_str(), dst_);
                }
              else if(iter->first == "bandwidth")
                {
                  // get the value
                  const ACE_UINT64 u64TxFrequency = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT64(1);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %s", 
                                         id_, pzLayerName, __func__, 
                                         iter->first.c_str(), EMANEUtils::formatFrequency(u64TxFrequency).c_str());

                  // set the ctrl msg 
                  txCtrl_.setBandWidthHz(u64TxFrequency);
                }
              else if(iter->first == "antennaprofileid")
                {
                  // get the value
                  const ACE_UINT16 u16ProfileId = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %hu", 
                                         id_, pzLayerName, __func__, iter->first.c_str(), u16ProfileId);
 
                  // set the ctrl msg 
                  txCtrl_.setAntennaProfileId(u16ProfileId);
                }
              else if(iter->first == "antennaazimuth")
                {
                  // get the value
                  const float fAzimuth = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(0.0, 360.0);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %f deg", 
                                         id_, pzLayerName, __func__, iter->first.c_str(), fAzimuth);
 
                  // set the ctrl msg 
                  txCtrl_.setAntennaAzimuthDegrees(fAzimuth);
                }
              else if(iter->first == "antennaelevation")
                {
                  // get the value
                  const float fElevation = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(-90.0, 90.0);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %f deg", 
                                         id_, pzLayerName, __func__, iter->first.c_str(), fElevation);
 
                  // set the ctrl msg 
                  txCtrl_.setAntennaElevationDegrees(fElevation);
                }
              else if(iter->first == "txpower")
                {
                  // get the value
                  const float fTxPower = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s = %f dBm", 
                                         id_, pzLayerName, __func__, iter->first.c_str(), fTxPower);
 
                  // set the ctrl msg 
                  txCtrl_.setTxPowerdBm(fTxPower);
                }
              else if(iter->first == "transmitter")
                {
                  // scan ctrl values    
                  const std::vector<std::string> tokens = EMANEUtils::getTokens(iter->second.item_.getValue(), ":");

                  // check num tokens nem:pwr
                  if(tokens.size() == 2)
                   {
                      ACE_UINT16 u16TxNEM  = EMANEUtils::ParameterConvert(tokens[0]).toUINT16();
                      float      fTxPower  = EMANEUtils::ParameterConvert(tokens[1]).toFloat();

                      pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s NEM = %hu, txpwr = %f dBm", 
                                             id_, pzLayerName, __func__, iter->first.c_str(), u16TxNEM, fTxPower);

                      // add to the ati
                      ati_.push_back(EMANE::UniversalPhyControlSendMessage::TransmitterInfo(u16TxNEM, fTxPower));
                   }
                  else
                   {
                      pPlatformService_->log(EMANE::ERROR_LEVEL,"SHIMI %03d %s::%s invalid format %s (%s), expected (NEM:txpwr)", 
                                             id_, pzLayerName, __func__, 
                                             iter->first.c_str(), iter->second.item_.getValue().c_str());

                      std::stringstream ssDescription;
                      ssDescription << "PHYAPITestShimLayer: Invalid format " << iter->first << std::ends;
                      throw EMANE::StartException(ssDescription.str());
                   }
                }
              else if(iter->first == "frequency")
                {
                  // scan ctrl values    
                  const std::vector<std::string> tokens = EMANEUtils::getTokens(iter->second.item_.getValue(), ":");

                  // check num tokens freq:duration:offset
                  if(tokens.size() == 3)
                   {
                      const ACE_UINT64 u64TxFrequency = EMANEUtils::ParameterConvert(tokens[0]).toUINT64();

                      ACE_Time_Value tvDuration, tvOffset;

                      tvDuration.set(EMANEUtils::ParameterConvert(tokens[1]).toFloat() / 1.0e6);

                      tvOffset.set(EMANEUtils::ParameterConvert(tokens[2]).toFloat() / 1.0e6);

                      pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s %s freq %s, duration %ld:%06ld, offset %ld:%06ld", 
                                             id_, pzLayerName, __func__, 
                                             iter->first.c_str(), 
                                             EMANEUtils::formatFrequency(u64TxFrequency).c_str(),
                                             tvDuration.sec(),
                                             tvDuration.usec(),
                                             tvOffset.sec(),
                                             tvOffset.usec());

                      // add to the fi
                      fi_.push_back(EMANE::PHYTxFrequencyInfo(u64TxFrequency, tvOffset, tvDuration));
                   }
                  else
                   {
                      pPlatformService_->log(EMANE::ERROR_LEVEL,"SHIMI %03d %s::%s invalid format %s (%s), expected (freq:duration:offset)", 
                                             id_, pzLayerName, __func__, 
                                             iter->first.c_str(), iter->second.item_.getValue().c_str());

                      std::stringstream ssDescription;
                      ssDescription << "PHYAPITestShimLayer: Invalid format " << iter->first << std::ends;
                      throw EMANE::StartException(ssDescription.str());
                   }
                }

            }
          else if(iter->second.bRequired_)
            {
              pPlatformService_->log(EMANE::ERROR_LEVEL,"SHIMI %03d %s::%s missing %s", 
                                     id_, pzLayerName, __func__, iter->first.c_str());

              std::stringstream ssDescription;
              ssDescription << "PHYAPITestShimLayer: Missing configuration item " << iter->first << std::ends;
              throw EMANE::StartException(ssDescription.str());
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream << "PHYAPITestShimLayer: Parameter " << iter->first << ": " << exp.what() << std::ends;
      throw EMANE::StartException(sstream.str());
    }
}



void EMANE::PHYAPITestShimLayer::stop()
  throw(EMANE::StopException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s", id_, pzLayerName, __func__);

  if(txTimedEventId_ != 0)
   {
     pPlatformService_->cancelTimedEvent(txTimedEventId_);

     txTimedEventId_ = 0;
   }
}



void EMANE::PHYAPITestShimLayer::postStart()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s", id_, pzLayerName, __func__);

  if(tvTxInterval_ > ACE_Time_Value::zero)
   {
     // schedule the event id, pkt data, timeout (absolute time), interval time
     txTimedEventId_ = pPlatformService_->scheduleTimedEvent(EVENT_TYPE_TX, 
                                                             NULL, 
                                                             ACE_OS::gettimeofday() + tvTxInterval_, 
                                                             tvTxInterval_);
   }
}



void EMANE::PHYAPITestShimLayer::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s", id_, pzLayerName, __func__);
}



void EMANE::PHYAPITestShimLayer::processUpstreamControl(const EMANE::ControlMessage &)
{
  // we do not take upstream control (yet)
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s, ignore", id_, pzLayerName, __func__);
#endif
}



void EMANE::PHYAPITestShimLayer::processDownstreamControl(const EMANE::ControlMessage &)
{
  // we do not take downstream control (yet)
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s, ignore", id_, pzLayerName, __func__);
#endif
}



void EMANE::PHYAPITestShimLayer::processUpstreamPacket(EMANE::UpstreamPacket & pkt, const EMANE::ControlMessage & msg)
{
   // get current time
   const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();

   // get pkt info
   const EMANE::PacketInfo pinfo = pkt.getPacketInfo();

   // so overwrite any items that may have been set by the caller for this pkt 
   if((msg.getMajorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_MAJOR_ID) &&
      (msg.getMinorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_RECV_MINOR_ID))
     {
       // phy control recv message (or so we hope)
       const EMANE::UniversalPhyControlRecvMessage ctrl(msg);

#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "SHIMI %03hu %s::%s: src %hu, dst %hu, len %zu, %s",
                              id_, pzLayerName, __func__,
                              pinfo.source_,
                              pinfo.destination_,
                              pkt.length(),
                              ctrl.format(tvCurrentTime).c_str());
#endif
     }
   else
     {
       pPlatformService_->log(EMANE::ERROR_LEVEL,
                              "SHIMI %03d %s::%s, expected recv control message missing major: %d minor: %d",
                              id_,
                              pzLayerName,
                              __func__,
                              msg.getMajorIdentifier(),
                              msg.getMinorIdentifier());
     }
}



void EMANE::PHYAPITestShimLayer::processDownstreamPacket(EMANE::DownstreamPacket &, const EMANE::ControlMessage &)
{
  // we do not take traffic (yet)
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s, ignore", id_, pzLayerName, __func__);
#endif
}



void EMANE::PHYAPITestShimLayer::processEvent(const EMANE::EventId & eventId, const EMANE::EventObjectState &)
{
  // we do not handle events (yet)
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s, ignore event id %d", 
                         id_, pzLayerName, __func__, eventId);
#else
  (void) eventId;
#endif
}



void EMANE::PHYAPITestShimLayer::processTimedEvent(ACE_UINT32 eventType, long, const ACE_Time_Value &, const void *)
{
  // the tx event fired
  if(eventType == EVENT_TYPE_TX)
   {
     // the pkt
     EMANE::DownstreamPacket pkt(EMANE::PacketInfo(id_, dst_, 0), PAYLOAD_BUFFER, u16PacketSize_);

     // set ati
     txCtrl_.setAdditionalTransmitters (ati_);

     // if we have freq info
     if(fi_.empty() == false)
      {
        // set fi
        txCtrl_.setFrequencyInfo (fi_);
      }
     else
      {
        // else no freq info
        txCtrl_.setFrequencyInfo (EMANE::PHYTxFrequencyInfoItems(1, EMANE::PHYTxFrequencyInfo()));
      }

#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIMI %03d %s::%s, %s", 
                            id_, pzLayerName, __func__, txCtrl_.format().c_str());
#endif


     // send pkt to phy 
     sendDownstreamPacket(pkt, txCtrl_.buildControlMessage());
   }
}



DECLARE_SHIM_LAYER(EMANE::PHYAPITestShimLayer);
