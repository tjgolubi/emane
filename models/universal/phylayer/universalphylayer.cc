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

#include "universalphylayer.h"
#include "emaneutils/parameterconvert.h"

#include "emane/emanephytypes.h"
#include "emane/emaneconstants.h"
#include "emane/emanecommonphyheaderoptions.h"

#include "emaneevents/antennaprofileevent.h"

#include "emanecontrolmessages/universalphycontrolantennaprofilemessage.h"
#include "emanecontrolmessages/otatransmittersmessage.h"

#include <ace/OS_NS_sys_time.h>

#include <math.h>

#include <iomanip>


namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
      /* required, default, count, name,                              value,        type, description */
      {true,       true,    1,     "antennagain",                     "0.0",        0,    "antenna gain in dBi"},
      {true,       true,    1,     "antennaazimuth",                  "0.0",        0,    "antenna azimuth in degrees"},
      {true,       true,    1,     "antennaelevation",                "0.0",        0,    "antenna elevation in degrees"},
      {false,      false,   1,     "antennaprofileid",                "",           0,    "antenna profile id"},
      {false,      false,   1,     "antennaprofilemanifesturi",       "",           0,    "antenna profile manifest URI"},
      {false,      true,    1,     "antennaprofileenable",            "off",        0,    "antenna profile mode on/off"},
      {true,       true,    1,     "bandwidth",                       "1M",         0,    "rf bandwidth in Hz"},
      {true,       true,    1,     "defaultconnectivitymode",         "on",         0,    "default connectivity mode on/off"},
      {true,       true,    1,     "frequency",                       "2.347G",     0,    "frequency in Hz"},
      {true,       true,    0,     "frequencyofinterest",             "2.347G",     0,    "frequency of interest"},
      {true,       true,    1,     "frequencyofinterestfilterenable", "on",         0,    "frequency of interest filter enable on/off"},
      {true,       true,    1,     "noiseprocessingmode",             "off",        0,    "enable noise processing on/off"},
      {true,       true,    1,     "pathlossmode",                    "pathloss",   0,    "path loss mode"},
      {true,       true,    1,     "systemnoisefigure",               "4.0",        0,    "system noise figure dB"},
      {true,       false,   1,     "subid",                           "",           0,    "subid"},
      {true,       true,    1,     "txpower",                         "0.0",        0,    "transmit power dBm"},
      {0,0,0,0,0,0,0},
    };

    // thermal noise level in dB
    const float THERMAL_NOISE_DB = -174.0;

    // this module name
    const char * MODULE = "UniversalPHYLayer";

    // min/max dBm power levels
    const float DBM_MIN = -327.0;
    const float DBM_MAX = 327.0;
  }

UniversalPHY::UniversalPHYLayer::UniversalPHYLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService):
  PHYLayerImplementor   (id, pPlatformService),
  pathLossManager_      (id, pPlatformService),
  noiseManager_         (id, pPlatformService),
  antennaProfileManager_(id, pPlatformService),
  u16TxSequenceNumber_(0),
  u16RegistrationId_(EMANE::REGISTERED_EMANE_PHY_UNIVERSAL),
  bFOIFilterEnable_(true)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);

  // register stats
  registerAllStatistics();
}


UniversalPHY::UniversalPHYLayer::~UniversalPHYLayer()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);

  // unregister stats
  unregisterAllStatistics();
}


void 
UniversalPHY::UniversalPHYLayer::initialize()
  throw(EMANE::InitializeException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);
}


void 
UniversalPHY::UniversalPHYLayer::configure(const EMANE::ConfigurationItems & rItems)
  throw(EMANE::ConfigureException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);
  Component::configure(rItems);
}


void 
UniversalPHY::UniversalPHYLayer::start()
  throw(EMANE::StartException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);

  // local config options
  bool bDefaultConnectivityEnable = true;

  bool bNoiseProcessingEnable = true;

  ACE_UINT16 u16ProfileId = 0;

  std::string sAntennaProfileManifestURI;

  bool bEnableAntennaProfile = false;

  UniversalPHY::PathLossManager::PathLossMode pathLossMode = UniversalPHY::PathLossManager::PATH_LOSS_MODE_NONE;

  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin(); 

  try
    {
      for(; iter != configRequirements_.end(); ++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "bandwidth")
               {
                 // enforce non-zero value
                 const ACE_UINT64 value = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT64(1);

                 // setup default tx control
                 defaultTxControl_.setBandWidthHz(value);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s", 
                                        id_, __func__, MODULE, iter->first.c_str(),  EMANEUtils::formatFrequency(value).c_str());
               }
              else if(iter->first == "antennagain")
               {
                 // enforce range inclusive
                 const float value = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(DBM_MIN, DBM_MAX);

                 // set antenna profile info
                 antennaProfileManager_.setFixedAntennaGaindBi(value);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %3.2f dBi",
                                        id_, __func__, MODULE, iter->first.c_str(), value);

               }
              else if(iter->first == "antennaazimuth")
               {
                 // enforce range inclusive
                 const float value = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(0.0, 360.0);

                 // setup default tx control
                 defaultTxControl_.setAntennaAzimuthDegrees(value);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %3.2f deg",
                                        id_, __func__, MODULE, iter->first.c_str(), value);

               }
              else if(iter->first == "antennaelevation")
               {
                 // enforce range inclusive
                 const float value = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(-90.0, 90.0);

                 // setup default tx control
                 defaultTxControl_.setAntennaElevationDegrees(value);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %3.2f deg",
                                        id_, __func__, MODULE, iter->first.c_str(), value);

               }
              else if(iter->first == "antennaprofilemanifesturi")
               {
                 sAntennaProfileManifestURI = iter->second.item_.getValue();

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                        id_, __func__, MODULE, iter->first.c_str(), sAntennaProfileManifestURI.c_str());
               }
              else if(iter->first == "antennaprofileid")
               {
                 u16ProfileId = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16();

                 // setup default tx control
                 defaultTxControl_.setAntennaProfileId(u16ProfileId);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %hu",
                                        id_, __func__, MODULE, iter->first.c_str(), u16ProfileId);
               }
              else if(iter->first == "antennaprofileenable")
               {
                 // enforce range inclusive
                 bEnableAntennaProfile = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                        id_, __func__, MODULE, iter->first.c_str(), bEnableAntennaProfile ? "on" : "off");
               }
              else if(iter->first == "pathlossmode")
               {
                 const std::string str = iter->second.item_.getValue();

                 // location 2 ray flat earth
                 if(str == "2ray")
                  {
                    pathLossMode = UniversalPHY::PathLossManager::PATH_LOSS_MODE_TWO_RAY;
                  }
                 // location free space
                 else if(str == "freespace")
                  {
                    pathLossMode = UniversalPHY::PathLossManager::PATH_LOSS_MODE_FREE_SPACE;
                  }
                 // use pre-computed pathloss
                 else if(str == "pathloss")
                  {
                    pathLossMode = UniversalPHY::PathLossManager::PATH_LOSS_MODE_PATH_LOSS;
                  }
                 else
                  {
                    std::stringstream ssDescription;
                    ssDescription << MODULE << ": invalid configuration item " << iter->first << " " << str << std::ends;
                    throw EMANE::StartException(ssDescription.str());
                  }
 
                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                        id_, __func__, MODULE, iter->first.c_str(), str.c_str());
               }
              else if(iter->first == "defaultconnectivitymode")
               {
                 bDefaultConnectivityEnable = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                        id_, __func__, MODULE, iter->first.c_str(), bDefaultConnectivityEnable ? "on" : "off");
               }
              else if(iter->first == "noiseprocessingmode")
               {
                 bNoiseProcessingEnable = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                        id_, __func__, MODULE, iter->first.c_str(), bNoiseProcessingEnable ? "on" : "off");
               }
              else if(iter->first == "systemnoisefigure")
               {
                 // enforce range inclusive
                 fSystemNoiseFiguredB_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(0.0, DBM_MAX);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %3.2f dB",
                                        id_, __func__, MODULE, iter->first.c_str(), fSystemNoiseFiguredB_);
               }
              else if(iter->first == "frequencyofinterest")
               {
                 // enforce non-zero value
                 const ACE_UINT64 value = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT64(1);

                 // try to add value to foi set
                 if(foi_.insert(value).second == true)
                  {
                    pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                           id_, __func__, MODULE, iter->first.c_str(), EMANEUtils::formatFrequency(value).c_str());
                  }
                 else
                  {
                    std::stringstream sstream;
      
                    sstream << "PHYI "
                            << std::setw(3)
                            << std::setfill('0')
                            << id_
                            << " CommonPHY: parameter "
                            << iter->first
                            << " value "
                            << value
                            << " is already present, please remove duplicate value "
                            << std::ends;

                    throw EMANE::StartException(sstream.str());
                  }
               }
              else if(iter->first == "frequencyofinterestfilterenable")
               {
                 bFOIFilterEnable_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                        id_, __func__, MODULE, iter->first.c_str(), bFOIFilterEnable_ ? "on" : "off");
               }
              else if(iter->first == "txpower")
               {
                 // enforce range inclusive
                 const float value = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toFloat(DBM_MIN, DBM_MAX);
   
                 // setup default tx control
                 defaultTxControl_.setTxPowerdBm(value);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %3.2f dBm",
                                        id_, __func__, MODULE, iter->first.c_str(), value);
               }
              else if(iter->first == "frequency")
               {
                 // enforce non-zero value
                 u64FrequencyHz_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT64(1);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %s",
                                        id_, __func__, MODULE, iter->first.c_str(), EMANEUtils::formatFrequency(u64FrequencyHz_).c_str());
               }
              else if(iter->first == "subid")
               {
                 // enforce non-zero value
                 u16SubId_ = EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT16(1);

                 pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s: %s = %hu",
                                        id_, __func__, MODULE, iter->first.c_str(), u16SubId_);
               }
            }
           else if(iter->second.bRequired_)
            {
              pPlatformService_->log(EMANE::ERROR_LEVEL,"PHYI %03hu %s::%s: missing %s", id_, __func__, MODULE, iter->first.c_str());
              
              std::stringstream ssDescription;
              ssDescription << MODULE << ": Missing configuration item " << iter->first << std::ends;
              throw EMANE::StartException(ssDescription.str());
            }
        }
    }
    catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      
      sstream << "PHYI "
              << std::setw(3)
              << std::setfill('0')
              << id_
              << " CommonPHY: parameter "
              << iter->first
              << ": "
              << exp.what()
              << std::ends;

      throw EMANE::StartException(sstream.str());
    }

   try {
     // configure the antenna profile manager
     antennaProfileManager_.configure(bEnableAntennaProfile, sAntennaProfileManifestURI, u16ProfileId);

     // set our local tx frequency info
     defaultTxControl_.setFrequencyInfo(EMANE::PHYTxFrequencyInfoItems(1,                      // 1 entry
                                          EMANE::PHYTxFrequencyInfo (u64FrequencyHz_,          // freq
                                                                     ACE_Time_Value::zero,     // offset
                                                                     ACE_Time_Value::zero)));  // duration

     // set pathloss mode
     pathLossManager_.setPathLossMode(pathLossMode);

     // set default connectivity
     pathLossManager_.setDefaultConnectivity(bDefaultConnectivityEnable);

     // noise processing enabled
     if(bNoiseProcessingEnable == true)
      {
        // set frequency(s) of interest and our bandwidth value
        noiseManager_.setFrequencies(foi_, defaultTxControl_.getBandWidthHz());
      }

     // if foi filtering is enabled then we require at least one foi
     if((bFOIFilterEnable_ == true) && (foi_.empty() == true))
      {
         std::stringstream sstream;
      
         sstream << "PHYI "
                 << std::setw(3)
                 << std::setfill('0')
                 << id_
                 << " CommonPHY:  "
                 << "frequencyofinterestfilterenable requires at lease one frequencyofinterest value"
                 << std::ends;

         throw EMANE::StartException(sstream.str());
       }

     // set the receiver sensitivity value in dBm
     fReceiverSensitivitydBm_ = THERMAL_NOISE_DB + fSystemNoiseFiguredB_ + (10.0 * log10(defaultTxControl_.getBandWidthHz()));

     // set the receiver sensitivity value in milliwatts
     dReceiverSensitivityMilliWatt_ = DB_TO_MILLIWATT(fReceiverSensitivitydBm_);
   }
   catch(EMANE::EMANEException & exp)
    {
      std::stringstream sstream;
      
      sstream << "PHYI "
              << id_
              << ": "
              << exp.what()
              << std::ends;

      throw EMANE::StartException(sstream.str());
    }
}



void 
UniversalPHY::UniversalPHYLayer::stop()
  throw(EMANE::StopException)
{
  foi_.clear();

  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);
}



void 
UniversalPHY::UniversalPHYLayer::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);
}



void 
UniversalPHY::UniversalPHYLayer::processEvent(const EMANE::EventId & rEventId, const EMANE::EventObjectState & rState)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);
#endif

  switch(rEventId)
   {
    case AntennaProfileEvent::EVENT_ID:
      {
        // update antenna profile 
        if(antennaProfileManager_.getAntennaMode() == EMANE::ANTENNA_MODE_PROFILE)
         {
           const AntennaProfileEvent event(rState);

           AntennaProfileEvent::AntennaProfileEntry entry;

           // find our entry
           if(event.findEntry(id_, entry) == true)
            {
               updateAntennaProfile(entry.u16ProfileId_,        // profile id
                                    entry.fAzimuthDegrees_,     // az
                                    entry.fElevationDegrees_);  // el
            }
           else
            {
#ifdef VERBOSE_LOGGING
              pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s, could not find antenna info for this nem, ignore", 
                                     id_, __func__, MODULE);
#endif
            }
         }
        else
         {
#ifdef VERBOSE_LOGGING
           pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s, not in antenna profile mode, ignore", 
                                  id_, __func__, MODULE);
#endif
         }
      }
    break;

    default:
      // we also handle pathloss and location events so pass to pathloss manager
      pathLossManager_.handleEvent(rEventId, rState);
   }
}


void 
UniversalPHY::UniversalPHYLayer::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{
  // no timed events are handled (yet)

#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);
#endif
}


void 
UniversalPHY::UniversalPHYLayer::processDownstreamControl(const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s", id_, __func__, MODULE);
#endif

  // antenna control message
  if ((msg.getMajorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_MAJOR_ID) &&
      (msg.getMinorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_ANTENNA_PROFILE_MINOR_ID))
    {
      // update antenna profile 
      if(antennaProfileManager_.getAntennaMode() == EMANE::ANTENNA_MODE_PROFILE)
       {
         // control message
         EMANE::UniversalPhyControlAntennaProfileMessage ctrl(msg.get(), msg.length());

         updateAntennaProfile(ctrl.getAntennaProfileId(),          // profile id
                              ctrl.getAntennaAzimuthDegrees(),     // az
                              ctrl.getAntennaElevationDegrees());  // el
       }
      else
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s, not in antenna profile mode, ignore", 
                                id_, __func__, MODULE);
#endif
       }
    }
}


void 
UniversalPHY::UniversalPHYLayer::processDownstreamPacket(EMANE::DownstreamPacket & pkt, const EMANE::ControlMessage &msg)
{
   // current time
   const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();

   // bump statistic
   ++numDownstreamPacketFromMAC_;

   // tx control message for this pkt using our default values
   EMANE::UniversalPhyControlSendMessage pktControl = defaultTxControl_;

   // we expect a UniversalPhyControlSendMessage with each packet
   // so overwrite any items that may have been set by the caller for this pkt 
   if((msg.getMajorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_MAJOR_ID) &&
      (msg.getMinorIdentifier() == EMANE::EMANE_UNIVERSALPHYCONTROL_SEND_MINOR_ID))
     {
       pktControl.overWrite(EMANE::UniversalPhyControlSendMessage(msg));
     }

   // create the common phy header
   EMANE::CommonPHYHeader phyHeader (u16RegistrationId_,                       // our phy registration id
                                     pktControl.getTxPowerdBm(),               // our tx power dBm
                                     pktControl.getBandWidthHz(),              // our bandwidth Hz
                                     antennaProfileManager_.getFixedGaindBi(), // our fixed antenna gain dBi
                                     antennaProfileManager_.getAntennaMode(),  // our antenna mode
                                     tvCurrentTime,                            // our tx time
                                     u16TxSequenceNumber_++);                  // our sequence number

   // set the phy header frequency/offset/duration parameters
   setPHYFrequencyInfo(phyHeader, pktControl.getFrequencyInfo());

   // set the phy header additional transmitter(s)
   setPHYAdditionalTransmitters(phyHeader, pktControl.getAdditionalTransmitters());

   // if antenna profile mode is enabled
   if(antennaProfileManager_.getAntennaMode() == EMANE::ANTENNA_MODE_PROFILE)
    {
      // set the phy header antenna profile info
      setPHYAntennaProfile(phyHeader, pktControl);
    }

#ifdef VERBOSE_LOGGING
   pPlatformService_->log(EMANE::DEBUG_LEVEL,"PHYI %03hu %s::%s, %s", 
                          id_, __func__, MODULE, phyHeader.format().c_str());
#endif
 
   // prepend the universal phy header to the packet body
   prependUniversalPhyHeader(pkt, u16SubId_);

   // the additional transmitter(s) for the ota manager ctrl msg
   EMANE::NEMIdSet additionalTransmitters;

   // get the additional transmitters from the pkt ctrl info
   getAdditionTransmittersSet(pktControl.getAdditionalTransmitters(), additionalTransmitters);

   // create an additional transmitter list control message
   EMANE::OTATransmittersMessage ctrl(additionalTransmitters);

   // send header, packet and control
   sendDownstreamPacket(phyHeader, pkt, ctrl.buildControlMessage());

   // bump statistic
   ++numDownstreamPacketToOTA_;
}



void 
UniversalPHY::UniversalPHYLayer::processUpstreamPacket(const EMANE::CommonPHYHeader & phyHeader,
                                                       EMANE::UpstreamPacket & pkt,
                                                       const EMANE::ControlMessage &)
{
  // current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday();
 
  // bump statistic
  ++numUpstreamPacketFromOTA_;

   // universal phy header
  UniversalPHY::UniversalPhyHeader universalPhyHeader;
 
  // get universal phy header and strip from pkt 
  getUniversalPhyHeader(pkt, universalPhyHeader);

  // get the pkt info
  const EMANE::PacketInfo pinfo = pkt.getPacketInfo();

  // info about all transmitters
  EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems allTransmitters;

  // get all transmitter info
  getAllTransmitters(phyHeader, pinfo, allTransmitters);

  // remote antenna profile
  UniversalPHY::AntennaProfile remoteAntennaProfile;

  // get remote antenna profile
  getAntennaProfile(phyHeader, remoteAntennaProfile);

  // transmitter(s) that are determined to be within range
  EMANE::UniversalPhyControlRecvMessage::TransmitterInfoItems validTransmitters;

#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: from %hu, to %hu, %s",
                         id_, __func__, MODULE,
                         pinfo.source_,
                         pinfo.destination_,
                         phyHeader.format().c_str());
#endif

  // for each transmitting NEM 
  for(EMANE::UniversalPhyControlSendMessage::TransmitterInfoItemsConstIter iter = allTransmitters.begin(); 
      iter != allTransmitters.end(); ++iter)
   {
#ifdef VERBOSE_LOGGING
     pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: handle transmitter %hu", 
                            id_, __func__, MODULE, iter->getSrc());
#endif

     // default rx power
     float fRxPowerdBm = 0.0;

     // default propagation delay
     ACE_Time_Value tvPropagationDelay = ACE_Time_Value::zero;

     // if default connectivity is NOT in effect
     if(pathLossManager_.getDefaultConnectivity() == false)
      {
        // get pathloss/position info about this NEM
        const UniversalPHY::PathLossManager::PathLoss pathLoss = 
              pathLossManager_.getPathLoss(iter->getSrc(), phyHeader.getFirstFrequencyHz());

        // check pathloss status which means we have sufficient information for the active path loss mode 
        if(pathLoss.isValid() == true)
         {
           // get total antenna gain 
           const float fTotalAntennaGaindBi = 
                  antennaProfileManager_.getGaindBi(phyHeader,                                      // phy header
                                                    remoteAntennaProfile,                           // remote antenna profile
                                                    pathLoss.getLocalPosition(),                    // our position
                                                    pathLoss.getRemotePosition(),                   // their position
                                                    pathLoss.getLocalVelocity(),                    // our velocity
                                                    pathLoss.getRemoteVelocity(),                   // their velocity
                                                    defaultTxControl_.getAntennaAzimuthDegrees(),   // our antenna az
                                                    defaultTxControl_.getAntennaElevationDegrees(), // our antenna el 
                                                    pathLoss.getDistanceMeters() >= 0);             // position info available

           // set rx power in dBm (tx power + total antenna gain - pathloss)
           fRxPowerdBm = iter->getTxPowerdBm() + fTotalAntennaGaindBi - pathLoss.getPathLoss();

           // get the propagation delay based on actual positions
           // note antenna placement is not a part of this calculation (yet)
           tvPropagationDelay = pathLoss.getPropagationDelay();
         }
        // no pathloss status
        else
         {
#ifdef VERBOSE_LOGGING
           pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: from %hu, to %hu, no pathloss information, drop", 
                                  id_, __func__, MODULE,
                                  iter->getSrc(),
                                  pinfo.destination_);
#endif

           // bump statistics
           ++numUpstreamDiscardDueToNoPathLossInfo_;

           // done with this transmitter
           continue;
         }
      }
     else
      {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: from %hu, to %hu, default connectivity is enabled", 
                                id_, __func__, MODULE,
                                iter->getSrc(),
                                pinfo.destination_);
#endif
      }

     // check rx power against receiver sensitivity
     if(fRxPowerdBm >= fReceiverSensitivitydBm_)
      {
        // check phy modulation paramters based on frequency and subid
        if(checkModulation(phyHeader.getFrequencyInfo(), universalPhyHeader.getSubId()) == false)
         {
           // message is out of band, add noise level in milliwatts 
           noiseManager_.addNoiseMilliWatt(DB_TO_MILLIWATT(fRxPowerdBm),        // rx power mW
                                           phyHeader.getBandWidthHz(),          // band width Hz
                                           tvCurrentTime + tvPropagationDelay,  // current time + propagation delay
                                           phyHeader.getFrequencyInfo());       // freq info list

           // done with this transmitter
           continue;
          }
         else
          {
#ifdef VERBOSE_LOGGING
             pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: from %hu, to %hu, rxpwr %3.2f dBm "
                                    "is within receiver sensitivity %3.2f dBm, pass", 
                                    id_, __func__, MODULE,
                                    iter->getSrc(),
                                    pinfo.destination_,
                                    fRxPowerdBm,
                                    fReceiverSensitivitydBm_);
#endif

            // add as a valid transmitter
            validTransmitters.push_back(
                  EMANE::UniversalPhyControlRecvMessage::TransmitterInfo(iter->getSrc(),       // src
                                                                         fRxPowerdBm,          // rx power in dBm
                                                                         tvPropagationDelay)); // propagation delay
          }
       }
      // out of range
      else
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: from %hu, to %hu, rxpwr %3.2f dBm "
                                 "< receiver sensitivity %3.2f dBm, drop", 
                                id_, __func__, MODULE,
                                iter->getSrc(),
                                pinfo.destination_,
                                fRxPowerdBm,
                                fReceiverSensitivitydBm_);
#endif

          // bump statistics
          ++numUpstreamDiscardDueToReceiverSensitivity_;

          // done with this transmitter
          continue;
       }
   } // end each transmitter


  // if we have any transmitters in range
  if(validTransmitters.empty() == false)
   {
      // create universal phy control recv message
      EMANE::UniversalPhyControlRecvMessage rxControlMessage;

      // set the bandwidth in Hz
      rxControlMessage.setBandWidthHz(phyHeader.getBandWidthHz());

      // set the transmit time
      rxControlMessage.setTxTime(phyHeader.getTxTime());

      // set the frequency info including noise level
      rxControlMessage.setFrequencyInfo(noiseManager_.getRxFrequencyInfo(phyHeader.getBandWidthHz(),
                                                                         tvCurrentTime,
                                                                         phyHeader.getFrequencyInfo(),
                                                                         dReceiverSensitivityMilliWatt_));

      // set the transmitter info (all valid transmitters)
      rxControlMessage.setTransmitterInfo(validTransmitters);

#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: forward pkt upstream, %s",
                              id_, __func__, MODULE, rxControlMessage.format(tvCurrentTime).c_str());
#endif

      // send pkt and ctrl upstream 
      sendUpstreamPacket(pkt, rxControlMessage.buildControlMessage());

      // bump statistics
      ++numUpstreamPacketToMAC_;
   }
}




bool 
UniversalPHY::UniversalPHYLayer::checkModulation(const EMANE::PHYTxFrequencyInfoItems & items, ACE_UINT16 u16SubId)
{
   // check sub id is non zero
   if(u16SubId == 0)
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: sub id must be non-zero",
                              id_, __func__, MODULE);
#endif

       // bump statistics
       ++numUpstreamDiscardDueToInvalidSubId_;

       // fail
       return false;
    }

   // check sub id
   if(u16SubId != u16SubId_)
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: sub id %hu != expected value %hu",
                              id_, __func__, MODULE,
                              u16SubId,
                              u16SubId_);
#endif
       // bump statistics
       ++numUpstreamDiscardDueToSubIdMismatch_;

       // fail
       return false;
    }

   // if FOI filtering is enabled
   if(bFOIFilterEnable_ == true)
    {
      // for each freq info entry
      for(EMANE::PHYTxFrequencyInfoItemsConstIter iter = items.begin(); iter != items.end(); ++iter)
       {
         // check center frequency is in the FOI set
         if(foi_.count(iter->getCenterFrequencyHz()) == 0)
          {
#ifdef VERBOSE_LOGGING
             pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: center frequency %s not in FOI set of %zd entries",
                                    id_, __func__, MODULE,
                                    EMANEUtils::formatFrequency(iter->getCenterFrequencyHz()).c_str(),
                                    foi_.size());
#endif

             // bump statistics
             ++numUpstreamDiscardDueToFoi_;

            // fail
            return false;
          }
       }
    }

    // pass
    return true;
}



void 
UniversalPHY::UniversalPHYLayer::prependUniversalPhyHeader(EMANE::DownstreamPacket & pkt, ACE_UINT16 u16SubId) const
{
   // universal phy header
   UniversalPHY::UniversalPhyHeader hdr(u16SubId);

   // convert to network byte order
   hdr.toNetworkByteOrder();

   // prepend header
   pkt.prepend(&hdr, sizeof(hdr));
}


void
UniversalPHY::UniversalPHYLayer::getUniversalPhyHeader(EMANE::UpstreamPacket & pkt, UniversalPHY::UniversalPhyHeader & hdr) const
{
  // copy header
  ACE_OS::memcpy(&hdr, pkt.get(), sizeof(hdr));

  // convert to host byte order
  hdr.toHostByteOrder();

  // strip header
  pkt.strip(sizeof(hdr));
}



bool
UniversalPHY::UniversalPHYLayer::checkCommonPHYHeader(EMANE::CommonPHYHeader & phyHeader)
{
  // check registration id
  if(phyHeader.getRegistrationId() != u16RegistrationId_)
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL,
                             "PHYI %03hu %s::%s: invalid phy registration id %hu, expected value %hu",
                             id_, __func__, MODULE,
                             phyHeader.getRegistrationId(),
                             u16RegistrationId_);

      // bump statistics
      ++numUpstreamDiscardDueToRegistrationId_;

      // fail
      return false;
    }
  else
    {
      // pass
      return true;
    }
}


void
UniversalPHY::UniversalPHYLayer::getAllTransmitters(const EMANE::CommonPHYHeader & phyHeader, 
                       const EMANE::PacketInfo & pinfo, EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & items) const
{
  const EMANE::CommonPHYHeaderOptionObjectStateList optionList = phyHeader.getOptionList();

  // for each option
  for(EMANE::CommonPHYHeaderOptionObjectStateList::const_iterator iter = optionList.begin(); iter != optionList.end(); ++iter)
   {
     try
      {
        // additional transmitter info
        if(iter->getType() == EMANE::COMMON_PHY_HEADER_OPTION_TRANSMITTERS)
         {
           EMANE::CommonPHYHeaderTransmitterOption option(*iter);

           // reserve room for all transmitters plus the pkt originator
           items.reserve(option.getTransmitterInfo().size() + 1);

           // for each additional transmitter
           for(EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfoItemsConstIter opt = option.getTransmitterInfo().begin();
               opt != option.getTransmitterInfo().end(); ++opt)
            {
              // add additional transmitters
              items.push_back(EMANE::UniversalPhyControlSendMessage::TransmitterInfo(opt->getSrc(),          // src
                                                                                     opt->getTxPowerdBm())); // tx power
            }

#ifdef VERBOSE_LOGGING
           pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: phy header option: %s",
                                    id_, __func__, MODULE, option.format().c_str());
#endif

           break;
         }
       }
     catch(EMANE::CommonPHYHeaderException & exp)
      {
        pPlatformService_->log(EMANE::ERROR_LEVEL,"PHYI %03hu %s::%s: %s", id_, __func__, MODULE, exp.what());
      }
   }

  // add the pkt originator
  items.push_back(EMANE::UniversalPhyControlSendMessage::TransmitterInfo(pinfo.source_, phyHeader.getTxPowerdBm()));
}



void
UniversalPHY::UniversalPHYLayer::getAdditionTransmittersSet(const EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & vec, 
                                                            EMANE::NEMIdSet & set) const
{
  for(EMANE::UniversalPhyControlSendMessage::TransmitterInfoItemsConstIter iter = vec.begin(); iter != vec.end(); ++iter)
   {
     // add to the additional transmitters 
     set.insert(iter->getSrc());
   }
}


   
void 
UniversalPHY::UniversalPHYLayer::updateAntennaProfile(const ACE_UINT16 u16ProfileId, 
                                                      const float fAzimuthDegrees, 
                                                      const float fElevationDegrees)
{
  // update antenna profile manager
  antennaProfileManager_.handleAntennaProfileChange(u16ProfileId);

  // update local/default tx control
  defaultTxControl_.setAntennaProfileId(u16ProfileId);

  defaultTxControl_.setAntennaElevationDegrees(fElevationDegrees);

  defaultTxControl_.setAntennaAzimuthDegrees(fAzimuthDegrees);
}



void
UniversalPHY::UniversalPHYLayer::getAntennaProfile(const EMANE::CommonPHYHeader & phyHeader, 
                                                   UniversalPHY::AntennaProfile & antennaProfile) const
{
  if(phyHeader.getAntennaMode() == EMANE::ANTENNA_MODE_PROFILE)
   {
     // get phy header option list
     const EMANE::CommonPHYHeaderOptionObjectStateList optionList = phyHeader.getOptionList();

     // for each option
     for(EMANE::CommonPHYHeaderOptionObjectStateList::const_iterator iter = optionList.begin(); iter != optionList.end(); ++iter)
      {
        try
         {
          // check antenna option type
          if(iter->getType() == EMANE::COMMON_PHY_HEADER_OPTION_ANTENNA)
            {
              // cerate option from object state
              EMANE::CommonPHYHeaderAntennaOption option(*iter);

              // set the remote antenna profile info
              antennaProfile = UniversalPHY::AntennaProfile (option.getAntennaProfileId(),         // profile id
                                                             option.getAntennaAzimuthDegrees(),    // antenna azimuth
                                                             option.getAntennaElevationDegrees()); // antenna elevation

#ifdef VERBOSE_LOGGING
              pPlatformService_->log(EMANE::DEBUG_LEVEL, "PHYI %03hu %s::%s: phy header option: %s",
                                     id_, __func__, MODULE, option.format().c_str());
#endif
              break;
            }
         }
        catch(EMANE::CommonPHYHeaderException & exp)
         {
           pPlatformService_->log(EMANE::ERROR_LEVEL,"PHYI %03hu %s::%s: %s", id_, __func__, MODULE, exp.what());
         }
      }
   }
}



void UniversalPHY::UniversalPHYLayer::setPHYAntennaProfile(EMANE::CommonPHYHeader & phyHeader, 
                                                         const EMANE::UniversalPhyControlSendMessage & ctrl)
{
   // add the profile info
   phyHeader.addOption(EMANE::CommonPHYHeaderAntennaOption(ctrl.getAntennaProfileId(),          // antenna profile id
                                                           ctrl.getAntennaAzimuthDegrees(),     // antenna azimuth
                                                           ctrl.getAntennaElevationDegrees())); // antenna elevation

   // update our local settings
   updateAntennaProfile(ctrl.getAntennaProfileId(),         // antenna profile id
                        ctrl.getAntennaAzimuthDegrees(),    // antenna azimuth
                        ctrl.getAntennaElevationDegrees()); // antenna elevation
}


void UniversalPHY::UniversalPHYLayer::setPHYAdditionalTransmitters(EMANE::CommonPHYHeader & phyHeader,
          const EMANE::UniversalPhyControlSendMessage::TransmitterInfoItems & vec) const
{
  if(vec.empty() == false)
   {
     EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfoItems opt;

     for(EMANE::UniversalPhyControlSendMessage::TransmitterInfoItemsConstIter iter = vec.begin(); iter != vec.end(); ++iter)
      {
        // add to the additional transmitters
        opt.push_back(EMANE::CommonPHYHeaderTransmitterOption::TransmitterInfo(iter->getSrc(), iter->getTxPowerdBm()));
      }

     phyHeader.addOption(EMANE::CommonPHYHeaderTransmitterOption(opt));
   }
}


void UniversalPHY::UniversalPHYLayer::setPHYFrequencyInfo(EMANE::CommonPHYHeader & phyHeader, 
      const EMANE::PHYTxFrequencyInfoItems & vec) const 
          throw(EMANE::CommonPHYHeaderException)
{
   // must supply at least one freq info entry
   if(vec.empty())
    {
      std::stringstream ss;

      ss << "Must supply at least one frequency info entry" << std::ends;

      throw(EMANE::CommonPHYHeaderException(ss.str()));
    }

   // get a copy of the entries since we may change them if needed
   EMANE::PHYTxFrequencyInfoItems info = vec;

   // check freq values
   for(EMANE::PHYTxFrequencyInfoItemsIter iter = info.begin(); iter != info.end(); ++iter)
    {
      // use our default value if not set by caller
      if((iter->getCenterFrequencyHz() == 0))
       {
         iter->setCenterFrequencyHz(u64FrequencyHz_);
       }

      // check bandwidth limit
      if((iter->getCenterFrequencyHz() / 2.0) < defaultTxControl_.getBandWidthHz())
        {
          std::stringstream ss;
          ss << "Center frequency "
          << EMANEUtils::formatFrequency(iter->getCenterFrequencyHz()).c_str()
          << " must be at least 1/2 of the bandwidth " 
          << EMANEUtils::formatFrequency(defaultTxControl_.getBandWidthHz()).c_str()
          << std::ends;

          throw(EMANE::CommonPHYHeaderException(ss.str()));
        }
     }

   // now set the frequency info
   phyHeader.setFrequencyInfo(info);
}


void
UniversalPHY::UniversalPHYLayer::registerAllStatistics()
{
  std::string name;

  // If the name is NOT already registered then register the statistic name and save it.
  // Later we can unregister ALL the statistics on termination and prevent
  // the platfrom service from holding any "dangling" statistic references when we go away.

  name = "numUpstreamDiscardDueToReceiverSensitivity";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamDiscardDueToReceiverSensitivity_);

     registeredStatistics_.insert(name);
   }
   
  name = "numUpstreamDiscardDueToRegistrationId";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamDiscardDueToRegistrationId_);

     registeredStatistics_.insert(name);
   }

  name = "numUpstreamDiscardDueToInvalidSubId";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamDiscardDueToInvalidSubId_);

     registeredStatistics_.insert(name);
   }

  name = "numUpstreamDiscardDueToSubIdMismatch";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamDiscardDueToSubIdMismatch_);

     registeredStatistics_.insert(name);
   }

  name = "numUpstreamDiscardDueToFoi";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamDiscardDueToFoi_);

     registeredStatistics_.insert(name);
   }

  name = "numUpstreamDiscardDueToNoPathLossInfo";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamDiscardDueToNoPathLossInfo_);

     registeredStatistics_.insert(name);
   }

  name = "numUpstreamPacketFromOTA";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamPacketFromOTA_);

     registeredStatistics_.insert(name);
   }

  name = "numDownstreamPacketToOTA";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numDownstreamPacketToOTA_);

     registeredStatistics_.insert(name);
   }

  name = "numUpstreamPacketToMAC";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numUpstreamPacketToMAC_);

     registeredStatistics_.insert(name);
   }

  name = "numDownstreamPacketFromMAC";
  if(registeredStatistics_.count(name) == 0)
   {
     pPlatformService_->registerStatistic(name,  &numDownstreamPacketFromMAC_);

     registeredStatistics_.insert(name);
   }
}


void
UniversalPHY::UniversalPHYLayer::unregisterAllStatistics()
{
   // now unregister all the statistics that we registered earlier
   for(StringSetIter iter = registeredStatistics_.begin(); iter != registeredStatistics_.end(); ++iter)
    {
       pPlatformService_->unregisterStatistic(*iter);
    }
  
   // clear the list 
   registeredStatistics_.clear();
}


DECLARE_PHY_LAYER(UniversalPHY::UniversalPHYLayer);
