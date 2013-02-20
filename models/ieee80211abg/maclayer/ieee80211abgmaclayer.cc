/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "ieee80211abgmaclayer.h"
#include "macstatistics.h"
#include "idletxstate.h"
#include "utils.h"
#include "onehopnbrlistevent.h"

#include "emane/emanecontrolmessage.h"
#include "emane/emaneevent.h"
#include "emane/emaneconfigurationrequirement.h"


#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/parameterconvert.h"

#include <sstream>

#include <ace/Basic_Types.h>

#include <math.h>


namespace
{
  EMANE::ConfigurationDefinition defs[] =
  {
    // reqd,  default, count, name,                             value,                type, description
    { false,  true,    1,     "mode",                           "0",                  0,    "mode index"} ,
    { false,  true,    1,     "enablepromiscuousmode",          "off",                0,    "enable promiscuous mode"} ,
    { false,  true,    1,     "distance",                       "1000",               0,    "max distance"} ,
    { false,  true,    1,     "unicastrate",                    "4",                  0,    "unicast data rate index"} ,
    { false,  true,    1,     "multicastrate",                  "1",                  0,    "multicast data rate index"} ,
    { false,  true,    1,     "rtsthreshold",                   "0",                  0,    "rts/cts threshold in bytes"} ,
    { false,  true,    1,     "wmmenable",                      "off",                0,    "wireless multimedia"} ,
    { false,  true,    1,     "pcrcurveuri",                    "ieee80211pcr.xml",   0,    "pcr curve uri"} ,

     // flow control
    { false,  true,    1,     "flowcontrolenable",              "off",                0,    "flow control enable"} ,
    { false,  true,    1,     "flowcontroltokens",              "10",                 0,    "number of flow control tokens"} ,

      // the following parameters with multiple values (0:x to 3:x) apply to the 4 types of traffic classes 
    { false,  true,    1,     "queuesize",           "0:255 1:255 2:255 3:255",       0,    "queue size"} ,
    { false,  true,    1,     "cwmin",               "0:32 1:32 2:16 3:8",            0,    "minimum contention window size"} ,
    { false,  true,    1,     "cwmax",               "0:1024 1:1024 2:64 3:16",       0,    "maximum contention window size"} ,
    { false,  true,    1,     "aifs",                "0:2 1:2 2:2 3:1",               0,    "arbitration inter frame spacing time"} ,
    { false,  true,    1,     "txop",                "0:0 1:0 2:0 3:0",               0,    "transmit oppurtunity time"} ,
    { false,  true,    1,     "retrylimit",          "0:2 1:2 2:2 3:2",               0,    "retry limit"} ,

      // undocumented config params
    { false,  true,    1,     "channelactivityestimationtimer", "100",                0,    "channel activity estimation timer in msec"} ,
    { false,  true,    1,     "neighbortimeout",                "5",                  0,    "neighbor timeout in sec"} ,

      // end of list
    { 0, 0, 0, 0, 0, 0, 0} };

  const char *pzLayerName = "IEEE80211abgMACLayer";

  const ACE_UINT32 CHANNEL_USAGE_CALLBACK   = 1;

  const ACE_UINT32 UPSTREAM_PACKET_CALLBACK = 2;


  // undocumented test params
  // tx retry logic enable
  const bool ENABLE_TX_RETRY = false;

  // wmm overhead enable
  const bool ENABLE_WMM_OVERHEAD = false;
}



IEEE80211ABG::IEEE80211abgMACLayer::IEEE80211abgMACLayer (EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService):
  MACLayerImplementor (id, pPlatformService),
  id_ (id),
  macStatistics_ (id, pPlatformService),
  downstreamQueue_ (id, pPlatformService),
  downstreamThread_(ACE_OS::NULL_thread),
  running_ (false),
  cond_ (mutex_),
  pTxState_ (IdleTxStateSingleton::instance ()),
  pcrManager_ (id_, pPlatformService_),
  neighborManager_ (id_, pPlatformService_, this), 
  flowControlManager_ (*this), 
  seqNumber_ (0),
  modeTiming_ (macConfig_),
  tvLastRxEndTime_(ACE_Time_Value::zero)
{
  // load configuration requirements
  configRequirements_ = EMANE::loadConfigurationRequirements (defs);
}


IEEE80211ABG::IEEE80211abgMACLayer::~IEEE80211abgMACLayer ()
{ }



void
IEEE80211ABG::IEEE80211abgMACLayer::initialize ()
throw (EMANE::InitializeException)
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s", id_, pzLayerName, __func__);
}



void
IEEE80211ABG::IEEE80211abgMACLayer::configure (const EMANE::ConfigurationItems & items)
throw (EMANE::ConfigureException)
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s", id_, pzLayerName, __func__);

  // load configuration items
  Component::configure (items);
}



void
IEEE80211ABG::IEEE80211abgMACLayer::start ()
throw (EMANE::StartException)
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s", id_, pzLayerName, __func__);

  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin ();

  try
  {
    // for each required config item
    for (; iter != configRequirements_.end (); ++iter)
      {
        // item not present
        if (!iter->second.bPresent_)
          {
            pPlatformService_->log (EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: missing %s", 
                 id_, pzLayerName, __func__, iter->first.c_str ());

            std::stringstream ssDescription;
            ssDescription << "Missing configuration item " << iter->first << std::ends;
            throw EMANE::StartException (ssDescription.str ());
          }

        // set config item value pair
        const bool bResult = macConfig_.set (iter->first, iter->second.item_.getValue ());

        // check result
        if (bResult == false)
          {
            // failure
            pPlatformService_->log (EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: config item error %s = %s", 
                 id_, pzLayerName, __func__, 
                 iter->first.c_str (), 
                 iter->second.item_.getValue ().c_str ());

            throw EMANE::StartException ("Invalid configuration item");
          }
        else
          {
            // success
            pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: config item %s = %s", 
                 id_, pzLayerName, __func__, 
                 iter->first.c_str (), 
                 iter->second.item_.getValue ().c_str ());
          }
      }
  }
  catch (EMANEUtils::ParameterConvert::ConversionException & exp)
  {
    std::stringstream sstream;
    sstream << "IEEE80211abgMacLayer:  Paramater " << iter->first << ": " << exp.what () << std::ends;
    throw EMANE::StartException (sstream.str ());
  }

  // error string
  std::string err;

  // verify config item(s)
  if (macConfig_.verify (err) == false)
    {
      std::stringstream sstream;
      sstream << "IEEE80211abgMacLayer:  Invalid Configuration " << err << std::ends;
      throw EMANE::StartException (sstream.str ());
    }

  // number of access categories (queues)
  const size_t categories = macConfig_.getNumAccessCategories ();

  // set number of categories
  downstreamQueue_.categories (categories);

  // for each category
  for (size_t idx = 0; idx < categories; ++idx)
    {
      // set maxsize for each category 
      downstreamQueue_.maxsize (macConfig_.getQueueSize (idx), idx);
    }

  // load pcr curve
  pcrManager_.load (macConfig_.getPcrUri ());

  // set neighbor timeout
  neighborManager_.setNeighborTimeout (macConfig_.getNeighborTimeout());

  // active state
  running_ = true;

  // create thread for downstream queue processing
  EMANEUtils::spawn (*this, &IEEE80211abgMACLayer::processDownstreamQueue, &downstreamThread_);
}



void
IEEE80211ABG::IEEE80211abgMACLayer::postStart ()
{
  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday ();

  // flow control enabled   
  if (macConfig_.getFlowControlEnable ())
    {
      // start flow control
      flowControlManager_.start (macConfig_.getFlowControlTokens ());
    }

   // schedule the event id, no data, initial timeout (absolute time), replay interval
   const long eventId1 = pPlatformService_->scheduleTimedEvent(CHANNEL_USAGE_CALLBACK,
                                          NULL, 
                                          macConfig_.getChannelActivityTimerInterval() + tvCurrentTime, 
                                          macConfig_.getChannelActivityTimerInterval());

   pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: added channel activity timed eventId %ld", id_, pzLayerName, __func__, eventId1);
}



void
IEEE80211ABG::IEEE80211abgMACLayer::stop ()
throw (EMANE::StopException)
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s", id_, pzLayerName, __func__);

  // flow control enabled   
  if (macConfig_.getFlowControlEnable ())
    {
      // stop flow control
      flowControlManager_.stop ();
    }

  // no longer active
  running_ = false;

  // cancel downstream queue
  downstreamQueue_.cancel ();

  // join dwonstream queue thread
  ACE_OS::thr_join (downstreamThread_, 0, 0);

  downstreamThread_ = ACE_OS::NULL_thread;
}



void
IEEE80211ABG::IEEE80211abgMACLayer::destroy ()
throw ()
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s", id_, pzLayerName, __func__);
}



void
IEEE80211ABG::IEEE80211abgMACLayer::processUpstreamControl (const EMANE::ControlMessage &)
{
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s", id_, pzLayerName, __func__);
}



void
IEEE80211ABG::IEEE80211abgMACLayer::processDownstreamControl (const EMANE::ControlMessage & ctrl)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: 0x%X:0x%X",
       id_, pzLayerName, __func__, 
       ctrl.getMajorIdentifier (), 
       ctrl.getMinorIdentifier ());
#endif

  // check for flow control ctrl type
  if (macConfig_.getFlowControlEnable () && ctrl.getMajorIdentifier () == EMANE::EMANE_FLOWCONTROL_MAJOR_ID)
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: processing Flow Control Message", id_, pzLayerName, __func__);
#endif

      flowControlManager_.processFlowControlMessage (ctrl);
    }
}



void
IEEE80211ABG::IEEE80211abgMACLayer::processUpstreamPacket (const EMANE::CommonMACHeader & hdr,
                                                           EMANE::UpstreamPacket & pkt,
                                                           const EMANE::ControlMessage & msg)
{
  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday ();

  // get packet info
  const EMANE::PacketInfo pinfo = pkt.getPacketInfo ();

  // check mac header registration id (ieee80211abg)
  if(hdr.getRegistrationId() != registrationId_)
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, MAC registration id %hu does not match our id %hu, drop.",
                             id_, pzLayerName, __func__,
                             pinfo.source_,
                             pinfo.destination_,
                             hdr.getRegistrationId(),
                             registrationId_);

      // bump counter
      macStatistics_.incrementUpstreamDiscardDueToRegistrationIdMismatch();

      // drop
      return;
    }

  // check for recv control info
  if ((msg.getMajorIdentifier () == EMANE::EMANE_UNIVERSALPHYCONTROL_MAJOR_ID) && 
      (msg.getMinorIdentifier () == EMANE::EMANE_UNIVERSALPHYCONTROL_RECV_MINOR_ID))
    {
      // phy control recv message
      const EMANE::UniversalPhyControlRecvMessage ctrl(msg);

#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, len %zu, %s",
           id_, pzLayerName, __func__,
           pinfo.source_,
           pinfo.destination_, 
           pkt.length (), 
           ctrl.format (tvCurrentTime).c_str ());
#endif

      // handle the upstream packet
      handleUpstreamPacket(pkt, ctrl);
    }
  else
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL,"MACI %03hu %s::%s: universal phy recv control message not provided from src %hu, drop",
          id_, pzLayerName, __func__, pinfo.source_);

      // bump counter
      macStatistics_.incrementUpstreamDiscardDueToLackOfRxCtrlInfo();

      // drop
      return;
    }
}


void 
IEEE80211ABG::IEEE80211abgMACLayer::handleUpstreamPacket (EMANE::UpstreamPacket &pkt, const EMANE::UniversalPhyControlRecvMessage &ctrl)
{
  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday ();

  // mac header
  IEEE80211ABG::MacHeader macHeader;

  // copy header
  ACE_OS::memcpy (&macHeader, pkt.get (), sizeof (macHeader));

  // strip header
  pkt.strip (sizeof (macHeader));

  // convert to host byte order
  macHeader.toHostByteOrder ();

  // lock mutex, we want to complete this section without interruption
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  // get the frequency info for the pkt (a list is returned but we are only interested in the first value)
  const EMANE::PHYRxFrequencyInfoItems freqInfoVec = ctrl.getFrequencyInfo ();

  // get the transmitter info (a list is returned but we are only interested in the first value)
  const EMANE::UniversalPhyControlRecvMessage::TransmitterInfoItems transmitters = ctrl.getTransmitterInfo ();

  const float fRxPowerdBm = transmitters[0].getRxPowerdBm();

  const double fRxPowermW = IEEE80211ABG::DB_TO_MILLIWATT(fRxPowerdBm);

  // unicast cts ctrl
  if(macHeader.getType() == MSG_TYPE_UNICAST_CTS_CTRL)
   {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: cts pkt from %hu, seq %hu",
                              id_, pzLayerName, __func__, 
                              macHeader.getSrcNEM(),
                              macHeader.getSequence ());
#endif
      // update ctrl channel activity
      neighborManager_.updateCtrlChannelActivity (macHeader.getSrcNEM(),    // src
                                                  macHeader.getDstNEM(),    // dst (origin)
                                                  macHeader.getType(),      // msg type
                                                  fRxPowermW,               // rx power
                                                  tvCurrentTime,            // time
                                                  macHeader.getDuration()); // duration 


      // bump counter
      macStatistics_.incrementUpstreamUnicastCtsRxFromPhy ();

      // drop, done with this cts pkt
      return;
   }
  // unicast rts-cts data
  else if((macHeader.getType() == MSG_TYPE_UNICAST_RTS_CTS_DATA))
   {
     // for this nem
     if(macHeader.getDstNEM() == id_)
      {
        // cts pkt info src is us, dst is broadcast, dscp is 0
        const EMANE::PacketInfo ctsinfo(id_, EMANE::NEM_BROADCAST_MAC_ADDRESS, 0);

        // cts pkt
        const EMANE::DownstreamPacket ctsPkt(ctsinfo, NULL, 0);

        // create a queue entry
        DownstreamQueueEntry entry (ctsPkt, tvCurrentTime, dscpToQindex(0));

        // set the duration 
        entry.tvDurationInterval_ = macHeader.getDuration();

        // send cts
        sendDownstreamUnicastCts (entry, macHeader.getSrcNEM());
      }

      // bump counter
      macStatistics_.incrementUpstreamUnicastRtsCtsDataRxFromPhy ();
   }
  else if((macHeader.getType() == MSG_TYPE_UNICAST_DATA))
   {
      // bump counter
      macStatistics_.incrementUpstreamUnicastDataRxFromPhy ();
   }
  else if((macHeader.getType() == MSG_TYPE_BROADCAST_DATA))
   {
      // bump counter
      macStatistics_.incrementUpstreamMulticastDataRxFromPhy ();
   }
  else
   {
      // bump counter
      macStatistics_.incrementUpstreamDiscardDueToUnknownType ();

      // drop
      return;
   }

  // get overhead if enabled
  const ACE_Time_Value tvOverhead = (ENABLE_WMM_OVERHEAD == true) ? 
                                     modeTiming_.getOverheadInterval(dscpToQindex(pkt.getPacketInfo ().dscp_)) :
                                     ACE_Time_Value::zero;

  // update data channel activity
  neighborManager_.updateDataChannelActivity (macHeader.getSrcNEM(),                                // src
                                              macHeader.getType(),                                  // msg type
                                              fRxPowermW,                                           // rx power
                                              tvCurrentTime,                                        // time
                                              macHeader.getDuration() + tvOverhead);                // duration + overhead


  if (ENABLE_TX_RETRY == true)
   {
     // check reception for collision and sinr
     if(checkUpstremReception(pkt, fRxPowerdBm, freqInfoVec[0].getNoiseFloordBm(), macHeader, macHeader.getRetries()) == false)
      {  
#ifdef VERBOSE_LOGGING
         pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, reception failed, drop", 
                                  id_, pzLayerName, __func__,  
                                  macHeader.getSrcNEM(),
                                  macHeader.getDstNEM());
#endif
         // drop
         return;
      }
   }
  else
   {
     // assume failed
     bool bPass = false;

     int tryNum;

     // check until retry limit reached or pkt passes
     for (tryNum = 0; (tryNum <= macHeader.getRetries()) && (bPass == false); ++tryNum)
      {
         // did the pkt pass
         if(checkUpstremReception(pkt, fRxPowerdBm, freqInfoVec[0].getNoiseFloordBm(), macHeader, tryNum) == true)
          {
#ifdef VERBOSE_LOGGING
             pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, reception passed on try %d", 
                                     id_, pzLayerName, __func__, 
                                     macHeader.getSrcNEM(),
                                     macHeader.getDstNEM(),
                                     tryNum + 1);
#endif

             // set passed flag
             bPass = true;
          }
      }

      // check passed flag
      if(bPass == false)
       {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, reception failed in %d tries", 
                                  id_, pzLayerName, __func__,  
                                  macHeader.getSrcNEM(),
                                  macHeader.getDstNEM(),
                                  tryNum);
#endif

          // drop
          return;
       }
   }
   

  // check for duplicate packet based on previous sender
  if (isDuplicate (macHeader.getSrcNEM(), macHeader.getSequence ()) == true)
   {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: duplicate pkt from %hu, seq %hu, drop",
                              id_, pzLayerName, __func__, 
                              macHeader.getSrcNEM(),
                              macHeader.getSequence ());
#endif

      // bump counter
      macHeader.getDstNEM() == EMANE::NEM_BROADCAST_MAC_ADDRESS ?
           macStatistics_.incrementUpstreamMulticastDataDiscardDueToDuplicates () :
           macStatistics_.incrementUpstreamUnicastDataDiscardDueToDuplicates ();

      // drop
      return;
   }

   // check dst
   if(macHeader.getDstNEM() != id_ && macHeader.getDstNEM() != EMANE::NEM_BROADCAST_MAC_ADDRESS)
     {
       if(macConfig_.getPromiscuosEnable() == false)
         {
#ifdef VERBOSE_LOGGING
            pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: nexthop %hu, not this nem, promiscous mode disabled, drop", 
                                    id_, pzLayerName, __func__, macHeader.getDstNEM());

#endif
             // bump counter
             macStatistics_.incrementUpstreamUnicastDataDiscardDueToNotForThisNEM ();

            // drop
            return;
         }
    }

   // timer delay
   ACE_Time_Value tvTimerDelayTime = tvCurrentTime + macHeader.getDuration();

   // previous pkt rx time has not passed
   if(tvLastRxEndTime_ > tvCurrentTime)
     {
       // bump delay time out 
       tvTimerDelayTime += (tvLastRxEndTime_ - tvCurrentTime);
     }
        
   // create a copy of the pkt, the callback will take ownership of the pkt
   const EMANE::UpstreamPacket * p = new EMANE::UpstreamPacket(pkt);

   // schedule the event id, pkt data, timeout (absolute time), one shot
   const long eventId = pPlatformService_->scheduleTimedEvent(UPSTREAM_PACKET_CALLBACK, p, tvTimerDelayTime);

   // timed event was scheduled
   if(eventId >= 0)
     {
      // update last rx end time
      tvLastRxEndTime_ = tvTimerDelayTime;

#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: tx state %s, timed eventId %ld, delay %ld:%06ld, %s", 
                              id_, pzLayerName, __func__, 
                              pTxState_->statename (),
                              eventId,
                              (tvTimerDelayTime - tvCurrentTime).sec(),
                              (tvTimerDelayTime - tvCurrentTime).usec(),
                              macHeader.format ().c_str ());
#endif
    }
   // could not schedule the timed event
   else
    {
      // cleanup
      delete p;

      pPlatformService_->log(EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: failed to add timed event, drop",
                             id_, pzLayerName, __func__);

      // bump counter
      macStatistics_.incrementUpstreamDiscardDueToQueueRejection();

      // drop
      return;
    }
}



bool 
IEEE80211ABG::IEEE80211abgMACLayer::checkUpstremReception (EMANE::UpstreamPacket & pkt, 
                                                          const float fRxPowerdBm,
                                                          const float fNoiseFloordBm,
                                                          const IEEE80211ABG::MacHeader & macHeader,
                                                          int tryNum)
{
   // get the noise floor in milli watts
   const double dNoiseFloorMilliWatts = DB_TO_MILLIWATT(fNoiseFloordBm);

   // get packet info
   const EMANE::PacketInfo pinfo = pkt.getPacketInfo ();

   // check for rx collision, use num retries from pkt
   const COLLISION_TYPE collisionType = checkForRxCollision(macHeader.getSrcNEM(), dscpToQindex(pinfo.dscp_), tryNum);

   // adjustment to noise floor
   double dNoiseFloorAdjustmentMilliWatts = 0.0;

   // clobbered rx during tx
   if(collisionType & COLLISION_TYPE_CLOBBER_RX_DURING_TX)
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: clobber rx during tx, drop",
                               id_, pzLayerName, __func__);
#endif


        // bump counter
        macHeader.getDstNEM() == EMANE::NEM_BROADCAST_MAC_ADDRESS ?
           macStatistics_.incrementUpstreamMulticastDataDiscardDueToClobberRxDuringTx () :
           macStatistics_.incrementUpstreamUnicastDataDiscardDueToClobberRxDuringTx ();

       // drop
       return false;
    }

   // clobbered rx hidden busy
   if(collisionType & COLLISION_TYPE_CLOBBER_RX_HIDDEN_BUSY)
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: clobber rx hidden busy, drop",
                               id_, pzLayerName, __func__);
#endif

       // bump counter
        macHeader.getDstNEM() == EMANE::NEM_BROADCAST_MAC_ADDRESS ?
           macStatistics_.incrementUpstreamMulticastDataDiscardDueToClobberRxHiddenBusy () :
           macStatistics_.incrementUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy ();

       // drop
       return false;
    }

   // noise common rx
   if (collisionType & COLLISION_TYPE_NOISE_COMMON_RX)
    {
       // get random rx power common
       const double fRandomNoiseMilliWatts = neighborManager_.getRandomRxPowerCommonNodesMilliWatts(pinfo.source_);

#ifdef VERBOSE_LOGGING
       // get avg rx power per msg
       const double dAverageNoiseMilliWatts = neighborManager_.getAverageRxPowerPerMessageMilliWatts();

       pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: common rx noise, avg %3.2f, random %3.2f dBm of noise to noise floor %3.2f dBm",
                               id_, pzLayerName, __func__,
                               MILLIWATT_TO_DB(dAverageNoiseMilliWatts),
                               MILLIWATT_TO_DB(fRandomNoiseMilliWatts),
                               MILLIWATT_TO_DB(dNoiseFloorMilliWatts));
#endif

       // adjust noise floor
       dNoiseFloorAdjustmentMilliWatts += fRandomNoiseMilliWatts;

       // bump counter
       macHeader.getDstNEM() == EMANE::NEM_BROADCAST_MAC_ADDRESS ?
          macStatistics_.incrementUpstreamMulticastNoiseRxCommon () :
          macStatistics_.incrementUpstreamUnicastNoiseRxCommon ();
    }

   // noise hidden rx
   if (collisionType & COLLISION_TYPE_NOISE_HIDDEN_RX)
    {
       // get random rx power hidden
       const double fRandomNoiseMilliWatts = neighborManager_.getRandomRxPowerHiddenNodesMilliWatts(pinfo.source_);

#ifdef VERBOSE_LOGGING
       // get avg rx power per msg hidden nodes
       const double dAverageNoiseMilliWatts = neighborManager_.getAverageRxPowerPerMessageHiddenNodesMilliWatts();

       pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: hidden rx noise, avg %3.2f, random %3.2f, dBm of noise to noise floor %3.2f dBm",
                               id_, pzLayerName, __func__,
                               MILLIWATT_TO_DB(dAverageNoiseMilliWatts),
                               MILLIWATT_TO_DB(fRandomNoiseMilliWatts),
                               MILLIWATT_TO_DB(dNoiseFloorMilliWatts));
#endif
       // adjust noise floor
       dNoiseFloorAdjustmentMilliWatts += fRandomNoiseMilliWatts;

       // bump counter
       macHeader.getDstNEM() == EMANE::NEM_BROADCAST_MAC_ADDRESS ?
          macStatistics_.incrementUpstreamMulticastNoiseHiddenRx () :
          macStatistics_.incrementUpstreamUnicastNoiseHiddenRx ();
    }

   // convert noise floor from milliwatts to dBm
   const float fNoiseFloorAdjusteddBm = MILLIWATT_TO_DB(dNoiseFloorMilliWatts + dNoiseFloorAdjustmentMilliWatts);

   // check sinr for this pkt
   if (checkPOR (fRxPowerdBm - fNoiseFloorAdjusteddBm, pkt.length (), macHeader.getDataRateIndex ()) == false)
     {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: rxpwr %3.2f dBm, adjusted noise floor %3.2f dBm, datarate %hu, drop",
                               id_, pzLayerName, __func__,
                               fRxPowerdBm, 
                               fNoiseFloorAdjusteddBm,
                               macHeader.getDataRateIndex ());
#endif

       // bump counter
       macHeader.getDstNEM() == EMANE::NEM_BROADCAST_MAC_ADDRESS ?
           macStatistics_.incrementUpstreamMulticastDataDiscardDueToSinr () :
           macStatistics_.incrementUpstreamUnicastDataDiscardDueToSinr ();

       // drop
       return false;
     }
   else
     {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: rxpwr %3.2f dBm, adjusted noise floor %3.2f dBm, datarate %hu, pass",
                               id_, pzLayerName, __func__,
                               fRxPowerdBm, 
                               fNoiseFloorAdjusteddBm,
                               macHeader.getDataRateIndex ());
#endif
       // pass
       return true;
     }
 }



void
IEEE80211ABG::IEEE80211abgMACLayer::processDownstreamPacket (EMANE::DownstreamPacket & pkt, const EMANE::ControlMessage &)
{
  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday ();

  // remove token
  if (removeToken () == false)
    {
      // bump counter
      macStatistics_.incrementDownstreamDiscardDueToFlowControl();

      // drop
      return;
    }

  // get packet info
  const EMANE::PacketInfo pinfo = pkt.getPacketInfo ();

  // create a queue entry
  DownstreamQueueEntry entry (pkt, tvCurrentTime, dscpToQindex(pinfo.dscp_));

  // check rts cts enable
  if((macConfig_.getRtsThreshold() != 0) && (macConfig_.getRtsThreshold() <= pkt.length()))
   {
     // set rts cts flag
     entry.bRtsCtsEnable_ = true;
   }

  // add queue entry
  if (downstreamQueue_.enqueue (entry))
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: enqueue src %hu, dst %hu, len %zu, dscp %hhu, total queued %zu",
           id_, pzLayerName, __func__, 
           pinfo.source_, 
           pinfo.destination_, 
           pkt.length (),
           pinfo.dscp_, 
           downstreamQueue_.total ());
#endif
    }
  // entry not added
  else
    {
      pPlatformService_->log (EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: failed to enqueue src %hu, dst %hu, len %zu, dscp %hhu, total queued %zu, drop",
                              id_, pzLayerName, __func__,
                              pinfo.source_, 
                              pinfo.destination_, 
                              pkt.length (),
                              pinfo.dscp_, 
                              downstreamQueue_.total ());

      // bump counter
      macStatistics_.incrementDownstreamDiscardDueToQueueRejection();

      // drop, replace token
      addToken ();
    }
}




void
IEEE80211ABG::IEEE80211abgMACLayer::processEvent (const EMANE::EventId &eventId, const EMANE::EventObjectState &state)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: event id %hu", 
                          id_, pzLayerName, __func__, eventId);
#endif

  // check event id
  switch(eventId)
   {
    case OneHopNbrListEvent::EVENT_ID:
      {
        // lock mutex, protects nbr mgr one hop nbr list until free to do so
        ACE_Guard < ACE_Thread_Mutex > m (mutex_);

        // the nbr manager knows how to handle these
        neighborManager_.processEvent(OneHopNbrListEvent(state));
      }
    break;
   }

  // no other events to be handled
}




ACE_THR_FUNC_RETURN 
IEEE80211ABG::IEEE80211abgMACLayer::processDownstreamQueue ()
{
  // while in the running mode
  while (running_)
    {
      // get entry, blocking call
      DownstreamQueueEntry entry = downstreamQueue_.dequeue ();

      // add token
      addToken ();

      // lock mutex, complete each pkt processing state without interruption
      ACE_Guard < ACE_Thread_Mutex > m (mutex_);

      do {
          // the amount of thime to wait
          ACE_Time_Value tvWaitTime = ACE_Time_Value::zero;

          // while in the waiting state
          while (running_ && ((tvWaitTime = pTxState_->getWaitTime (entry)) > ACE_OS::gettimeofday ()))
            {
              // get the wait time interval (yes its only for logging but is very helpful when timing goes wrong).
              const ACE_Time_Value waitInterval = tvWaitTime - ACE_OS::gettimeofday ();

#ifdef VERBOSE_LOGGING
              pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: waiting %06ld:%06ld in state %s",
                   id_, pzLayerName, __func__, 
                   waitInterval.sec (),
                   waitInterval.usec (), 
                   pTxState_->statename ());
#endif

              // condition timed wait on absolute time
              if (cond_.wait (&tvWaitTime) == 0)
                {
                  // woke up early, break out and call process
                  break;
                }
            }
        // while running and entry still needs to be processed
      } while (running_ && pTxState_->process (this, entry));

      // done with this entry, update final entry statistics
      pTxState_->update (this, entry);
    }

  // done
  return 0;
}



void
IEEE80211ABG::IEEE80211abgMACLayer::sendDownstreamMulticastData (DownstreamQueueEntry & entry)
{
  // set sequence number on first try, else retain current seq number
  if (entry.numRetries_ == 0)
    {
      entry.u16Seq_ = getNextSequenceNumber ();
    }

  // check retry logic enable
  const int numRetries = (ENABLE_TX_RETRY == true) ? entry.numRetries_ : 0;

  // set data mac header info
  MacHeader macHeader (MSG_TYPE_BROADCAST_DATA,                  // msg type
                       numRetries,                               // num retries
                       macConfig_.getMulticastDataRateIndex (),  // data rate
                       entry.u16Seq_,                            // sequence number
                       id_,                                      // src
                       entry.pkt_.getPacketInfo ().destination_, // dst
                       entry.tvDurationInterval_);               // duration


#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: %s", id_, pzLayerName, __func__,  macHeader.format().c_str());
#endif

  // send msg
  sendDownstreamMessage (entry, macHeader);

  // bump counter
  macStatistics_.incrementDownstreamMulticastDataSentToPhy ();
}




void
IEEE80211ABG::IEEE80211abgMACLayer::sendDownstreamUnicastData (DownstreamQueueEntry & entry)
{
  // set sequence number on first try, else retain current seq number
  if (entry.numRetries_ == 0)
    {
      entry.u16Seq_ = getNextSequenceNumber ();
    }

  // check retry logic enable
  const int numRetries = (ENABLE_TX_RETRY == true) ? entry.numRetries_ : macConfig_.getRetryLimit (entry.qidx_);

  // set data mac header info
  MacHeader macHeader (entry.bRtsCtsEnable_ ? 
                          MSG_TYPE_UNICAST_RTS_CTS_DATA : 
                          MSG_TYPE_UNICAST_DATA,                 // msg type
                       numRetries,                               // num retries
                       macConfig_.getUnicastDataRateIndex (),    // data rate index
                       entry.u16Seq_,                            // sequence number
                       id_,                                      // src
                       entry.pkt_.getPacketInfo ().destination_, // dst
                       entry.tvDurationInterval_);               // duration

#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: %s", id_, pzLayerName, __func__,  macHeader.format().c_str());
#endif

  // send msg
  sendDownstreamMessage (entry, macHeader);

  // bump counter
  entry.bRtsCtsEnable_ ? 
     macStatistics_.incrementDownstreamUnicastRtsCtsDataSentToPhy () :
     macStatistics_.incrementDownstreamUnicastDataSentToPhy ();
}



void
IEEE80211ABG::IEEE80211abgMACLayer::sendDownstreamUnicastCts (DownstreamQueueEntry & entry, EMANE::NEMId origin)
{
  // set sequence number on first try, else retain current seq number
  if (entry.numRetries_ == 0)
    {
      entry.u16Seq_ = getNextSequenceNumber ();
    }

  // check retry logic enable
  const int numRetries = (ENABLE_TX_RETRY == true) ? entry.numRetries_ : macConfig_.getRetryLimit (entry.qidx_);

  // set data mac header info
  MacHeader macHeader (MSG_TYPE_UNICAST_CTS_CTRL,                // msg type
                       numRetries,                               // num retries
                       macConfig_.getUnicastDataRateIndex (),    // data rate index
                       entry.u16Seq_,                            // sequence number
                       id_,                                      // src
                       origin,                                   // dst is the origin
                       entry.tvDurationInterval_);               // duration

  // reset duration for cts message
  entry.tvDurationInterval_ = ACE_Time_Value::zero;

#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: %s", id_, pzLayerName, __func__,  macHeader.format().c_str());
#endif


  // send msg
  sendDownstreamMessage (entry, macHeader);

  // bump counter
  macStatistics_.incrementDownstreamUnicastCtsSentToPhy ();
}



void
IEEE80211ABG::IEEE80211abgMACLayer::sendDownstreamMessage(const DownstreamQueueEntry & entry, MacHeader & macHeader)
{
  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday ();

  // get overhead if enabled
  const ACE_Time_Value tvOverhead = (ENABLE_WMM_OVERHEAD == true) ? 
                                     modeTiming_.getOverheadInterval(entry.qidx_) :
                                     ACE_Time_Value::zero;

  // update channel activity (self nem)
  neighborManager_.updateDataChannelActivity (id_,                                      // id
                                              macHeader.getType(),                      // msg type
                                              0.0,                                      // rx power (don't include our pwr)
                                              tvCurrentTime,                            // current time
                                              entry.tvDurationInterval_ + tvOverhead);  // duration + overhead
                                  

  // convert to network byte order    
  macHeader.toNetworkByteOrder ();

  // create a packet to send to phy
  EMANE::DownstreamPacket pkt (entry.pkt_.getPacketInfo (), entry.pkt_.get (), entry.pkt_.length ());

  // prepend mac header to outgoing packet
  pkt.prepend (&macHeader, sizeof (macHeader));

  // universal phy control send message
  EMANE::UniversalPhyControlSendMessage msg;

  // set the frequency info (1 entry)
  msg.setFrequencyInfo(EMANE::PHYTxFrequencyInfoItems(1, 
                              EMANE::PHYTxFrequencyInfo (0,                            // tx frequency Hz (use phy default)
                                                         ACE_Time_Value::zero,         // tx offset (T = 0) no offset
                                                         entry.tvDurationInterval_))); // duration
 
  // send the packet
  sendDownstreamPacket (EMANE::CommonMACHeader(registrationId_), // hdr
                        pkt,                                     // pkt
                        msg.buildControlMessage());              // ctrl
}




/**
 *
 * @brief callback to change current tx state
 * 
 * @param pState new state
 * 
 */
void
IEEE80211ABG::IEEE80211abgMACLayer::changeDownstreamState (TransmissionTxState * pState)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s:  %s => %s",
       id_, pzLayerName, __func__, 
       pTxState_->statename (), 
       pState->statename ());
#endif

  // change state
  pTxState_ = pState;
}




ACE_UINT16 
IEEE80211ABG::IEEE80211abgMACLayer::getNextSequenceNumber ()
{
  return seqNumber_++;
}




EMANE::NEMId 
IEEE80211ABG::IEEE80211abgMACLayer::getId () const
{
  return id_;
}




void
IEEE80211ABG::IEEE80211abgMACLayer::setDelayTime (DownstreamQueueEntry &entry)
{
  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday ();

  // estimated number of one hop nbrs
  const float fNumEstimatedOneHopNeighbors = neighborManager_.getNumberOfEstimatedOneHopNeighbors();

  // estimated number of two hop nbrs
  const float fNumEstimatedTwoHopNeighbors = neighborManager_.getNumberOfEstimatedTwoHopNeighbors();

  // estimated number of one and two hop nbrs
  const float fNumEstimatedOneAndTwoHopNeighbors = fNumEstimatedOneHopNeighbors + fNumEstimatedTwoHopNeighbors;

  // total one and two hop utilization
  const ACE_Time_Value tvTotalOneAndTwoBandWidthUtilization = neighborManager_.getTotalOneHopBandWidthUtilization() +
                                                              neighborManager_.getTotalTwoHopBandWidthUtilization();

  // get estimated num nbrs to this dst
  const float fNumEstimatedCommonNeighbors = neighborManager_.getNumberOfEstimatedCommonNeighbors(entry.pkt_.getPacketInfo().destination_);

  // get hidden channel activity
  const float fHiddenChannelActivity = neighborManager_.getHiddenChannelActivity(entry.pkt_.getPacketInfo().destination_);

  // avg msg duration this is for one hops nbrs
  const ACE_Time_Value tvAverageMessageDuration = neighborManager_.getAverageMessageDuration();

  // activity timer interval
  const ACE_Time_Value tvDeltaT = macConfig_.getChannelActivityTimerInterval();

  // slot time in usec
  const float fSlotTimeUsec = modeTiming_.getSlotTimeUsec();

  // amount of additional delay (if required)
  const float X2 = pPlatformService_->getRandomProbability();

  // get the contention window
  const float CW = modeTiming_.getContentionWindow(entry.qidx_, entry.numRetries_);

  // set initial pre delay
  //ACE_Time_Value tvPreDelay = USEC_TO_TV(floor (X2 * CW) * fSlotTimeUsec);
  ACE_Time_Value tvPreDelay = ACE_Time_Value::zero;

  // set the initial post delay
  ACE_Time_Value tvPostDelay = ACE_Time_Value::zero;

  // check number of estimated one and two hop nbrs
  if(fNumEstimatedOneAndTwoHopNeighbors > 1)
   {
     // calcuating excess overhead per neighbor in excess of 2 for the estimated average message duration
     const ACE_Time_Value tvMessageDurationOverheadExtra = USEC_TO_TV(fNumEstimatedOneAndTwoHopNeighbors - 2) * ((CW * fSlotTimeUsec) / 2.0);

     // probability of additional deley required
     const float X1 = pPlatformService_->getRandomProbability();
     const double delayTimeFactor = (TV_TO_SEC(tvTotalOneAndTwoBandWidthUtilization) / TV_TO_SEC(tvDeltaT));

     //if(X1 <= (TV_TO_SEC(tvTotalOneAndTwoBandWidthUtilization) / TV_TO_SEC(tvDeltaT)))
     if(X1 <= delayTimeFactor)
      {
         // get pre delay
         const float fNodeDelay = floor(X2 * fNumEstimatedOneAndTwoHopNeighbors);

         // add to the pre dealy
         tvPreDelay += fNodeDelay * tvAverageMessageDuration;

         // if have node delay
         if(fNodeDelay > 0)
          {
            // remove the overhead
            tvPreDelay -= tvMessageDurationOverheadExtra;
          }

         // set post delay
         //tvPostDelay = ((fNumEstimatedOneAndTwoHopNeighbors - 1) * tvAverageMessageDuration) - tvPreDelay - tvMessageDurationOverheadExtra;
         tvPostDelay = delayTimeFactor * delayTimeFactor * ((fNumEstimatedOneAndTwoHopNeighbors - 1) * tvAverageMessageDuration);
      }
   }

  // add defer time to pre delay
  tvPreDelay += modeTiming_.getDeferInterval(entry.qidx_);

  // initial set flag that collision will not occur
  entry.bCollisionOccured_ = false;

  // check tx retry enable
  if(ENABLE_TX_RETRY == false)
   {
     // probability of tx collision
     const float X3 = pPlatformService_->getRandomProbability();

     // check probability
     if(X3 < (fNumEstimatedCommonNeighbors / CW))
      {
        // set flag that collision will occur
        entry.bCollisionOccured_ = true;
      }
     else
      {
        // probability of hidden collision
        const float X4 = pPlatformService_->getRandomProbability();

        // const C2
        const float C2 = 0.1;

        float H = fHiddenChannelActivity - C2;

        // clamp low
        if(H < 0.0)
         {
           H = 0.0;
         }
        // clamp high
        else if(H > (1.0 - C2))
         {
           H = (1.0 - C2);
         }
   
        // check probability
        if(X4 < H)
         {
           // set flag that collision will occur
           entry.bCollisionOccured_ = true;
         }
      }
   }
 
  // set pre delay time absolute
  entry.tvPreTxDelayTime_ = tvPreDelay + tvCurrentTime;

  // set post delay interval
  entry.tvPostTxDelayInterval_ = tvPostDelay;
 
#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: est nbrs 1 hop %3.2f, 2 hop %3.2f, bw %ld:%06ld, avg duration %ld:%06ld,"
                                               " pre delay %ld:%06ld, post delay %ld:%06ld, tx collision %s",
                          id_, pzLayerName, __func__, 
                          fNumEstimatedOneHopNeighbors, 
                          fNumEstimatedTwoHopNeighbors, 
                          tvTotalOneAndTwoBandWidthUtilization.sec(), 
                          tvTotalOneAndTwoBandWidthUtilization.usec(), 
                          tvAverageMessageDuration.sec(), 
                          tvAverageMessageDuration.usec(), 
                          tvPreDelay.sec(), 
                          tvPreDelay.usec(),
                          tvPostDelay.sec(),
                          tvPostDelay.usec(),
                          entry.bCollisionOccured_ == true ? "true" : "false");
#endif
}




IEEE80211ABG::IEEE80211abgMACLayer::COLLISION_TYPE
IEEE80211ABG::IEEE80211abgMACLayer::checkForRxCollision(EMANE::NEMId src, ACE_UINT8 qidx, ACE_UINT8 retries)
{
   int result = COLLISION_TYPE_NONE;

   float fNumEstimatedOneHopNeighbors = neighborManager_.getNumberOfEstimatedOneHopNeighbors();

   float fNumEstimatedCommonNeighbors = neighborManager_.getNumberOfEstimatedCommonNeighbors(src);

   const float fHiddenChannelActivity = neighborManager_.getHiddenChannelActivity(src);

   const ACE_Time_Value tvTotalOneHopBandWidthUtilization = neighborManager_.getTotalOneHopBandWidthUtilization();

   const ACE_Time_Value tvLocalBandWidthUtilization = neighborManager_.getAllBandWidthUtilization(id_);

   const ACE_Time_Value tvRemoteBandWidthUtilization = neighborManager_.getAllBandWidthUtilization(src);

   const ACE_Time_Value tvDeltaT = macConfig_.getChannelActivityTimerInterval();

   float fBandWidthUtilizationFactorAdjusted = (TV_TO_SEC(tvTotalOneHopBandWidthUtilization - 
                                                           tvLocalBandWidthUtilization - 
                                                           tvRemoteBandWidthUtilization) / TV_TO_SEC(tvDeltaT));

   float fBandWidthUtilizationFactorActual = (TV_TO_SEC(tvTotalOneHopBandWidthUtilization) / TV_TO_SEC(tvDeltaT));

   const float A = neighborManager_.getLocalNodeTx();

   const float X1 = pPlatformService_->getRandomProbability();

   const float X2 = pPlatformService_->getRandomProbability();

   const float X3 = pPlatformService_->getRandomProbability();

   const float CW = modeTiming_.getContentionWindow(qidx, retries);

   const float C1 = 0.0;

   float P1 = 0.0;

   float P2 = 0.0;

   float P3 = 0.0;

   float P4 = 0.0;

   // bump adjusted utilization
   if(fBandWidthUtilizationFactorAdjusted > 1.0)
    {
      fBandWidthUtilizationFactorAdjusted = 1.0;
    }

   // cap actual utilization
   if(fBandWidthUtilizationFactorActual > 1.0)
    {
      fBandWidthUtilizationFactorActual = 1.0;
    }

   // check for estimated nbrs 
   if(fNumEstimatedOneHopNeighbors > 0)
    {
      //P1 = fBandWidthUtilizationFactorAdjusted * A * (fNumEstimatedOneHopNeighbors / CW);
      P1 = fBandWidthUtilizationFactorAdjusted * fBandWidthUtilizationFactorAdjusted * A * (fNumEstimatedOneHopNeighbors / CW);
    
      if(X1 < P1)
       {
         // set rx during tx collision clobber, no need to continue
         return COLLISION_TYPE_CLOBBER_RX_DURING_TX;
       }
    }

   // check for hidden channel activity
   if(fHiddenChannelActivity > 0)
    {
      fBandWidthUtilizationFactorActual = 1 + log10(fBandWidthUtilizationFactorActual);

      float bwDelta = fBandWidthUtilizationFactorActual - (2.0 * fHiddenChannelActivity);

      if(bwDelta < 0.0)
       {
         bwDelta = 0.0;
       }

      P3 = 1.0 - fBandWidthUtilizationFactorActual + bwDelta + C1;

      if(X2 > P3)
       {
         P4 = 0.5;

         if(X3 < P4)
          {
             // set hidden rx noise
             result |= COLLISION_TYPE_NOISE_HIDDEN_RX;
          }
         else
          {
             // set rx hidden busy clobber, no need to continue
             return COLLISION_TYPE_CLOBBER_RX_HIDDEN_BUSY;
          }
       }
    }

   // check for estimated common nbrs
   if(fNumEstimatedCommonNeighbors > 0)
    {
      if(CW > fNumEstimatedCommonNeighbors)
        {
          //P2 = fBandWidthUtilizationFactorAdjusted * fBandWidthUtilizationFactorAdjusted * collisionTable_.getCollisionFactor(fNumEstimatedCommonNeighbors, CW) / 100.0;
          P2 = (fNumEstimatedCommonNeighbors / CW) * fBandWidthUtilizationFactorAdjusted * fBandWidthUtilizationFactorAdjusted;
        }
      else
        {
          P2 = 1.0;
        }

      if(P2 >= X1)
        {
          // set common rx noise
          result |= COLLISION_TYPE_NOISE_COMMON_RX;
        }
    }


#ifdef VERY_VERBOSE_LOGGING
   pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, nbrs [active %zd, est %3.2f, common %3.2f, hidden %3.2f]"
                          " hidden activity %3.2f, bw utlization [actual %3.2f, adjusted %3.2f] A %3.2f CW %3.2f, X1 %3.2f, X2 %3.2f, X3 %3.2f, C1 %3.2f, P1 %3.2f, P2 %3.2f, P3 %3.2f P4 %3.2f, 0x%02X",
                          id_, pzLayerName, __func__, 
                          src,
                          neighborManager_.getTotalActiveOneHopNeighbors(),
                          fNumEstimatedOneHopNeighbors,
                          fNumEstimatedCommonNeighbors,
                          neighborManager_.getNumberOfEstimatedHiddenNeighbors(src),
                          fHiddenChannelActivity,
                          fBandWidthUtilizationFactorActual,
                          fBandWidthUtilizationFactorAdjusted,
                          A, CW, X1, X2, X3, C1, P1, P2, P3, P4, result);
#endif

  return (COLLISION_TYPE) result;
}




void 
IEEE80211ABG::IEEE80211abgMACLayer::processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value & tvEventTime, const void *arg)
{
  // get processing delay
  const ACE_Time_Value tvProcessingDelay = ACE_OS::gettimeofday() - tvEventTime;
 
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"MACI %03hu %s::%s task type %u, delay %06ld:%06ld, event id %ld", 
         id_, pzLayerName, __func__,
         taskType,
         eventId,
         tvProcessingDelay.sec(),
         tvProcessingDelay.usec());
#else
  (void) eventId;
#endif

  // channel usage callback
  if(taskType == CHANNEL_USAGE_CALLBACK)
   {
      // current time minus scheduled wakeup time + wakeup interval
      const ACE_Time_Value tvElapsedTime = ACE_OS::gettimeofday () - tvEventTime + macConfig_.getChannelActivityTimerInterval();

      // lock mutex, protect nbr manager statistics until all pending operations are complete
      ACE_Guard < ACE_Thread_Mutex > m (mutex_);

      // reset statistics
      neighborManager_.resetStatistics(tvElapsedTime);
   }
  // upsteam packet callback
  else if(taskType == UPSTREAM_PACKET_CALLBACK)
   {
     // pkt ptr passed in as arg
     EMANE::UpstreamPacket *p = (EMANE::UpstreamPacket*) arg;

     // no mutex lock required since this just gets sent upstream
     
     // send upstream
     sendUpstreamPacket (*p);

     // bump counter
     p->getPacketInfo().destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS ?
        macStatistics_.incrementUpstreamMulticastDataSentToTransport () :
        macStatistics_.incrementUpstreamUnicastDataSentToTransport ();

     // delete pkt
     delete p;
   }

  // done
  return;
}





bool 
IEEE80211ABG::IEEE80211abgMACLayer::isDuplicate (EMANE::NEMId src, ACE_UINT16 seq)
{
  // get current time
  const ACE_Time_Value tvCurrentTime = ACE_OS::gettimeofday ();

  // entry vector len
  const size_t historySize = 16;

  // entry valid interval 5 seconds
  const ACE_Time_Value validInterval (5, 0);

  // check for src
  DuplicateMapIter dupIter = duplicateMap_.find (src);

  // src not found
  if (dupIter == duplicateMap_.end ())
    {
      // sequence vector
      SequenceVector v;

      // reserve history len
      v.reserve (historySize);

      // add entry to vector
      v.push_back (SequenceEntry (seq, tvCurrentTime));

      // add vector to map
      duplicateMap_.insert (std::make_pair (src, v));

      // not a duplicate
      return false;
    }
  // src found
  else
    {
      // oldest index
      size_t oldestIndex = 0;

      // oldest time starts as current time
      ACE_Time_Value oldestTime = tvCurrentTime;

      // check for seq
      for (size_t idx = 0; idx < dupIter->second.size (); ++idx)
        {
          // seq match
          if (dupIter->second.at (idx).seq_ == seq)
            {
              // within the time window
              if ((dupIter->second.at (idx).tv_ + validInterval) >= tvCurrentTime)
                {
                  // is duplicate
                  return true;
                }
              else
                {
                  // update entry
                  dupIter->second.at (idx) = SequenceEntry (seq, tvCurrentTime);

                  // not a duplicate
                  return false;
                }
            }
          // no match
          else
            {
              // check entry age
              if (dupIter->second.at (idx).tv_ < oldestTime)
                {
                  // new oldest time
                  oldestTime = dupIter->second.at (idx).tv_;

                  // new oldest index
                  oldestIndex = idx;
                }
            }
        }

      // vector not full
      if (dupIter->second.size () < historySize)
        {
          // add entry
          dupIter->second.push_back (SequenceEntry (seq, tvCurrentTime));
        }
      // vector full
      else
        {
          // replace entry
          dupIter->second.at (oldestIndex) = SequenceEntry (seq, tvCurrentTime);
        }

      // not a duplicate
      return false;
    }
}



bool 
IEEE80211ABG::IEEE80211abgMACLayer::addToken ()
{
  // check flow control enabled
  if (macConfig_.getFlowControlEnable ())
    {
      // add token 
      const EMANEUtils::FlowControlManager::FlowControlStatus flowControlStatus = flowControlManager_.addToken ();

      // check status
      if (flowControlStatus.bStatus_ == false)
        {
          pPlatformService_->log (EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: failed to add token, %hu/%hu flow control tokens available",
               id_, pzLayerName, __func__,
               flowControlStatus.ui16TokensAvailable_, 
               flowControlStatus.ui16ShadowTokenCount_);

          // failed
          return false;
        }
      else
        {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: added token, %hu/%hu flow control tokens available",
               id_, pzLayerName, __func__,
               flowControlStatus.ui16TokensAvailable_, 
               flowControlStatus.ui16ShadowTokenCount_);
#endif

          // success
          return true;
        }
    }
  else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: flow control disabled", id_, pzLayerName, __func__);
#endif

      // success
      return true;
    }
}



bool 
IEEE80211ABG::IEEE80211abgMACLayer::removeToken ()
{
  // check flow control enabled
  if (macConfig_.getFlowControlEnable ())
    {
      // remove token
      const EMANEUtils::FlowControlManager::FlowControlStatus flowControlStatus = flowControlManager_.removeToken ();

      // check status
      if (flowControlStatus.bStatus_ == false)
        {
          pPlatformService_->log (EMANE::ERROR_LEVEL, "MACI %03hu %s::%s: failed to remove token, %hu/%hu flow control tokens available",
               id_, pzLayerName, __func__,
               flowControlStatus.ui16TokensAvailable_, 
               flowControlStatus.ui16ShadowTokenCount_);

          // failed
          return false;
        }
      else
        {
#ifdef VERBOSE_LOGGING
          pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: removed token, %hu/%hu flow control tokens available",
               id_, pzLayerName, __func__,
               flowControlStatus.ui16TokensAvailable_, 
               flowControlStatus.ui16ShadowTokenCount_);
#endif
          // success
          return true;
        }
    }
  else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: flow control disabled", id_, pzLayerName, __func__);
#endif

      // success
      return true;
    }
}



bool
IEEE80211ABG::IEEE80211abgMACLayer::checkPOR (float fSINR, size_t packetSize, ACE_UINT16 u16DataRateIndex)
{
  // find por
  const float fPCR = pcrManager_.getPCR (fSINR, packetSize, u16DataRateIndex);

  // random value from 0.0 to 1.0 inclusive
  const float X = pPlatformService_->getRandomProbability();

  // pcr >= random value
  const bool bResult = (fPCR >= X);

#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: sinr %3.2f, pcr %3.2f %s rand %3.3f", 
       id_, pzLayerName, __func__, 
       fSINR, fPCR, bResult == true ? ">=" : "<", X);
#endif

  // return result
  return bResult;
}



ACE_UINT8 
IEEE80211ABG::IEEE80211abgMACLayer::dscpToQindex(ACE_UINT8 dscp) const
{
  // default value is 0
  ACE_UINT8 result = 0;

  if (macConfig_.getWmmEnable () == true)
    {
      // with WMM, map the 4 access categories to different queues
      if (dscp >= 8 && dscp <= 23)
        {
          result = 1;
        }
      else if (dscp >= 32 && dscp <= 47)
        {
          result = 2;
        }
      else if (dscp >= 48 && dscp <= 63)
        {
          result = 3;
        }
      else
        {
          result = 0;
        }
    }

#ifdef VERBOSE_LOGGING
  pPlatformService_->log (EMANE::DEBUG_LEVEL, "MACI %03hu %s::%s: wmm %s, dscp %hhu, Qindex %hhu",
       id_, pzLayerName, __func__, (macConfig_.getWmmEnable () == true ? "on" : "off"), dscp, result);
#endif


  return result;
}



IEEE80211ABG::MACStatistics & 
IEEE80211ABG::IEEE80211abgMACLayer::getStatistics ()
{
  return macStatistics_;
}


IEEE80211ABG::MACConfig & 
IEEE80211ABG::IEEE80211abgMACLayer::getConfig ()
{
  return macConfig_;
}


IEEE80211ABG::ModeTimingParameters & 
IEEE80211ABG::IEEE80211abgMACLayer::getModeTiming ()
{
  return modeTiming_;
}

DECLARE_MAC_LAYER (IEEE80211ABG::IEEE80211abgMACLayer);
