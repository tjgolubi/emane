/*
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "timinganalysisshim.h"

#include "emaneutils/parameterconvert.h"

#include <sstream>

#include <ace/OS_NS_sys_time.h>

namespace
{
  EMANE::ConfigurationDefinition defs[] = 
  {
   
    {true,true,1,"maxqueuesize","0",0,"Max size of the Delta time storage queue"},
    {0,0,0,0,0,0,0},
  };
}

EMANE::TimingAnalysisShimLayer::TimingAnalysisShimLayer(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService) :
  ShimLayerImplementor(id, pPlatformService),
  u32MaxQueueSize_(0),
  u16PacketID_(0)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::TimingAnalysisSHIMLayer:",id_);
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

EMANE::TimingAnalysisShimLayer::~TimingAnalysisShimLayer() 
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::~TimingAnalysisSHIMLayer",id_);
}

void EMANE::TimingAnalysisShimLayer::initialize() 
  throw(EMANE::InitializeException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::initialize",id_);
}

void EMANE::TimingAnalysisShimLayer::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::configure",id_);
  Component::configure(items);
}

void EMANE::TimingAnalysisShimLayer::start()
  throw(EMANE::StartException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::start",id_);

  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();

  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "maxqueuesize")
                {
                  u32MaxQueueSize_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toUINT32();
                   pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::start maxqueuesize: %d"
                                      ,id_,u32MaxQueueSize_);
                }
             
            }
          else if(iter->second.bRequired_)
            {
              pPlatformService_->log(EMANE::ERROR_LEVEL,"PHYI %03d IEEE80211abgPHYLayer::start. missing %s",
                  id_, iter->first.c_str());
              std::stringstream ssDescription;
              ssDescription<<"TimingAnalysisShimLayer: Missing configuration item "<<iter->first<<std::ends;
              throw EMANE::StartException(ssDescription.str());
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"TimingAnalysisShimLayer: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }

}

void EMANE::TimingAnalysisShimLayer::stop()
  throw(EMANE::StopException)
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::stop",id_);

  char filename[256];

  queueContainer timingInfo;

  sprintf(filename,"/tmp/timinganalysis%d.txt",id_);

  FILE *fp = fopen(filename,"w");

  if(fp != NULL)
   {
     while(queue_.empty() != true)
      {
        timingInfo = queue_.front();

        fprintf(fp,"%hu %hu %06u.%06u %06ld.%06ld\n",
                timingInfo.hdr.u16Source_,
                timingInfo.hdr.u16PacketID_,
                timingInfo.hdr.u32TimeSec_, 
                timingInfo.hdr.u32TimeUsec_,
                timingInfo.receiveTime.sec(),
                timingInfo.receiveTime.usec());
        queue_.pop();
      }
     fclose(fp);
   }
  else
   {
     pPlatformService_->log(EMANE::ERROR_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::stop: Can't open file",id_); 
   }
}

void EMANE::TimingAnalysisShimLayer::destroy()
  throw()
{
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::destroy",id_);
}

void EMANE::TimingAnalysisShimLayer::processUpstreamControl(const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::processUpstreamControl",id_);
#endif

  sendUpstreamControl(msg);
}

void EMANE::TimingAnalysisShimLayer::processDownstreamControl(const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::processDownstreamControl",id_);
#endif

  sendDownstreamControl(msg);
}

void EMANE::TimingAnalysisShimLayer::processUpstreamPacket(EMANE::UpstreamPacket & pkt,
                                               const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::processUpstreamPacket",id_);
#endif

  // get the SHIM header
   TimingShimHeader hdr;

   queueContainer timingInfo;

  // check pkt length
  if(pkt.get() != NULL && pkt.length() >= sizeof(hdr))
  {
      // get phy header
      memcpy(&hdr, pkt.get(), sizeof(hdr));

      // convert to host byte order
      hdr.toHostByteOrder();
      timingInfo.hdr = hdr;

      timingInfo.receiveTime = ACE_OS::gettimeofday();

#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::processUpstreamPacket:SHIM Src: %hu PktID: %hu TxTime: %06u.%06u"
                             "RxTime: %06ld.%06ld",
          id_,
          timingInfo.hdr.u16Source_,
          timingInfo.hdr.u16PacketID_,         
          timingInfo.hdr.u32TimeSec_,
          timingInfo.hdr.u32TimeUsec_,
          timingInfo.receiveTime.sec(),
          timingInfo.receiveTime.usec());
#endif

      if(u32MaxQueueSize_ != 0)
      {
        if(queue_.size() >= u32MaxQueueSize_)
        {
          queue_.pop();
        }
      }  

      queue_.push(timingInfo);

      // strip hdr
      pkt.strip(sizeof(hdr));        
  }

  sendUpstreamPacket(pkt,msg);
  
}

void EMANE::TimingAnalysisShimLayer::processDownstreamPacket(EMANE::DownstreamPacket & pkt,
                                               const EMANE::ControlMessage & msg)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::processDownstreamPacket",id_);
#endif

  ACE_Time_Value pktSourceTime = ACE_OS::gettimeofday();

  TimingShimHeader hdr(pktSourceTime.sec(), pktSourceTime.usec(),u16PacketID_++,id_);

  hdr.toNetworkByteOrder();

  pkt.prepend(&hdr,sizeof(hdr));

  sendDownstreamPacket(pkt,msg);
}

void EMANE::TimingAnalysisShimLayer::processEvent(const EMANE::EventId &,
                                      const EMANE::EventObjectState &)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::processEvent",id_);
#endif
}


void EMANE::TimingAnalysisShimLayer::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{
#ifdef VERBOSE_LOGGING
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"SHIM %03d TimingAnalysisSHIMLayer::processTimedEvent",id_);
#endif
}

DECLARE_SHIM_LAYER(EMANE::TimingAnalysisShimLayer);
