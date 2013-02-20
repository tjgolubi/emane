/*
 * Copyright (c) 2008-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "virtualtransport.h"
#include "emane/emanedownstreampacket.h"
#include "emane/emanedownstreamtransport.h"
#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/netutils.h"
#include "emaneutils/parameterconvert.h"

#include <sstream>

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
    // reqd,  default,  count, name,                value,               type, description
      {true,   true,    1,     "bitrate",           "0",                  0,   0},
      {true,   true,    1,     "devicepath",        NETWORK_DEVICE_PATH,  0,   0},
      {true,   true,    1,     "device",            "emane0",             0,   0},
      {false,  false,   1,     "mask",              NULL,                 0,   0},
      {false,  false,   1,     "address",           NULL,                 0,   0},
      {true,   true,    1,     "arpmode",           "on",                 0,   0},
      {true,   true,    1,     "broadcastmode",     "off",                0,   0},
      {true,   true,    1,     "arpcacheenable",    "on",                 0,   0},
      {true,   true,    1,     "flowcontrolenable", "off",                0,   0},
      {0,0,0,0,0,0,0},
    };
}

VirtualTransport::VirtualTransport(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService):
  EthernetTransport(id, pPlatformService),
  pTunTap_(NULL),
  pBitPool_(NULL),
  threadRead_(0),
  bCanceled_(false),
  flowControl_(*this),
  bFlowControlEnable_(false)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

VirtualTransport::~VirtualTransport()
{
   if(pTunTap_ != NULL)
     {
       delete pTunTap_;

       pTunTap_ = NULL;
     }

   if(pBitPool_ != NULL)
     {
       delete pBitPool_;

       pBitPool_ = NULL;
     }
}

void VirtualTransport::initialize()
  throw(EMANE::InitializeException)
{
  // create tuntap
  pTunTap_ = new TunTap(pPlatformService_);

  // create bit pool
  pBitPool_ = new BitPool(pPlatformService_);
}


void VirtualTransport::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  Component::configure(items);
}


void VirtualTransport::start()
  throw(EMANE::StartException)
{

  bool bHaveAddress = false;

  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();
  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {      
              if(iter->first == "bitrate")
                {
                  // value is in bits
                  const ACE_UINT64 bitRate = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue().c_str()).toUINT64();

                  // set the bit rate
                  pBitPool_->size(bitRate);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03hu VirtualTransport::%s %s: %ju",
                      id_, __func__, iter->first.c_str(), bitRate);
                }
              else if(iter->first == "devicepath")
                {
                  sDevicePath_ = iter->second.item_.getValue();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %s",
                      id_, __func__, iter->first.c_str(), sDevicePath_.c_str());
                }
              else if(iter->first == "device")
                {
                  sDeviceName_ = iter->second.item_.getValue();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %s", 
                      id_, __func__, iter->first.c_str(), sDeviceName_.c_str());
                }
              else if(iter->first == "mask")
                {
                  mask_.set("0",iter->second.item_.getValue().c_str());

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %s", 
                      id_, __func__, iter->first.c_str(), mask_.get_host_addr());
                }
              else if(iter->first == "address")
                {
                  address_.set("0",iter->second.item_.getValue().c_str());

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %s", 
                      id_, __func__, iter->first.c_str(), address_.get_host_addr());

                  bHaveAddress = true;
                }
              else if(iter->first == "arpmode")
                {
                  bARPMode_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue().c_str()).toBool();

                    pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %d",
                        id_, __func__, iter->first.c_str(), bARPMode_);
                }
              else if(iter->first == "broadcastmode")
                {
                  bBroadcastMode_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue().c_str()).toBool();

                    pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %d", 
                        id_, __func__, iter->first.c_str(), bBroadcastMode_);
                }
              else if(iter->first == "arpcacheenable")
                {
                  bArpCacheMode_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue().c_str()).toBool();

                    pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %d", 
                        id_, __func__, iter->first.c_str(), bArpCacheMode_);
                }
              else if(iter->first == "flowcontrolenable")
                {
                  bFlowControlEnable_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue().c_str()).toBool();

                    pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s: %d", 
                        id_, __func__, iter->first.c_str(), bFlowControlEnable_);
                }
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"VirtualTransport: Missing configuration item "<<iter->first<<std::ends;
              throw EMANE::StartException(ssDescription.str());
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"VirtualTransport: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }

  // open tuntap
  if(pTunTap_->open(sDevicePath_.c_str(), sDeviceName_.c_str()) < 0) 
    {
      std::stringstream ssDescription;
      ssDescription << "could not open tuntap device path " 
                    << sDevicePath_ 
                    << " name " 
                    << sDeviceName_ 
                    << std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

  // set ether addr
  if(pTunTap_->set_ethaddr (id_) < 0) 
    {
      std::stringstream ssDescription;
      ssDescription << "could not set tuntap eth address " 
                    << id_
                    << std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

  // activate tuntap
  if(pTunTap_->activate(bARPMode_) < 0) 
    {
      std::stringstream ssDescription;
      ssDescription << "could not activate tuntap arp mode " 
                    << bARPMode_
                    << std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

  // was an interface address provided
  if(bHaveAddress == true)
    {  
      // set tuntap address/mask
      if(pTunTap_->set_addr(address_, mask_) < 0) 
        {
          std::stringstream ssDescription;
          ssDescription << "could not set tuntap address " 
                        << address_.get_host_addr();
          ssDescription << " mask " 
                        << mask_.get_host_addr() 
                        << std::ends;
          throw EMANE::StartException(ssDescription.str());
        }
    }

  // unset canceled flag
  bCanceled_ = false;

  // start tuntap read thread
  EMANEUtils::spawn(*this, &VirtualTransport::readDevice, &threadRead_);
}

void VirtualTransport::postStart()
{
  if(bFlowControlEnable_)
    {
      // start flow control
      flowControl_.start();
    }
}

void VirtualTransport::stop()
  throw(EMANE::StopException)
{
  // if thread was created
  if(threadRead_ != 0)
    {
      if(bFlowControlEnable_)
        {
          // stop flow control
          flowControl_.stop();
        }

      // set canceled flag
      bCanceled_ = true;

      // cancel read thread
      ACE_OS::thr_cancel(threadRead_);

      // join read thread
      ACE_OS::thr_join(threadRead_,0,0);

      // unset thread id
      threadRead_ = 0;
    }

  // deactivate tuntap
  pTunTap_->deactivate();

  // close tuntap
  pTunTap_->close();
}


void VirtualTransport::destroy()
  throw()
{ }


void VirtualTransport::processUpstreamPacket(EMANE::UpstreamPacket & pkt,
                                             const EMANE::ControlMessage &)
{
  // frame sanity check
  if(verifyFrame(pkt.get(), pkt.length()) < 0)
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s frame error", id_, __func__);
    }

  // get packet info
  const EMANE::PacketInfo info = pkt.getPacketInfo();

  // ether header
  const EMANEUtils::EtherHeader * pEtherHeader = (const EMANEUtils::EtherHeader*) pkt.get();

  // update arp cache
  updateArpCache(pEtherHeader, info.source_);

  // iovec for tuntap io
  iovec iov;

  iov.iov_base = (char*)pkt.get();
  iov.iov_len  = pkt.length();

  // send to tuntap
  if(pTunTap_->writev(&iov, 1) < 0) 
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s", id_, __func__, ACE_OS::strerror(errno));
    }
  else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s src %hu, dst %hu, dscp %hhu, length %zu", 
          id_, __func__, info.source_, info.destination_, info.dscp_, pkt.length());
#endif

      // drain the bit pool 
      const size_t sizePending = pBitPool_->get(pkt.length() * 8);

      // check for bitpool error
      if(sizePending != 0)
       {
         pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s bitpool request error %zd of %zd", 
             id_, __func__, sizePending, pkt.length() * 8);
       }
    }
}


void VirtualTransport::processUpstreamControl(const EMANE::ControlMessage & ctrl)
{
  if(bFlowControlEnable_ && ctrl.getMajorIdentifier() == EMANE::EMANE_FLOWCONTROL_MAJOR_ID)
    {
      flowControl_.processFlowControlMessage(ctrl);

#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL,
          "TRANSPORTI %03d VirtualTransport processUpstreamControl Flow Control Message, %u tokens available",
          id_, flowControl_.getAvailableTokens());
#endif
    }
  else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL,
          "TRANSPORTI %03d VirtualTransport:%sprocessUpstreamControl len: %zu", 
          id_, __func__, ctrl.length());
#endif
    }
}


void VirtualTransport::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{
  pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s", id_, __func__);
}


void VirtualTransport::processEvent(const EMANE::EventId&, const EMANE::EventObjectState&)
{
  pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s", id_, __func__);
}


ACE_THR_FUNC_RETURN VirtualTransport::readDevice()
{
  ACE_UINT8 buf[EMANEUtils::IP_MAX_PACKET];

  while(!bCanceled_)
    {
      // read result 
      ssize_t len;

      // clear buf
      memset(buf, 0x0, sizeof(buf));

      // iovec for tuntap io
      iovec iov;

      iov.iov_base = (char*)buf;
      iov.iov_len  = sizeof(buf);

      // read from tuntap
      if((len = pTunTap_->readv(&iov, 1)) < 0) 
        {
          pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s %s", id_, __func__, ACE_OS::strerror(errno));

          break;
        }
      else
        {
          // frame sanity check
          if(verifyFrame(buf, len) < 0)
            {
              pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s frame error", id_, __func__);
            }
          else
            {
              // NEM destination
              EMANE::NEMId nemDestination;
   
              // pkt tos/qos converted to dscp
              ACE_UINT8 dscp;

              // get dst and dscp values
              if(parseFrame((const EMANEUtils::EtherHeader *)buf, nemDestination, dscp) < 0)
                {
                  pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s frame parse error", id_, __func__);
                }
              else
                {
#ifdef VERBOSE_LOGGING
                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s src %hu, dst %hu, dscp %hhu, length %zd", 
                                          id_, __func__, id_, nemDestination, dscp, len);
#endif

                  // create downstream packet with packet info
                  EMANE::DownstreamPacket pkt(EMANE::PacketInfo (id_, nemDestination, dscp), buf, len);

                  // check flow control 
                  if(bFlowControlEnable_)
                   {
                     // block and wait for an available flow control token
                     const EMANEUtils::FlowControl::FlowControlStatus flowControlStatus = flowControl_.removeToken();

                     if(flowControlStatus.bStatus_ == false)
                      {
#ifdef VERBOSE_LOGGING
                        pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s failed to remove token, %hu tokens available", 
                            id_, __func__, flowControlStatus.ui16TokensAvailable_);
#endif
                        // done
                        break;
                      }
                     else
                      {
#ifdef VERBOSE_LOGGING
                        pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d VirtualTransport::%s removed token, %hu tokens available", 
                            id_, __func__, flowControlStatus.ui16TokensAvailable_);
#endif
                      }
                   }
 
                  // send to downstream transport
                  sendDownstreamPacket(pkt);

                  // drain the bit pool 
                  const size_t sizePending = pBitPool_->get(len * 8);

                  // check for bitpool error
                  if(sizePending != 0)
                   {
                     pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d VirtualTransport::%s bitpool request error %zd of %zd", 
                         id_, __func__, sizePending, len * 8);
                   }
                }
            }
        }
    }

    return (ACE_THR_FUNC_RETURN) 0;
}



DECLARE_TRANSPORT(VirtualTransport);
