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

#include <ace/OS_NS_Thread.h>

#ifdef WIN32
#include <winsock2.h>
#include <iphlpapi.h>
#endif

#include "rawtransport.h"
#include "emane/emanedownstreampacket.h"
#include "emane/emanedownstreamtransport.h"
#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/parameterconvert.h"

#include <sstream>

namespace
{
  EMANE::ConfigurationDefinition defs[] =
    {
    // reqd,  default,  count, name,            value,    type, description
      {true,  false,    1,    "device",         NULL,     0,    0},
      {true,  true,     1,    "bitrate",        "0",      0,    0},
      {true,  true,     1,    "broadcastmode",  "off",    0,    0},
      {true,  true,     1,    "arpcacheenable", "on",     0,    0},
      {0,0,0,0,0,0,0},
    };

// common
  const int PCAP_SNAPLEN = 0xFFFF;
  const int PCAP_PROMISC = 1;

#ifdef WIN32

  const std::string DEVICE_PREFIX = "\\Device\\NPF_";

  // win32 will buffer packets if PCAP_TIMEOUT is 0
  const int PCAP_TIMEOUT = 1;

#elif defined(__APPLE__)

  const int AddressType = AF_LINK;

  const std::string DEVICE_PREFIX = "";

  // mac will buffer packets if PCAP_TIMEOUT is 0
  const int PCAP_TIMEOUT = 1;

  struct sockaddr_ll_t {
   ACE_UINT8  sll_len;
   ACE_UINT8  sll_family;
   ACE_UINT16 sll_ifindex;
   ACE_UINT8  sll_type;
   ACE_UINT8  sll_nlen;
   ACE_UINT8  sll_alen;
   ACE_UINT8  sll_slen;
   ACE_UINT8  sll_addr[12];
  }__attribute__((packed));

#elif defined(linux)

  const int AddressType = AF_PACKET;

  const std::string DEVICE_PREFIX = "";

  const int PCAP_TIMEOUT = 0;

  struct sockaddr_ll_t {
   ACE_UINT16 sll_family;
   ACE_UINT16 sll_protocol;
   ACE_UINT32 sll_ifindex;
   ACE_UINT16 sll_type;
   ACE_UINT8  sll_pkttype;
   ACE_UINT8  sll_halen;
   ACE_UINT8  sll_addr[8];
  }__attribute__((packed));

#endif 
}


RawTransport::RawTransport(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService):
  EthernetTransport(id, pPlatformService),
  threadRead_(0),
  pPcapHandle_(NULL),
  pBitPool_(NULL)
{
  ACE_OS::memset(&macAddr_, 0x0, sizeof(macAddr_));

  configRequirements_ = EMANE::loadConfigurationRequirements(defs);

}


RawTransport::~RawTransport()
{
  // close pcap handle
  // Moves pcap_close call to the destructor
  // instead of the stop call because having it
  // there caused emanetransportd to hang on a 
  // ctrl-c 
   if (pPcapHandle_ != NULL) 
    {
      pcap_close(pPcapHandle_);

      pPcapHandle_ = NULL;
    }

   if(pBitPool_ != NULL)
    {
      delete pBitPool_;

      pBitPool_ = NULL;
    }
}


void RawTransport::initialize()
  throw(EMANE::InitializeException)
{ 
  // create bit pool
  pBitPool_ = new BitPool(pPlatformService_);
}



void RawTransport::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  Component::configure(items);
}



void RawTransport::start()
  throw(EMANE::StartException)
{
  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();

  bool bMacResolved = false;

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

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03hu RawTransport::%s %s: %ju",
                      id_, __func__, iter->first.c_str(), bitRate);
                }
              else if(iter->first == "device")
                {
                  sTargetDevice_ = iter->second.item_.getValue();

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03hu RawTransport::%s %s: %s", 
                      id_, __func__, iter->first.c_str(), sTargetDevice_.c_str());
                }
              else if(iter->first == "broadcastmode")
                {
                  bBroadcastMode_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue().c_str()).toBool();

                    pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s %s: %d", 
                        id_, __func__, iter->first.c_str(), bBroadcastMode_);
                }
              else if(iter->first == "arpcacheenable")
                {
                  bArpCacheMode_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue().c_str()).toBool();

                    pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s %s: %d", 
                        id_, __func__, iter->first.c_str(), bArpCacheMode_);
                }
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"RawTransport: Missing configuration item "<<iter->first<<std::ends;
              throw EMANE::StartException(ssDescription.str());
              
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"RawTransport: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }


  // pcap error buff
  char errbuf[PCAP_ERRBUF_SIZE];

  // pcap interface list
  struct pcap_if *iflist;

  // device name
  std::string sDeviceName = DEVICE_PREFIX;

  // get all available interfaces
  if (pcap_findalldevs (&iflist, errbuf) < 0 || iflist == NULL)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not get interface list "<< errbuf<<std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

#ifdef WIN32

  // we would prefer to use libpcap to obtain the interface info
  // but the mac address does not seem to be supported in win32
  // and we need the mac address to check for our own transmissions
  // since winpcap does not support pcap_setdirection
  
  // query the adapter list here and search based on the ip address in win32
  // instead of the complex adapter name like {11661990-8F30-46AD-A4F57D7AC1D2}

  // adpater buff
  IP_ADAPTER_INFO ai[256];

  // buff len
  ULONG buflen = sizeof(ai);

  // adapter list
  IP_ADAPTER_INFO* aip = &ai[0];

  // get all adpater info
  if(GetAdaptersInfo(aip, &buflen) == NO_ERROR) 
    {
      // each adpater
      while (aip && !bMacResolved) 
        {
          // ip address
          IP_ADDR_STRING *ip = &aip->IpAddressList;

          // each ip address
          while(ip && !bMacResolved) 
            {
              // address match
              if(sTargetDevice_ == ip->IpAddress.String) 
                {
                  // add adapter name to device prefix
                  sDeviceName += aip->AdapterName;

                  // save mac addr
                  ACE_OS::memcpy(macAddr_.bytes.buff, aip->Address, EMANEUtils::ETH_ALEN);

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s adapter %s, hw addr %s",
                                         id_, __func__,
                                         aip->AdapterName,
                                         ethaddr_to_string(&macAddr_).c_str());

                  // set resolved flag
                  bMacResolved = true;
               }
            // next address
            ip = ip->Next;
          }
          // next adapter 
          aip = aip->Next;
       }
    }
  else 
    {
      std::stringstream ssDescription;
      ssDescription<<"could not get adapters list "<< sTargetDevice_ << " " << errbuf<<std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

#elif defined(linux) || defined(__APPLE__)

  // each interface
  struct pcap_if *ifp = iflist;

  // walk through all available interfaces
  while (ifp)
    {
      // name match
      if(sTargetDevice_ == ifp->name) 
        {
          // add adapter name to device prefix
          sDeviceName += ifp->name;

          // address list
          pcap_addr_t *ap = ifp->addresses;

          // for each address
          while (ap && !bMacResolved) 
            {
              // mac addr
              if(ap->addr->sa_family == AddressType) 
                {
                  struct sockaddr_ll_t *s = (struct sockaddr_ll_t *) ap->addr;

#ifdef __APPLE__
                  // save mac addr
                  ACE_OS::memcpy(macAddr_.bytes.buff, &s->sll_addr[s->sll_nlen], EMANEUtils::ETH_ALEN);
#elif defined(linux)
                  ACE_OS::memcpy(macAddr_.bytes.buff, &s->sll_addr[0], EMANEUtils::ETH_ALEN);
#endif

                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s adapter %s, hw addr %s",
                                         id_, __func__, 
                                         ifp->name,
                                         ethaddr_to_string(&macAddr_).c_str());

                  // set resolved flag
                  bMacResolved = true;
                }
             // next addr
             ap = ap->next;
          }
       }
    // next interface
    ifp = ifp->next;
  }
#else
#error "unknown platfrom"
#endif


  // free interface list
  pcap_freealldevs(iflist);

  // mac address not resolved
  if(bMacResolved == false)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not resolve our mac address "<< sTargetDevice_ << " " << errbuf<<std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

  // open pcap handle
  if((pPcapHandle_ = pcap_open_live(sDeviceName.c_str(), PCAP_SNAPLEN, PCAP_PROMISC, PCAP_TIMEOUT, errbuf)) == NULL)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not open device "<< sDeviceName << " " << errbuf<<std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

  // set datalink type, this covers 10/100/1000
  if(pcap_set_datalink(pPcapHandle_, DLT_EN10MB) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set datalink type on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw EMANE::StartException(ssDescription.str());
    }

#ifndef WIN32

  // currently unsupported by winpcap
  if(pcap_setdirection(pPcapHandle_, PCAP_D_IN) < 0)
    {
       std::stringstream ssDescription;
       ssDescription<<"could not set direction on device "<< sDeviceName << " " << errbuf<<std::ends;
       throw EMANE::StartException(ssDescription.str());
    }

#endif

  // start pcap read thread
  EMANEUtils::spawn(*this, &RawTransport::readDevice, &threadRead_);
}



void RawTransport::stop()
  throw(EMANE::StopException)
{
  // cancel read thread
  if(threadRead_ != 0)
    {
       ACE_OS::thr_cancel(threadRead_);

       ACE_OS::thr_join(threadRead_,0,0);

       threadRead_ = 0;
    }
}


void RawTransport::destroy()
  throw()
{ }



void RawTransport::processUpstreamPacket(EMANE::UpstreamPacket & pkt, const EMANE::ControlMessage &)
{
  // frame sanity check
  if(verifyFrame(pkt.get(), pkt.length()) < 0)
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d RawTransport::%s ethernet frame error", id_, __func__);
    }

  // get packet info
  const EMANE::PacketInfo info = pkt.getPacketInfo();

  // ether header
  const EMANEUtils::EtherHeader * pEtherHeader = (const EMANEUtils::EtherHeader*) pkt.get();

  // update arp cache
  updateArpCache(pEtherHeader, info.source_);

#ifdef WIN32

  // lock mutex for writting
  mutex_.acquire_write();

  // add src mac addr to history set since we can not determine packet direction in win32
  upstreamHostSrcMacAddrHistorySet_.insert(*EMANEUtils::get_srcaddr(pEtherHeader));

  // unlock mutex
  mutex_.release();

#endif

  // send packet
  if(pcap_sendpacket(pPcapHandle_, (const ACE_UINT8*) pkt.get(), pkt.length()) < 0)
    {
      pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d RawTransport::%s pcap_sendpacket error %s", 
          id_, __func__, pcap_geterr(pPcapHandle_));
    }
  else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s src %hu, dst %hu, dscp %hhu, length %zu",
          id_, __func__, info.source_, info.destination_, info.dscp_, pkt.length());
#endif
      // drain the bit pool 
      const size_t sizePending = pBitPool_->get(pkt.length() * 8);
 
      // check for bitpool error
      if(sizePending != 0)
        {
          pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d RawTransport::%s bitpool request error %zd of %zd", 
              id_, __func__, sizePending, pkt.length() * 8);
        }
    }
}


void RawTransport::processUpstreamControl(const EMANE::ControlMessage &)
{
#ifdef VERBOSE_LOGGING
   pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::processUpstreamControl", id_);
#endif
}



void RawTransport::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{
#ifdef VERBOSE_LOGGING
   pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::processTimedEvent", id_);
#endif
}


void RawTransport::processEvent(const EMANE::EventId&, const EMANE::EventObjectState&)
{
#ifdef VERBOSE_LOGGING
   pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::processEvent", id_);
#endif
}


ACE_THR_FUNC_RETURN RawTransport::readDevice()
{
  const ACE_UINT8* buf;

  struct pcap_pkthdr *pcap_hdr;  

  int iPcapResult;

  while(1)
    {
      // get frame, blocks here
      iPcapResult = pcap_next_ex(pPcapHandle_, &pcap_hdr, &buf);

      // error
      if (iPcapResult < 0)
        {
          pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d RawTransport::%s pcap_next_ex error %s", 
              id_, __func__, pcap_geterr(pPcapHandle_));

          // break out of loop
          break;
        }
      // time out
      else if (iPcapResult == 0)
        {
          // continue
          continue;
        }
      // success
      else if(iPcapResult == 1)
         {
           // frame sanity check
           if(verifyFrame(buf, pcap_hdr->caplen) < 0)
             {
               pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d RawTransport::%s frame error", id_, __func__);
             }
           else
             {
               const EMANEUtils::EtherHeader *pEtherHeader = (const EMANEUtils::EtherHeader *) buf;

               // NEM dsstination
               EMANE::NEMId nemDestination;

               // pkt tos/qos
               ACE_UINT8 dscp;

               // get dst and dscp values
               if(parseFrame(pEtherHeader, nemDestination, dscp) < 0)
                {  
                  pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d RawTransport::%s frame parse error", id_, __func__);
                }
               else
                {
                  // check for our own outbound transmission
                  if(isOutbound(pEtherHeader))
                   {
                     // skip
                     continue;
                   }

#ifdef VERBOSE_LOGGING
                  pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s src %hu, dst %hu, dscp %hhu, length %u",
                                          id_, __func__, id_, nemDestination, dscp, pcap_hdr->caplen);
#endif

                  // create downstream packet with packet info
                  EMANE::DownstreamPacket pkt(EMANE::PacketInfo (id_, nemDestination, dscp), buf, pcap_hdr->caplen);

                  // send to downstream transport
                  sendDownstreamPacket(pkt);

                  // drain the bit pool 
                  const size_t sizePending = pBitPool_->get(pcap_hdr->caplen * 8);
 
                  // check for bitpool error
                  if(sizePending != 0)
                   {
                     pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d RawTransport::%s bitpool request error %zd of %u", 
                                             id_, __func__, sizePending, pcap_hdr->caplen * 8);
                   }
               }
            }
         }
     }

     return (ACE_THR_FUNC_RETURN) 0;
}


int RawTransport::isOutbound(const EMANEUtils::EtherHeader *pEtherHeader)
{
#ifdef WIN32

   // check mac src address since we can not determine packet direction in win32

   // lock mutex for reading
   mutex_.acquire_read();

   // packet is from self
   if(ACE_OS::memcmp(EMANEUtils::get_srcaddr(pEtherHeader), &macAddr_, EMANEUtils::ETH_ALEN) == 0)
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s src %s is us",
                      id_, __func__,
                      ethaddr_to_string(EMANEUtils::get_srcaddr(pEtherHeader)).c_str());
#endif

      return 1;
    }

   // check the src mac addr with the upstream src mac addr set 
   EthAddrSetConstIter iter = upstreamHostSrcMacAddrHistorySet_.find(pEtherHeader->src);

   // this source was seen on the upstream path
   if(iter != upstreamHostSrcMacAddrHistorySet_.end())
    {
#ifdef VERBOSE_LOGGING
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d RawTransport::%s src %s is downstream",
                      id_, __func__,
                      ethaddr_to_string(&pEtherHeader->src).c_str());
#endif

      return 1;
    }

   // unlock mutex
   mutex_.release();

#else

   ACE_UNUSED_ARG(pEtherHeader);

#endif

   return 0;
}


DECLARE_TRANSPORT(RawTransport);
