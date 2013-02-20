/*
 * Copyright (c) 2009-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "ethernettransport.h"
#include "emane/emanecomponenttypes.h"

#include <ace/Guard_T.h>


EMANE::EthernetTransport::EthernetTransport(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService):
  Transport(id, pPlatformService),
  bBroadcastMode_(false),
  bArpCacheMode_(true)
{ }


EMANE::EthernetTransport::~EthernetTransport()
{ }


int EMANE::EthernetTransport::verifyFrame(const void * buf, size_t len)
{
   // check min header len
   if(len < EMANEUtils::ETH_HEADER_LEN)
     {
        pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d EthernetTransport::%s len %zd < min eth header len %d", 
                               id_, __func__, len, EMANEUtils::ETH_HEADER_LEN);

        // error
        return -1;
     }
   else
     {
       // eth header
       const EMANEUtils::EtherHeader *pEthHeader = (EMANEUtils::EtherHeader *) buf;

       // eth protocol
       const ACE_UINT16 u16ethProtocol = EMANEUtils::get_protocol(pEthHeader);

       // reduce length by eth hdr len
       len -= EMANEUtils::ETH_HEADER_LEN;

       switch(u16ethProtocol)
        {
          // eth ipv4
          case EMANEUtils::ETH_P_IPV4: 
          {
            // check min len
            if(len < EMANEUtils::IPV4_HEADER_LEN)
             {
               pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d EthernetTransport::%s ipv4, len %zu < min len %d", 
                                      id_, __func__, len, EMANEUtils::IPV4_HEADER_LEN);

               // error
               return -1;
             }
            else
             {
               // success
               return 0;
             }
          } 

         // eth ipv6
         case EMANEUtils::ETH_P_IPV6: 
          {
            // check min len
            if(len < EMANEUtils::IPV6_HEADER_LEN)
             {
                pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d EthernetTransport::%s ipv6, len %zu < min len %d", 
                                       id_, __func__, len, EMANEUtils::IPV6_HEADER_LEN);

                // error
                return -1;
             }
           else
             { 
               // success
               return 0;
             }
          }

         // eth arp
         case EMANEUtils::ETH_P_ARP: 
          {
            // check min len
            if(len < EMANEUtils::ETHARP_HEADER_LEN)
             {
               pPlatformService_->log(EMANE::ERROR_LEVEL, "TRANSPORTI %03d EthernetTransport::%s arp, len %zu < len %d", 
                                      id_, __func__, len, EMANEUtils::ETHARP_HEADER_LEN);

               // error
               return -1;
             }
            else
             {
               // success
               return 0;
             }
          }

        // unknown protocol
        default:
          pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d EthernetTransport::%s allow unknown protocol %02X", 
                                 id_, __func__, u16ethProtocol);

          // but not an error
          return 1;
      }
   }
}



int EMANE::EthernetTransport::parseFrame(const EMANEUtils::EtherHeader *pEthHeader, EMANE::NEMId & rNemDestination, ACE_UINT8 & rDspc)
{
   // eth protocol
   const ACE_UINT16 u16ethProtocol = EMANEUtils::get_protocol(pEthHeader);

   switch(u16ethProtocol)
   {
     // eth ipv4
     case EMANEUtils::ETH_P_IPV4: 
       {
         // ipv4 header
         const EMANEUtils::Ip4Header *pIpHeader = (EMANEUtils::Ip4Header*) ((EMANEUtils::EtherHeader*) pEthHeader + 1);

         // broadcast always mode
         if(bBroadcastMode_) 
           {
             rNemDestination = EMANE::NEM_BROADCAST_MAC_ADDRESS;
           }
         // check arp cache
         else if (bArpCacheMode_)
           {
             rNemDestination = lookupArpCache(&pEthHeader->dst);
           }
         // use ether dst
         else
           {
             rNemDestination = EMANEUtils::ethaddr4_to_id(&pEthHeader->dst);
           }

         // set the dscp based on ip header
         rDspc = EMANEUtils::get_dscp(pIpHeader);

         // success
         return 0;
       } 

     // eth ipv6
     case EMANEUtils::ETH_P_IPV6: 
       {
         // ipv6 header
         const EMANEUtils::Ip6Header *pIpHeader = (EMANEUtils::Ip6Header*) ((EMANEUtils::EtherHeader*) pEthHeader + 1);

         // broadcast always mode
         if(bBroadcastMode_) 
           {
             rNemDestination = EMANE::NEM_BROADCAST_MAC_ADDRESS;
           }
         // check arp cache
         else if (bArpCacheMode_)
           {
             rNemDestination = lookupArpCache(&pEthHeader->dst);
           }
         // use ether dst
         else
           {
             rNemDestination = EMANEUtils::ethaddr6_to_id(&pEthHeader->dst);
           }

         // set the dscp based on ip header
         rDspc = EMANEUtils::get_dscp(pIpHeader);

         // success
         return 0;
       }

     // eth arp
     case EMANEUtils::ETH_P_ARP: 
       {
         // broadcast always mode
         if(bBroadcastMode_) 
           {
             rNemDestination = EMANE::NEM_BROADCAST_MAC_ADDRESS;
           }
         // check arp cache
         else if (bArpCacheMode_)
           {
             rNemDestination = lookupArpCache(&pEthHeader->dst);
           }
         // use ether dst
         else 
           {
             rNemDestination = EMANEUtils::ethaddr4_to_id(&pEthHeader->dst);
           }

         // set dscp to 0 for all arp types
         rDspc = 0;

         // success
         return 0;
       }

     // unknown protocol
     default:
       pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d EthernetTransport::%s allow unknown protocol %02X", 
                              id_, __func__, u16ethProtocol);

       // use broadcast mac addr
       rNemDestination = EMANE::NEM_BROADCAST_MAC_ADDRESS;

       // set dscp to 0
       rDspc = 0;

       // but not an error
       return 1;
   }
}



void EMANE::EthernetTransport::updateArpCache(const EMANEUtils::EtherHeader *pEthHeader, EMANE::NEMId nemId)
{
   // not needed in broadcast mode
   if(bBroadcastMode_)
    {
      return;
    }
   // not needed if arp cache disabled
   else if (! bArpCacheMode_)
    {
      return;
    }
   else
    {
      // lock mutex
      ACE_Guard<ACE_Thread_Mutex> m(mutex_);

      // eth protocol
      const ACE_UINT16 u16ethProtocol = EMANEUtils::get_protocol(pEthHeader);

      switch(u16ethProtocol)
      {
        // eth arp
        case EMANEUtils::ETH_P_ARP: 
          {
            // ether arp header
            const EMANEUtils::EtherArpHeader *pEtherArpHeader = (EMANEUtils::EtherArpHeader*) ((EMANEUtils::EtherHeader*) pEthHeader + 1);

            // arp option
            const ACE_UINT16 u16code = EMANEUtils::get_code(pEtherArpHeader);

            // arp type reply or request
            if((u16code == EMANEUtils::ETH_ARPOP_REPLY) || (u16code == EMANEUtils::ETH_ARPOP_REQUEST)) 
             {
               addEntry(*EMANEUtils::get_srchwaddr(pEtherArpHeader), nemId);
             }
          }
        break;

        case EMANEUtils::ETH_P_IPV6:
          {
            //ipv6 header
            const EMANEUtils::Ip6Header *pIp6Header = (EMANEUtils::Ip6Header*) ((EMANEUtils::EtherHeader*) pEthHeader + 1);
            
            // check for icmpv6 
            if (pIp6Header->u8Ipv6next == EMANEUtils::IPV6_P_ICMP) 
             {
               // icmpv6 header
               const EMANEUtils::IP6ICMPHeader *pICMP6Header = (EMANEUtils::IP6ICMPHeader*) ((EMANEUtils::Ip6Header*) pIp6Header + 1);
              
               // icmp type
               const ACE_UINT8 icmpv6Type = pICMP6Header->u8Type;
              
               // icmpv6 neighbor solicitation or advertisement
               if ((icmpv6Type == EMANEUtils::IP6_ICMP_NEIGH_SOLICIT) || (icmpv6Type == EMANEUtils::IP6_ICMP_NEIGH_ADVERT)) 
                {
                  addEntry(pEthHeader->src, nemId);
                }
             }
          }
        break;
      }
   }
}

void EMANE::EthernetTransport::addEntry(const EMANEUtils::EtherAddr& addr, EMANE::NEMId nemId) 
{
  const EthAddrMapIter iter = macCache_.find(addr);
  
  // new entry
  if(iter == macCache_.end())
    {
      macCache_.insert(std::make_pair(addr, nemId));
      
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d ARPCache::%s added cache entry %s to nem %hu",
                             id_, __func__, ethaddr_to_string(&addr).c_str(), nemId);
#endif
    }
  else
    {
      // entry found but different nem
      if(iter->second != nemId)
       {
#ifdef VERBOSE_LOGGING
         pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d ARPCache::%s updated cache entry %s from nem %hu to nem %hu",
                                id_, __func__, ethaddr_to_string(&addr).c_str(), iter->second, nemId);
#endif
         // updated nem id
         iter->second = nemId;
       }
    }
}


EMANE::NEMId EMANE::EthernetTransport::lookupArpCache(const EMANEUtils::EtherAddr *pEtherAddr)
{
   // lock mutex
   ACE_Guard<ACE_Thread_Mutex> m(mutex_);

   const EthAddrMapIter iter = macCache_.find(*pEtherAddr);

   // entry not found, most likely broadcast
   if(iter == macCache_.end())
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d EthernetTransport::%s no nem found for %s, using broadcast mac address", 
          id_, __func__, EMANEUtils::ethaddr_to_string(pEtherAddr).c_str());
#endif

      return EMANE::NEM_BROADCAST_MAC_ADDRESS;
    }
   else
    {
#ifdef VERBOSE_LOGGING
      pPlatformService_->log(EMANE::DEBUG_LEVEL, "TRANSPORTI %03d EthernetTransport::%s nem %hu found for %s, using %hu", 
          id_, __func__, iter->second, EMANEUtils::ethaddr_to_string(pEtherAddr).c_str(), iter->second);
#endif

      return iter->second;
    }
}

