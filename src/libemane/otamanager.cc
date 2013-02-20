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

#include "otamanager.h"
#include "otauser.h"
#include "otamanagermessage.h"
#include "logservice.h"

#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/recvcancelable.h"

#include "emanecontrolmessages/otatransmittersmessage.h"

#include <sstream>

#include <ace/Guard_T.h>
#include <ace/OS_NS_string.h>

EMANE::OTAManager::OTAManager():
  thread_(0),
  mcast_(ACE_SOCK_Dgram_Mcast::OPT_BINDADDR_NO),
  bOpen_(false)
{}
    
EMANE::OTAManager::~OTAManager()
{
  if(bOpen_)
    {
      ACE_OS::thr_cancel(thread_);

      ACE_OS::thr_join(thread_,0,0);
    }
}
    
void EMANE::OTAManager::sendOTAPacket(NEMId id, DownstreamPacket pkt, ControlMessage msg)
{
  ACE_Read_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);
 
  // get the pkt info 
  const PacketInfo pktInfo = pkt.getPacketInfo();

  // combine the pkt into a contiguous block of data
  pkt.combine();

  // set of optional additional transmitters (AT)
  EMANE::NEMIdSet ats;
  
  // check if control is an ota transmitter massage
  if(msg.getMajorIdentifier () == EMANE_OTA_MAJOR_ID && 
     msg.getMinorIdentifier () == EMANE_OTA_TRANSMITTERS_MINOR_ID)
   {
     // create an AT message
     OTATransmittersMessage ctrl(msg);

     // get the additional transmitters
     ctrl.get(ats);
   }

  // bounce a copy of the pkt back up to our local NEM stack(s)
  for(NEMUserMap::const_iterator iter = nemUserMap_.begin(); iter != nemUserMap_.end(); ++iter)
    {
      if(iter->first == id)
        {
          // skip our own transmisstion
        }
      else if(ats.count(iter->first) > 0)
        {
          // skip NEM(s) in the additional transmitter set (ATS)
        }
      else
        {
          /*
           * each NEM needs is own copy of the packet,
           * you can not share between NEM(s), so create and send an Upstream pkt for each NEM
           */
          iter->second->processOTAPacket(UpstreamPacket(pktInfo, pkt.get(), pkt.length()), EMPTY_CONTROL_MESSAGE);
        }
    }

  // send the packet to additional OTAManagers using OTA multicast transport
  if(bOpen_)
    {
      // create an ota data message to carry the packet_info, and variable ctrl data len only
      // total message length with data payload is defined below
      OTADataMessage otaDataMessage(pktInfo.source_, 
                                    pktInfo.destination_, 
                                    pktInfo.dscp_, 
                                    msg.length());

      // create an ota message to describe the entire message (OTA_HEADER + PKT_INFO + CTRLID + CTRL + PKT)
      OTAMessage otaMessage(OTA_DATA_MESSAGE,                // ID
                            sizeof(OTAMessage) +             // fixed ota message header
                            sizeof(OTADataMessage) +         // fixed ota data message header
                            sizeof(OTAControlMessageId) +    // fixed control id header
                            msg.length() +                   // variable control msg length
                            pkt.length());                   // variable pkt data length

      // get the control msg major/minor in net byte order
      OTAControlMessageId controlMessageId(ACE_HTONL(msg.getMajorIdentifier()), ACE_HTONL(msg.getMinorIdentifier()));

      // 5 pieces to combine
      const int iovSize = 5;

      // the message blocks are held here
      iovec iov[iovSize];

      // first is the ota header 
      iov[0].iov_base = reinterpret_cast<char *>(&otaMessage);
      iov[0].iov_len  = sizeof(otaMessage);
    
      // second is the ota data msg
      iov[1].iov_base = reinterpret_cast<char *>(&otaDataMessage);
      iov[1].iov_len  = sizeof(otaDataMessage);

      // third is the ctrl major/minor id
      iov[2].iov_base = const_cast<char *>(reinterpret_cast<const char *>(&controlMessageId));
      iov[2].iov_len  = sizeof(controlMessageId);
 
      // fourth is the ctrl msg body
      iov[3].iov_base = const_cast<char *>(reinterpret_cast<const char *>(msg.get()));
      iov[3].iov_len  = msg.length();
     
      // last is the pkt body
      iov[4].iov_base = const_cast<char *>(reinterpret_cast<const char *>(pkt.get()));
      iov[4].iov_len  = pkt.length();
          
      // convert to network byte order 
      OTAMessageToNet(&otaMessage);

      // convert to network byte order 
      OTADataMessageToNet(&otaDataMessage);

      // gather and send
      if(mcast_.send(iov, iovSize) == -1)
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                               "OTAManager sendOTAPacket unable to send src:%hu dst:%hu\n",
                                               pktInfo.source_,
                                               pktInfo.destination_);
        }
    }
}
    
void EMANE::OTAManager::registerOTAUser(NEMId id, OTAUser * pOTAUser)
  throw(EMANE::OTAException)
{
  ACE_Write_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);

  std::pair<NEMUserMap::iterator, bool> ret;
  
  if(nemUserMap_.insert(std::make_pair(id,pOTAUser)).second == false)
    {
      std::stringstream ssDescription;
      ssDescription<<"attempted to register duplicate user with id "<<id<<std::ends;
      throw OTAException(ssDescription.str());
    }
}

void EMANE::OTAManager::unregisterOTAUser(NEMId id)
  throw(EMANE::OTAException)
{
  ACE_Write_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);

  if(nemUserMap_.erase(id) == 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"attempted to unregister unknown user with id "<<id<<std::ends;
      throw OTAException(ssDescription.str());
    }
}

void EMANE::OTAManager::open(const ACE_INET_Addr & otaGroupAddress, const ACE_TCHAR* otaManagerDevice)
{
  otaGroupAddress_ =  otaGroupAddress;
  ACE_hthread_t threadHandle;
  int priority = 0;
  int policy = 0;
  
  if(mcast_.open(otaGroupAddress,otaManagerDevice) == -1)
    {
      std::stringstream sstream;
          
      sstream<<"Platform OTA Manager: Unable to open OTA Manager socket: '"
             <<otaGroupAddress.get_host_addr()
             <<":"
             <<otaGroupAddress.get_port_number()
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * No Multicast device specified and routing table nondeterministic"
             <<std::endl
             <<"   (no multicast route and no default route)."
             <<std::endl
             <<" * Multicast device "
             <<otaManagerDevice
             <<" does not exist or is not up."
             <<std::endl
             <<std::ends;
      
      throw OTAException(sstream.str());
    }
  
  if(mcast_.join(otaGroupAddress,1,otaManagerDevice) == -1)
    {
      std::stringstream sstream;
      
      sstream<<"Platform OTA Manager: Unable to join OTA Manager group: '"
             <<otaGroupAddress.get_host_addr()
             <<":"
             <<otaGroupAddress.get_port_number()
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<otaGroupAddress.get_host_addr()
             <<" is not a multicast address."
             <<std::endl
             <<std::ends;
      
      throw OTAException(sstream.str());
    }

  if(otaGroupAddress.get_type() == AF_INET) {
      if(mcast_.set_option(IP_MULTICAST_LOOP,0) == -1)
        {
          std::stringstream ssDescription;
          ssDescription<<"unable to unset OTA Manager group IP_MULTICAST_LOOP errno "<< ACE_OS::strerror (errno) << std::ends;
          throw OTAException(ssDescription.str()); 
        }
    }
  else if(otaGroupAddress.get_type() == AF_INET6) {
      int loop = 0;
      if(mcast_.ACE_SOCK::set_option(IPPROTO_IPV6,IPV6_MULTICAST_LOOP,&loop,sizeof(loop)) == -1)
        {
          std::stringstream ssDescription;
          ssDescription<<"unable to unset OTA Manager group IPV6_MULTICAST_LOOP errno " << ACE_OS::strerror (errno) << std::ends;
          throw OTAException(ssDescription.str()); 
        }
    }

  
  EMANEUtils::spawn(*this,&EMANE::OTAManager::processOTAMessage,&thread_,&threadHandle);

  ACE_OS::thr_getprio(threadHandle,priority,policy);

  if(policy == ACE_SCHED_RR)
  { 
    int retval = ACE_OS::thr_setprio(threadHandle,ACE_THR_PRI_FIFO_DEF + 1,ACE_SCHED_RR);

    if(retval != 0)
    {
      LogServiceSingleton::instance()->log(ERROR_LEVEL,"OTAManager::open: Unable to set Real Time Priority");
    }
  }

  bOpen_ = true;
}



ACE_THR_FUNC_RETURN EMANE::OTAManager::processOTAMessage()
{
  unsigned char buf[65536];

  ssize_t len = 0;

  while(1)
    {
      if((len = EMANEUtils::recvCancelable(mcast_, buf, sizeof(buf), addr_)) > 0)
        {
          // ota message len sanity check
          if(static_cast<size_t>(len) >= sizeof(OTAMessage))
            {
              unsigned char * p = buf;

              // get the ota message from the head of the buffer
              OTAMessage * pOTAMessage = reinterpret_cast<OTAMessage *>(p);

              // bump pos/offset
              p += sizeof(OTAMessage);
             
              // convert to host byte order 
              OTAMessageToHost(pOTAMessage);
              
              switch(pOTAMessage->u16Id_)
                {
                case OTA_DATA_MESSAGE:
                  // check that the rx msg length matches the ota header total length
                  if(static_cast<size_t>(len) == pOTAMessage->u16Length_)
                    {
                      ACE_Read_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);

                      // get the ota data message next
                      OTADataMessage * pOTADataMessage = reinterpret_cast<OTADataMessage *>(p);
                     
                      // bump pos/offset
                      p += sizeof(OTADataMessage);

                       // convert to host byte order 
                      OTADataMessageToHost(pOTADataMessage);
                     
                      // create packet info from the ota data message 
                      PacketInfo pktInfo(pOTADataMessage->u16Src_,
                                         pOTADataMessage->u16Dst_,
                                         pOTADataMessage->u16DSCP_);
                  
                      // get the control message id from the ota data
                      OTAControlMessageId * pControlMessageId = reinterpret_cast<OTAControlMessageId *>(p);

                      // bump pos/offset
                      p += sizeof(OTAControlMessageId);

                      // set of optional additional transmitters (AT)
                      EMANE::NEMIdSet ats;
  
                     // check if control is a ota transmitter massage
                     if(ACE_NTOHL(pControlMessageId->i32Major_) == EMANE_OTA_MAJOR_ID && 
                        ACE_NTOHL(pControlMessageId->i32Major_) == EMANE_OTA_TRANSMITTERS_MINOR_ID)
                      {
                        // create control message from the ota data msg after the control msg id
                        ControlMessage msg (ACE_NTOHL(pControlMessageId->i32Major_),   // ctrl msg major
                                            ACE_NTOHL(pControlMessageId->i32Minor_),   // ctrl msg minor
                                            reinterpret_cast<void *>(p),               // ctrl msg body
                                            pOTADataMessage->u16DataLen_);             // ctrl msg length


                        // create an AT message
                        OTATransmittersMessage ctrl(msg);

                        // get the additional transmitters
                        ctrl.get(ats);
                      }

                      // bump pos/offset
                      p += pOTADataMessage->u16DataLen_;

                      // for each local NEM stack
                      for(NEMUserMap::iterator iter = nemUserMap_.begin(); iter != nemUserMap_.end(); ++iter)
                        {
                          // only send pkt up to NEM(s) NOT in the ATS
                          if(ats.count(iter->first) == 0)
                           {
                             iter->second->processOTAPacket(UpstreamPacket(pktInfo, p, len - (p - buf)), EMPTY_CONTROL_MESSAGE);
                           }
                        }
                    }
                  else
                    {
                      LogServiceSingleton::instance()->log(ERROR_LEVEL,"OTAManager Packet Received length mismatch not OTADataMessage");
                    }
                  break;

                default:
                  LogServiceSingleton::instance()->log(ERROR_LEVEL,"OTAManager Packet Received unknown message: %hu",pOTAMessage->u16Id_);
                  break;
                }
            }
          else
            {
              LogServiceSingleton::instance()->log(ERROR_LEVEL,"OTAManager Packet Received length %zd, less than OTAMessage min size %zu",
                                                   len, sizeof(OTAMessage));
            }
        }
      else
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"OTAManager Packet Received error");
          
          // break out of while(1)
          break;
        }
    }
  
  return 0;
}
