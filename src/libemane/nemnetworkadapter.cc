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

#include "nemnetworkadapter.h"
#include "netadaptermessage.h"
#include "logservice.h"

#include "emane/emanedownstreamtransport.h"

#include "emaneutils/spawnmemberfunc.h"
#include "emaneutils/netutils.h"
#include "emaneutils/recvcancelable.h"

#include <sstream>
#include <iomanip>

#include <ace/Guard_T.h>

EMANE::NEMNetworkAdapter::NEMNetworkAdapter(NEMId id):
  id_(id),
  thread_(0),
  bOpen_(false){}

EMANE::NEMNetworkAdapter::~NEMNetworkAdapter(){}

void EMANE::NEMNetworkAdapter::processUpstreamPacket(UpstreamPacket & pkt, 
                                                     const ControlMessage & ctrl)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);
  
  if(bOpen_)
    {
      ssize_t len = 0;

      iovec iov[4];
      
      NetAdapterDataMessage dataMessage;
      
      memset(&dataMessage,0,sizeof(dataMessage));
      
      PacketInfo info  = pkt.getPacketInfo();
      dataMessage.u16Src_ = info.source_;
      dataMessage.u16Dst_ = info.destination_;
      dataMessage.u8Dscp_ = info.dscp_;
      dataMessage.u16DataLen_ = pkt.length();
      
      dataMessage.u16CtrlLen_ = ctrl.length();
      dataMessage.u32CtrlMajorId_ = ctrl.getMajorIdentifier();
      dataMessage.u32CtrlMinorId_ = ctrl.getMinorIdentifier();
      
      NetAdapterDataMessageToNet(&dataMessage);
      
      NetAdapterHeader header;
      
      memset(&header,0,sizeof(header));
      
      header.u16Id_ =  NETADAPTER_DATA_MSG;
      header.u16Length_ = sizeof(header) +  sizeof(dataMessage) + pkt.length() + ctrl.length();
      
      NetAdapterHeaderToNet(&header);
      
      iov[0].iov_base = reinterpret_cast<char *>(&header);
      iov[0].iov_len  = sizeof(header);
      
      iov[1].iov_base = reinterpret_cast<char *>(&dataMessage);
      iov[1].iov_len  = sizeof(dataMessage);
      
      iov[2].iov_base = reinterpret_cast<char *>(const_cast<void *>(pkt.get()));
      iov[2].iov_len  = pkt.length();
      
      iov[3].iov_base = reinterpret_cast<char *>(const_cast<void *>(ctrl.get()));
      iov[3].iov_len  = ctrl.length();
      
      if((len = udp_.send(iov,4,remoteAddress_)) == -1)
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"NEM %03hu error on message send (expected:%hu actual:%zd)",
                                 id_, 
                                 header.u16Length_,
                                 len);
        }
    }
}

void EMANE::NEMNetworkAdapter::processUpstreamControl(const ControlMessage & ctrl)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  if(bOpen_)
    {
      ssize_t len = 0;
      
      iovec iov[3];

      NetAdapterControlMessage controlMessage;
    
      memset(&controlMessage,0,sizeof(controlMessage));
      
      controlMessage.u16Len_     = ctrl.length();
      controlMessage.u32MajorId_ = ctrl.getMajorIdentifier();
      controlMessage.u32MinorId_ = ctrl.getMinorIdentifier();
      
      NetAdapterControlMessageToNet(&controlMessage);
      
      NetAdapterHeader header;
      
      memset(&header,0,sizeof(header));
      
      header.u16Id_ =  NETADAPTER_CTRL_MSG;
      header.u16Length_ = sizeof(header) +  sizeof(controlMessage) + ctrl.length();
      
      NetAdapterHeaderToNet(&header);
      
      iov[0].iov_base = reinterpret_cast<char *>(&header);
      iov[0].iov_len  = sizeof(header);
      
      iov[1].iov_base = reinterpret_cast<char *>(&controlMessage);
      iov[1].iov_len  = sizeof(controlMessage);
      
      iov[2].iov_base = reinterpret_cast<char *>(const_cast<void *>(ctrl.get()));
      iov[2].iov_len  = ctrl.length();
      
      if((len = udp_.send(iov,3,remoteAddress_)) == -1)
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"NEM %03hu error on control message send (expected:%hu actual:%zd)",
                                 id_, 
                                 header.u16Length_,
                                 len);
        }
    }
  else
    {
      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                             "NEM %03hu NEMNetworkAdapter error on control message send socket not ready",
                             id_);
    }
}

void EMANE::NEMNetworkAdapter::open(const ACE_INET_Addr & localAddress, const ACE_INET_Addr & remoteAddress)
  throw(NetworkAdapterException)
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  localAddress_   = localAddress;
  remoteAddress_  = remoteAddress;
  
  if(udp_.open(localAddress_,ACE_PROTOCOL_FAMILY_INET,0,1) == -1)
    {
      std::stringstream sstream;

      sstream<<"NEM "
             <<std::setw(3)
             <<std::setfill('0')
             <<id_
             <<": Unable to open receive socket to transport: '"
             <<localAddress_.get_host_name()
             <<":"
             <<localAddress_.get_port_number()
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<localAddress_.get_host_name()
             <<" is not this host."
             <<std::endl
             <<std::ends;

      throw NetworkAdapterException(sstream.str());
    }

  EMANEUtils::spawn(*this,& EMANE::NEMNetworkAdapter::processNetworkMessage,&thread_);

  bOpen_ = true;
}

void EMANE::NEMNetworkAdapter::close()
{
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  if(thread_)
    {
      ACE_OS::thr_cancel(thread_);

      ACE_OS::thr_join(thread_,0,0);
    }
  
  udp_.close();

  bOpen_ = false;
}

ACE_THR_FUNC_RETURN EMANE::NEMNetworkAdapter::processNetworkMessage()
{
  unsigned char buf[65536];
  ssize_t len = 0;
  NEMId dstNemId;
  
  while(1)
    {
      dstNemId = 0;

      memset(&buf,0,sizeof(buf));
 
      if((len =  EMANEUtils::recvCancelable(udp_,buf,sizeof(buf),addr_)) > 0)
        {
          if(static_cast<size_t>(len) >= sizeof(NetAdapterHeader))
            {
              NetAdapterHeader * pHeader = reinterpret_cast<NetAdapterHeader *>(buf);
          
              NetAdapterHeaderToHost(pHeader);
              
              if(static_cast<size_t>(len) == pHeader->u16Length_)
                {
                  switch(pHeader->u16Id_)
                    {
                    case NETADAPTER_DATA_MSG:
                      {
                        NetAdapterDataMessage * pMsg =  reinterpret_cast<NetAdapterDataMessage *>(pHeader->data_);
                        
                        NetAdapterDataMessageToHost(pMsg);
                        
                        len -= sizeof(NetAdapterHeader);
                        
                        if(static_cast<size_t>(len) == pMsg->u16DataLen_ + sizeof(NetAdapterDataMessage) + pMsg->u16CtrlLen_)
                          {
                            /*
                             * Determine the NEM id to address mapping.  If the 
                             * destination address is broadcast address type of broadcast/multicast change
                             * it to be the all 1s NEM BROADCAST ADDR address.
                             */
                            if(pMsg->u16Dst_ == ACE_HTONS(NETADAPTER_BROADCAST_ADDRESS))
                              {
                                dstNemId = NEM_BROADCAST_MAC_ADDRESS;
                              }
                            else
                              {
                                dstNemId = pMsg->u16Dst_;
                              }
                            
                            if(dstNemId)
                              {
                                PacketInfo pinfo(pMsg->u16Src_, dstNemId, pMsg->u8Dscp_);
                                
                                DownstreamPacket pkt = DownstreamPacket(pinfo,
                                                                        pMsg->data_,
                                                                        pMsg->u16DataLen_);
                                
                                ControlMessage ctrl = ControlMessage(pMsg->u32CtrlMajorId_,
                                                                     pMsg->u32CtrlMinorId_,
                                                                     pMsg->data_ + pMsg->u16DataLen_,
                                                                     pMsg->u16CtrlLen_);
                                sendDownstreamPacket(pkt,ctrl);
                              }
                            else
                              {
                                LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                                       "NEM %03hu NEMNetworkAdapter Unkown NEM Id for %hu",
                                                       id_,
                                                       ACE_NTOHS(pMsg->u16Dst_));
                              }
                          }
                        else
                          {
                            LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                                   "NEM %03hu NEMNetworkAdapter Size mismatch expected %zd got %zd",
                                                   id_,
                                                   pMsg->u16DataLen_ + sizeof(NetAdapterDataMessage) + pMsg->u16CtrlLen_,
                                                   len);
                          }
                      }
                      break;
                      
                    case NETADAPTER_CTRL_MSG:
                      {
                        NetAdapterControlMessage * pMsg =  reinterpret_cast<NetAdapterControlMessage *>(pHeader->data_);
                        
                        NetAdapterControlMessageToHost(pMsg);
                        
                        len -= sizeof(NetAdapterHeader);
                        
                        if(static_cast<size_t>(len) == sizeof(NetAdapterControlMessage) + pMsg->u16Len_)
                          {
                            ControlMessage ctrl = ControlMessage(pMsg->u32MajorId_,
                                                                 pMsg->u32MinorId_,
                                                                 pMsg->data_,
                                                                 pMsg->u16Len_);
                            sendDownstreamControl(ctrl);
                          }
                        else
                          {
                            LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                                   "NEM %03hu NEMNetworkAdapter Size mismatch expected %zd got %zd",
                                                   id_,
                                                   sizeof(NetAdapterControlMessage) + pMsg->u16Len_,
                                                   len);
                          }
                      }
                      break;
                      
                    default:
                      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                             "NEM %03hu NEMNetworkAdapter Received unknown message type: %hu",
                                              id_,
                                              pHeader->u16Id_);
                      break;
                    }
                }
              else
                {
                  LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                         "NEM %03hu NEMNetworkAdapter Message mismatch expected %hu got %zd",
                                         id_,
                                         pHeader->u16Length_,
                                         len);
                }
            }
          else
            {
              LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                     "NEM %03hu NEMNetworkAdapter Message Header mismatch expected %zu got %zd",
                                     id_,
                                     sizeof(NetAdapterHeader),
                                     len);
            }
          
        }
      else
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,"NEM %03hu NEMNetworkAdapter recevieve error",id_);
          break;
        }
    }

  return 0;
}
