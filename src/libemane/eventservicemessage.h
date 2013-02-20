/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTSERVICEMESSAGE_HEADER_
#define EMANEEVENTSERVICEMESSAGE_HEADER_

#include <ace/Basic_Types.h>

namespace EMANE
{
  const ACE_UINT16 EVENTSERVICE_CLIENT_MSG = 1;
  const ACE_UINT16 EVENTSERVICE_SERVER_MSG = 2;

  /**
   * @class EventServiceHeader
   *
   * @brief Event Service message header
   */
  struct EventServiceHeader
  {
    ACE_UINT16 u16Id_;     /**< Event id */
    ACE_UINT16 u16Length_; /**< Total message length in bytes */
    ACE_UINT8  data_[0];   /**< Pointer to message payload */
  } __attribute__((packed));
  
  /**
   * Convert a EventServiceHeader to host byte order
   * 
   * @param pMsg Message header reference
   *
   * @return Message header reference.  Same as @a pMsg.
   */
  inline EventServiceHeader * eventServiceHeaderToHost(EventServiceHeader * pMsg)
  {
    pMsg->u16Id_     =  ACE_NTOHS(pMsg->u16Id_);
    pMsg->u16Length_ =  ACE_NTOHS(pMsg->u16Length_);
    return pMsg;
  }

  /**
   * Convert a EventServiceHeader to network byte order
   * 
   * @param pMsg Message header reference
   *
   * @return Message header reference.  Same as @a pMsg.
   */
  inline EventServiceHeader * eventServiceHeaderToNet(EventServiceHeader * pMsg)
  {
    pMsg->u16Id_     =  ACE_HTONS(pMsg->u16Id_);
    pMsg->u16Length_ =  ACE_HTONS(pMsg->u16Length_);
    return pMsg;
  }

  /**
   * @class EventServiceClientMessage
   *
   * @brief Client specific event message
   */
  struct EventServiceClientMessage
  {
    ACE_UINT16 u16PlatformId_; /**< Platform id or @c 0 for all platforms */
    ACE_UINT16 u16NEMId_;      /**< NEM id or @c 0 for all NEMs */
    ACE_UINT16 u16LayerType_;  /**< Layer destination.  See ComponentType. */
    ACE_UINT16 u16EventId_;    /**< Event id */
    ACE_UINT16 u16Length_;     /**< Length of message in bytes */
    ACE_UINT8  data_[0];       /**< Pointer to message payload */
  } __attribute__((packed));
  
  /**
   * Convert a EventServiceClientMessage to host byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  inline EventServiceClientMessage * eventServiceClientMessageToHost(EventServiceClientMessage * pMsg)
  {
    pMsg->u16PlatformId_ = ACE_NTOHS(pMsg->u16PlatformId_);
    pMsg->u16NEMId_      = ACE_NTOHS(pMsg->u16NEMId_);
    pMsg->u16LayerType_  = ACE_NTOHS(pMsg->u16LayerType_);
    pMsg->u16EventId_    = ACE_NTOHS(pMsg->u16EventId_);
    pMsg->u16Length_     = ACE_NTOHS(pMsg->u16Length_);
    return pMsg;
  }
  
  /**
   * Convert a EventServiceClientMessage to network byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  inline EventServiceClientMessage * eventServiceClientMessageToNet(EventServiceClientMessage * pMsg)
  {
    pMsg->u16PlatformId_ = ACE_HTONS(pMsg->u16PlatformId_);
    pMsg->u16NEMId_      = ACE_HTONS(pMsg->u16NEMId_);
    pMsg->u16LayerType_  = ACE_HTONS(pMsg->u16LayerType_);
    pMsg->u16EventId_    = ACE_HTONS(pMsg->u16EventId_);
    pMsg->u16Length_     = ACE_HTONS(pMsg->u16Length_);
    return pMsg;
  }
}

#endif //EMANEEVENTSERVICEMESSAGE_HEADER_
