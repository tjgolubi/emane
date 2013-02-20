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

#ifndef EMANEOTAMANAGERMESSAGE_HEADER_
#define EMANEOTAMANAGERMESSAGE_HEADER_

#include <ace/Basic_Types.h>

namespace EMANE
{
  const ACE_UINT16 OTA_DATA_MESSAGE = 1;

  /**
   * @class OTAMessage
   *
   * @brief OTA message header
   */
  struct OTAMessage
  {
    ACE_UINT16 u16Id_;     /**< OTA message id */
    ACE_UINT16 u16Length_; /**< Total length in bytes id */
    ACE_UINT8  data_[0];

    OTAMessage () :
      u16Id_(0),
      u16Length_(0)
    { }

    OTAMessage (ACE_UINT16 u16Id, ACE_UINT16 u16Length) :
      u16Id_(u16Id),
      u16Length_(u16Length)
    { }
  } __attribute__((packed));
  
  /**
   * Convert a OTAMessage to host byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  OTAMessage * OTAMessageToHost(OTAMessage * pMsg)
  {
    pMsg->u16Id_     =  ACE_NTOHS(pMsg->u16Id_);
    pMsg->u16Length_ =  ACE_NTOHS(pMsg->u16Length_);
    return pMsg;
  }

  /**
   * Convert a OTAMessage to network byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  OTAMessage * OTAMessageToNet(OTAMessage * pMsg)
  {
    pMsg->u16Id_     =  ACE_HTONS(pMsg->u16Id_);
    pMsg->u16Length_ =  ACE_HTONS(pMsg->u16Length_);
    return pMsg;
  }
  
  /**
   * @class OTADataMessage
   *
   * @brief OTA data message
   */
  struct OTADataMessage
  {
    ACE_UINT16 u16Src_;     /**< Test node source id */
    ACE_UINT16 u16Dst_;     /**< Test node destination id or 0xFFFF for broadcast*/
    ACE_UINT16 u16DSCP_;    /**< DSCP value */
    ACE_UINT16 u16DataLen_; /**< data len in bytes */
    ACE_UINT8  data_[0];

    OTADataMessage() :
     u16Src_(0),
     u16Dst_(0),
     u16DSCP_(0),
     u16DataLen_(0)
    { }

    OTADataMessage(ACE_UINT16 u16Src, ACE_UINT16 u16Dst, ACE_UINT16 u16DSCP, ACE_UINT16 u16DataLen = 0) :
     u16Src_(u16Src),
     u16Dst_(u16Dst),
     u16DSCP_(u16DSCP),
     u16DataLen_(u16DataLen)
    { }
  } __attribute__((packed));
  
  /**
   * Convert a OTADataMessage to host byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  OTADataMessage * OTADataMessageToHost(OTADataMessage * pMsg)
  {
    pMsg->u16Src_     = ACE_NTOHS(pMsg->u16Src_);
    pMsg->u16Dst_     = ACE_NTOHS(pMsg->u16Dst_);
    pMsg->u16DSCP_    = ACE_NTOHS(pMsg->u16DSCP_);
    pMsg->u16DataLen_ = ACE_NTOHS(pMsg->u16DataLen_);
    return pMsg;
  }

  /**
   * Convert a OTADataMessage to network byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  OTADataMessage * OTADataMessageToNet(OTADataMessage * pMsg)
  {
    pMsg->u16Src_     = ACE_HTONS(pMsg->u16Src_);
    pMsg->u16Dst_     = ACE_HTONS(pMsg->u16Dst_);
    pMsg->u16DSCP_    = ACE_HTONS(pMsg->u16DSCP_);
    pMsg->u16DataLen_ = ACE_HTONS(pMsg->u16DataLen_);
    return pMsg;
  }

  /**
   * @struct OTAControlMessageId
   *
   * @brief OTA Control message id
   *
   */
  struct OTAControlMessageId
    {
      ACE_INT32 i32Major_;
      ACE_INT32 i32Minor_;

      OTAControlMessageId () :
         i32Major_(0), i32Minor_(0)
       { }

      OTAControlMessageId (ACE_INT32 major, ACE_INT32 minor) :
         i32Major_(major), i32Minor_(minor)
       { }
    } __attribute__((packed));
}

#endif //EMANEOTAMANAGERMESSAGE_HEADER_
