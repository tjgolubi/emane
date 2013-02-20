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

#ifndef EMANELOGMESSAGE_HEADER_
#define EMANELOGMESSAGE_HEADER_

#include <ace/Basic_Types.h>

namespace EMANE
{
  const ACE_UINT16 LOG_COMMAND_LEVEL_CONTROL     = 1;
  const ACE_UINT16 NEM_LOG_COMMAND_LEVEL_CONTROL = 2;
  const ACE_UINT16 LOG_PUBLISH_MESSAGE           = 3;

  /**
   * @class LogMessageHeader
   *
   * @brief Log message header
   */
  struct LogMessageHeader
  {
    ACE_UINT16 u16Id_;     /**< Log message id */
    ACE_UINT16 u16Length_; /**< Total message length in bytes */
    ACE_UINT8  data_[0];
  } __attribute__((packed));

  /**
   * Convert a LogMessageHeader to host byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  LogMessageHeader * logMessageHeaderToHost(LogMessageHeader * pMsg)
  {
    pMsg->u16Id_     =  ACE_NTOHS(pMsg->u16Id_);
    pMsg->u16Length_ =  ACE_NTOHS(pMsg->u16Length_);
    return pMsg;
  }

  /**
   * Convert a LogMessageHeader to network byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  LogMessageHeader * logMessageHeaderToNet(LogMessageHeader * pMsg)
  {
    pMsg->u16Id_     =  ACE_HTONS(pMsg->u16Id_);
    pMsg->u16Length_ =  ACE_HTONS(pMsg->u16Length_);
    return pMsg;
  }
  
  /**
   * @class LogLevelControl
   *
   * @brief Log level control message header
   */
  struct LogLevelControl
  {
    ACE_UINT16 u16Level_;   /**< New log level */
  } __attribute__((packed));

  /**
   * Convert a LogLevelControl to host byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  LogLevelControl * logMessageToHost(LogLevelControl * pMsg)
  {
    pMsg->u16Level_ =  ACE_NTOHS(pMsg->u16Level_);
    return pMsg;
  }

  /**
   * Convert a LogLevelControl to network byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  LogLevelControl * logMessageToNet(LogLevelControl * pMsg)
  {
    pMsg->u16Level_ =  ACE_HTONS(pMsg->u16Level_);
    return pMsg;
  }

  /**
   * @class NEMLogLevelControl
   *
   * @brief NEM Log level control message header
   */
  struct NEMLogLevelControl
  {
    ACE_UINT16 u16NEMId_;   /**< NEM id of target */
    ACE_UINT16 u16Level_;   /**< New log level */
  } __attribute__((packed));


 /**
   * Convert a NEMLogLevelControl to host byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  NEMLogLevelControl * NEMLogMessageToHost(NEMLogLevelControl * pMsg)
  {
    pMsg->u16NEMId_ =  ACE_NTOHS(pMsg->u16NEMId_);
    pMsg->u16Level_ =  ACE_NTOHS(pMsg->u16Level_);
    return pMsg;
  }

  /**
   * Convert a NEMLogLevelControl to network byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  NEMLogLevelControl * NEMLogMessageToNet(NEMLogLevelControl * pMsg)
  {
    pMsg->u16NEMId_ =  ACE_HTONS(pMsg->u16NEMId_);
    pMsg->u16Level_ =  ACE_HTONS(pMsg->u16Level_);
    return pMsg;
  }

  /**
   * @class LogPublishMessage
   *
   * @brief Log message header
   */
  struct LogPublishMessage
  {
  } __attribute__((packed));

  /**
   * Convert a LogPublishMessage to host byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  LogPublishMessage * LogPublishMessageToHost(LogPublishMessage * pMsg)
  {
    return pMsg;
  }
  
  /**
   * Convert a LogPublishMessage to network byte order
   * 
   * @param pMsg Message reference
   *
   * @return Message reference.  Same as @a pMsg.
   */
  LogPublishMessage * LogPublishMessageToNet(LogPublishMessage * pMsg)
  {
    return pMsg;
  }
}

#endif //EMANELOGMESSAGE_HEADER_
