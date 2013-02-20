/*
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef IEEE80211ABG_MSGTYPES_HEADER_
#define IEEE80211ABG_MSGTYPES_HEADER_


#include <ace/Basic_Types.h>



namespace IEEE80211ABG {

const ACE_UINT8 MSG_TYPE_NONE                   =  0x00;
const ACE_UINT8 MSG_TYPE_BROADCAST_DATA         =  0x01;
const ACE_UINT8 MSG_TYPE_UNICAST_DATA           =  0x02;
const ACE_UINT8 MSG_TYPE_UNICAST_RTS_CTS_DATA   =  0x03;
const ACE_UINT8 MSG_TYPE_UNICAST_CTS_CTRL       =  0x04;

const ACE_UINT32 MSG_TYPE_MASK_BROADCAST        =  0x01;
const ACE_UINT32 MSG_TYPE_MASK_UNICAST          =  0x02;
const ACE_UINT32 MSG_TYPE_MASK_UNICAST_RTSCTS   =  0x04;
const ACE_UINT32 MSG_TYPE_MASK_CTS              =  0x08;
const ACE_UINT32 MSG_TYPE_MASK_ALL              =  0x0F;
const ACE_UINT32 MSG_TYPE_MASK_ALL_DATA         =  0x07;

}

#endif //IEEE80211ABG_MSGTYPES_HEADER_
