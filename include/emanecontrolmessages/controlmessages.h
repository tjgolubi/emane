/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANECONTROLMESSAGES_HEADER_
#define EMANECONTROLMESSAGES_HEADER_

#include "emane/emanetypes.h"

namespace EMANE
{
  /* 
   * Resevered Control Message Id Range (0,32767]
   *
   * Local Control Message Id Range     [32768,65535)
   */

  /*
   * Flow Control Message.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2010.07.19
   */
  static const EMANE::INT32 EMANE_FLOWCONTROL_MAJOR_ID             = 100;

  static const EMANE::INT32 EMANE_FLOWCONTROL_REQUEST_MINOR_ID     = 1;
  static const EMANE::INT32 EMANE_FLOWCONTROL_RESPONSE_MINOR_ID    = 2;
  static const EMANE::INT32 EMANE_FLOWCONTROL_RESPONSEACK_MINOR_ID = 3;
 

  /*
   * Common PHY Control Message.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2010.08.24
   */
  static const EMANE::INT32 EMANE_UNIVERSALPHYCONTROL_MAJOR_ID                             = 200;

  static const EMANE::INT32 EMANE_UNIVERSALPHYCONTROL_SEND_MINOR_ID                        =  10001;
  static const EMANE::INT32 EMANE_UNIVERSALPHYCONTROL_RECV_MINOR_ID                        =  10002;
  static const EMANE::INT32 EMANE_UNIVERSALPHYCONTROL_ANTENNA_PROFILE_MINOR_ID             =  10003;

  /*
   * OTA Control Message.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2013.01.06
   */
  static const EMANE::INT32 EMANE_OTA_MAJOR_ID                = 300;

  static const EMANE::INT32 EMANE_OTA_TRANSMITTERS_MINOR_ID   = 1;

}

#endif //EMANECONTROLMESSAGES_HEADER_
