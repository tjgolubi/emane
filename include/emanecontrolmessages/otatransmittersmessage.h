/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANE_OTA_TRANSMITTERSMESSAGE_HEADER_
#define EMANE_OTA_TRANSMITTERSMESSAGE_HEADER_

#include "emanecontrolmessages/controlmessages.h"
#include "emanecontrolmessages/otatransmittersmessageexception.h"
#include "emane/emanecontrolmessage.h"
#include "emane/emanenemlist.h"

#include <ace/Basic_Types.h>


namespace EMANE
{
  /**
   * @class OTATransmittersMessage
   *
   * @brief OTA additional transmitter list message
   *
   */
  class OTATransmittersMessage
  { 

    public:
      static const ACE_INT32 MAJOR_ID = EMANE_OTA_MAJOR_ID;
      static const ACE_INT32 MINOR_ID = EMANE_OTA_TRANSMITTERS_MINOR_ID;

      /**
       *
       * @brief default constructor
       *
       */
      OTATransmittersMessage();

      /**
       *
       * @brief parmater constructor
       *
       * @param msg EMANE control message
       * @exception throws  OTATransmittersMessageException
       *
       */
      OTATransmittersMessage(const ControlMessage & msg) 
        throw(OTATransmittersMessageException);

      /**
       *
       * @brief parmater constructor
       *
       * @param set a set of NEM id(s) (additional transmitters)
       *
       */
      OTATransmittersMessage(const EMANE::NEMIdSet & set);

      /**
       *
       * @param ats a constant reference to a set of addition transmitters
       *
       */
      void set (const EMANE::NEMIdSet & ats);


      /**
       *
       * @param ats a reference to a set of addition transmitters
       *
       */
      void get (EMANE::NEMIdSet & ats);


      /**
       *
       * @return returns a complete EMANE ControlMessage 
       *
       */
      ControlMessage buildControlMessage() const;

    private:
       EMANE::NEMIdSet set_;

      /**
       *
       * @param msg unpacks a complete EMANE ControlMessage
       * @exception throws  OTATransmittersMessageException
       *
       */
      void unpackControlMessage(const ControlMessage & msg)
        throw(OTATransmittersMessageException);


   };
}

#include "otatransmittersmessage.inl"

#endif //EMANE_OTA_TRANSMITTERSMESSAGE_HEADER_
