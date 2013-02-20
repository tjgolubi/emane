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

#ifndef EMANE_UNIVERSALPHYCONTROLANTENNAPROFILEMESSAGE_HEADER_
#define EMANE_UNIVERSALPHYCONTROLANTENNAPROFILEMESSAGE_HEADER_

#include "emane/emanenet.h"
#include "emanecontrolmessages/controlmessages.h"
#include "emanecontrolmessages/universalphycontrolantennaprofilemessageexception.h"

#include <ace/Basic_Types.h>

namespace EMANE
{
  /**
   * @class UniversalPhyControlAntennaProfileMessage
   *
   * @brief universal phy control antenna profile message
   *
   */
  class UniversalPhyControlAntennaProfileMessage
  { 
    private:

      public:
      static const ACE_UINT32 MAJOR_ID = EMANE_UNIVERSALPHYCONTROL_MAJOR_ID;
      static const ACE_UINT32 MINOR_ID = EMANE_UNIVERSALPHYCONTROL_ANTENNA_PROFILE_MINOR_ID;

      /**
       *
       * @brief default initializer.
       *
       */
      UniversalPhyControlAntennaProfileMessage();

      /**
       *
       * @brief data initializer.
       *
       */
      UniversalPhyControlAntennaProfileMessage(const void * buf, const size_t len)
         throw(UniversalPHYControlAntennaProfileMessageException);

      /**
       *
       * @brief data initializer.
       *
       */
      UniversalPhyControlAntennaProfileMessage(float fAzimuthDegrees,
                                               float fElevationDegrees,
                                               ACE_UINT16 u16ProfileId);
      /**
       * @brief get antenna elevation in degrees
       *
       * @return antenna elevation
       *
       */
      float getAntennaElevationDegrees() const;

      /**
       * @brief set antenna elevation in degrees
       *
       * @param fAntennaElevationDegrees antenna elevation in degrees
       *
       */
      void setAntennaElevationDegrees(float fAntennaElevationDegrees);

      /**
       * @brief get antenna azimuth in degrees
       *
       * @return antenna azimuth
       *
       */
      float getAntennaAzimuthDegrees() const;


      /**
       * @brief set antenna azimuth in degrees
       *
       * @param fAntennaAzimuthDegrees antenna azimuth in degrees
       *
       */
      void setAntennaAzimuthDegrees(float fAntennaAzimuthDegrees);

      /**
       * @brief get antenna profile id
       *
       * @return antenna profile id
       *
       */
      ACE_UINT16 getAntennaProfileId() const;

      /**
       * @brief set antenna profile id
       *
       * @param u16ProfileId antenna profile id
       *
       */
     void setAntennaProfileId(ACE_UINT16 u16ProfileId);

      /**
       *
       * @brief parmater format helper for logging
       *
       */
      std::string format() const;
    
      /**
       *
       * @brief convert packet data from host to network byte order.
       *
       */
      void toNetworkByteOrder();

      /**
       *
       * @brief convert packet data from network to host byte order.
       *
       */
      void toHostByteOrder();

      ACE_UINT16  u16ProfileId_;                // antenna profile id
      ACE_INT32   i32AzimuthMARCS_;             // antenna azimuth in degrees
      ACE_INT32   i32ElevationMARCS_;           // antenna evevation in degrees
   } __attribute__((packed));
}

#include "universalphycontrolantennaprofilemessage.inl"

#endif //EMANE_UNIVERSALPHYCONTROLANTENNAPROFILEMESSAGE_HEADER_
