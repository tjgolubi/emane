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

#ifndef UNIVERSALPHY_NOISEENTRY_HEADER_
#define UNIVERSALPHY_NOISEENTRY_HEADER_

#include <ace/Time_Value.h>

namespace UniversalPHY {
/**
 *
 * @class  NoiseEntry 
 *
 * @brief defines the attributes of noise on a channel (duration, power, time)
 *
 */
  class NoiseEntry {
     public:
      /**
       *
       * @enum NOISE_STATUS  
       *
       */
       enum NOISE_STATUS { NOISE_STATUS_NONE    = 0,  /**< noise status none */
                           NOISE_STATUS_CURRENT = 1,  /**< noise is current in time */
                           NOISE_STATUS_EXPIRED = 2,  /**< noise is expired in time */
                           NOISE_STATUS_FUTURE  = 3   /**< noise is future in time */
       };

      /**
       *
       * @brief default constructor
       *
       */
       NoiseEntry ();

      /**
       *
       * @brief parameter constructor
       *
       * @param tvT0 initial time
       * @param tvOffset time offset
       * @param tvDuration noise duration
       * @param dNoiseMilliWatt noise in milliwatts
       *
       */
       NoiseEntry (const ACE_Time_Value & tvT0, const ACE_Time_Value & tvOffset, const ACE_Time_Value & tvDuration, const double dNoiseMilliWatt);

      /**
       *
       * @param tvT0 initial time
       * @param tvOffset time offset
       * @param tvDuration noise duration
       *
       * @return returns the the noise status for the given parameters
       *
       */
       NOISE_STATUS getNoiseStatus(const ACE_Time_Value & tvT0, const ACE_Time_Value & tvOffset, const ACE_Time_Value & tvDuration);

       /**
        *
        * @return returns the noise level in milliwatts
        *
        */
       double getNoiseMilliWatt() const;

      private:
        ACE_Time_Value  tvTime_;
        ACE_Time_Value  tvDuration_;
        ACE_Time_Value  tvEndTime_;
        double          dNoiseMilliWatt_;
   };
}

#endif //UNIVERSALPHY_NOISEENTRY_HEADER_
