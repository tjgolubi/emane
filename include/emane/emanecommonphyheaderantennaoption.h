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

#ifndef EMANE_COMMONPHYHEADERANTENNAOPTION_HEADER_
#define EMANE_COMMONPHYHEADERANTENNAOPTION_HEADER_

#include <ace/Basic_Types.h>

#include <string>

#include "emane/emanecommonphyheaderoptions.h"

namespace EMANE
{
  /**
   * @class  CommonPHYHeaderAntennaOption 
   *
   * @brief  optional header added to the CommonPHYHeader to indicate antenna direction if supported by the NEM.
   *
   */
  class CommonPHYHeaderAntennaOption : public CommonPHYHeaderOption
  {
  public:
    const static ACE_UINT16 COMMON_PHY_OPTION = COMMON_PHY_HEADER_OPTION_ANTENNA;

  /**
   * CommonPHYHeaderAntenaOption initializer
   *
   * @param  u16ProfileId            antenna profile id
   * @param  fAntennaAzimuthDeg      antenna azimuth in degrees
   * @param  fAntennaElevationDeg    antenna elevation in degrees
   *
   */
   CommonPHYHeaderAntennaOption(const ACE_UINT16 u16ProfileId,
                                const float fAntennaAzimuthDeg,
                                const float fAntennaElevationDeg);
   
 
  /**
   * CommonPHYHeaderAntenaOption initializer
   *
   * @param  state CommonPHYHeaderOptionObjectState 
   * @exception throws  CommonPHYHeaderException
   *
   */
    CommonPHYHeaderAntennaOption(const CommonPHYHeaderOptionObjectState & state)
      throw(CommonPHYHeaderException);
 
   /**
    *
    * @return return antenna pattern profile id
    *
    */
    ACE_UINT8 getAntennaProfileId() const;
    
   /**
    *
    * @return return antenna elevation in degrees
    *
    */
    float getAntennaElevationDegrees() const;
    
   /**
    *
    * @return return antenna azimuth in degrees
    *
    */
    float getAntennaAzimuthDegrees() const;

   /**
    *
    * @return returns a formatted string of antenna data parameters and values for logging.
    *
    */
    std::string format();

   /**
    *
    * @return returns the antenna data object state
    *
    */
    CommonPHYHeaderOptionObjectState getObjectState() const;
      
  private:
  /**
   * @struct Data
   *
   * @brief  the CommonPHYHeaderAntennaOption private data
   *
   */
    struct Data
    {
      ACE_UINT16   u16ProfileId_;               // antenna pattern profile id
      ACE_UINT32   u32AntennaAzimuthMARCS_;     // antenna azimuth in milli arc seconds
      ACE_INT32    i32AntennaElevationMARCS_;   // antenna elevation in milli arc seconds
    } __attribute__((packed)) data_;

    /**
     *
     * @brief convert phy antenna direction header from host to network byte order.
     *
     */
    void toNetworkByteOrder(Data & data) const;
 
    /**
     *
     * @brief convert phy antenna direction header from network to host byte order.
     *
     */
    void toHostByteOrder(Data & data) const;
  };
}
#include "emane/emanecommonphyheaderantennaoption.inl"

#endif //EMANE_COMMONPHYHEADERANTENNAOPTION_HEADER_
