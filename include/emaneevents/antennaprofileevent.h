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

#ifndef ANTENNAPROFILEEVENT_HEADER_
#define ANTENNAPROFILEEVENT_HEADER_

#include "emane/emaneevent.h"
#include "emaneevents/events.h"

#include <ace/Basic_Types.h>
#include <vector>

/**
  *
  * @class AntennaProfileEvent
  *
  * @brief Defines the AntennaProfile Update Event
  */
class AntennaProfileEvent : public EMANE::Event
{
public:
  static const EMANE::EventId EVENT_ID = EMANE::REGISTERED_EMANE_EVENT_ANTENNAPROFILE;
 
  /**
   *
   * @struct AntennaProfileEntry
   *
   * @brief exposed Antenna Profile update definition
   *
   */ 
  struct AntennaProfileEntry
    { 
      ACE_UINT16 u16Node_;
      ACE_UINT16 u16ProfileId_;
      float      fAzimuthDegrees_;
      float      fElevationDegrees_;

      /**
       *
       * @brief  default constructor
       *
       */
      AntennaProfileEntry() :
       u16Node_(0),
       u16ProfileId_(0),
       fAzimuthDegrees_(0),
       fElevationDegrees_(0)
      { }

      /**
       *
       * @brief  parameter constructor
       *
       * @param u16Node the NEM id
       * @param u16ProfileId the antenna profile id
       * @param fAzimuthDegrees the antenna azimuth in degrees
       * @param fElevationDegrees the antenna elevation in degrees
       *
       */
      AntennaProfileEntry(const ACE_UINT16 u16Node, const ACE_UINT16 u16ProfileId, 
                          const float fAzimuthDegrees, const float fElevationDegrees) :
       u16Node_(u16Node),
       u16ProfileId_(u16ProfileId),
       fAzimuthDegrees_(fAzimuthDegrees),
       fElevationDegrees_(fElevationDegrees)
      { }
    } __attribute__((packed));

   /**
    *
    * @brief  parameter constructor
    *
    * @param pEntries pointer of  AntennaProfileEntry 
    * @param  u16NumberOfEntries number of entries
    *
    */
  AntennaProfileEvent(AntennaProfileEntry * pEntries, ACE_UINT16 u16NumberOfEntries);
 
   /**
    *
    * @brief  parameter constructor
    *
    * @param state event object state (the data)
    * @exception  throws EventObjectStateException
    *
    */
  AntennaProfileEvent(const EMANE::EventObjectState state)
    throw(EMANE::EventObjectStateException);
  
  ~AntennaProfileEvent();
 
  /**
   *
   * @return returns EventObjectState built from internal data
   *
   */
  EMANE::EventObjectState getObjectState() const;
 
  /**
   *
   * @return returns a const pointer to AntennaProfileEntry(s)
   *
   */
  const AntennaProfileEntry * getEntries() const;

  /**
   *
   * @param id the requested NEM id
   * @param entry the AntennaProfileEntry data (out param)
   *
   * @return copies entry data and returns true if the entry is found, else returns false
   *
   */
  bool findEntry(EMANE::NEMId id, AntennaProfileEntry & entry) const;

  /**
   *
   * @return returns the number of stored entries
   *
   */
  ACE_UINT16 getNumberOfEntries() const;
  
private:
  AntennaProfileEntry * pEntries_;

  ACE_UINT16            u16NumberOfEntries_;
};

#include "emaneevents/antennaprofileevent.inl"

#endif // ANTENNAPROFILEEVENT_HEADER_
