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

#ifndef LOCATIONEVENT_HEADER_
#define LOCATIONEVENT_HEADER_

#include "emane/emaneevent.h"
#include "emaneevents/events.h"
#include "emane/emaneconstants.h"

#include <ace/Basic_Types.h>

/**
  * @class LocationEvent 
  *
  * @brief defines the location update event
  *
  */
class LocationEvent : public EMANE::Event
{
public:
  static const EMANE::EventId EVENT_ID = EMANE::REGISTERED_EMANE_EVENT_LOCATION;

  /**
   * @struct  LocationEntry
   *
   * @brief Location update definition
   *
   */ 
  struct LocationEntry
    { 
      ACE_UINT16 u16Node_;
      ACE_INT32  i32LatitudeMARCS_;   // MARCS (milli arc seconds)
      ACE_INT32  i32LongitudeMARCS_;
      ACE_INT32  i32AltitudeMeters_;
      ACE_INT32  i32YawMARCS_;
      ACE_INT32  i32PitchMARCS_;
      ACE_INT32  i32RollMARCS_;
      ACE_INT32  i32VelocityAzimuthMARCS_; 
      ACE_INT32  i32VelocityElevationMARCS_; 
      ACE_UINT32 u32VelocityCMPS_;   // CMPS (centi meters per second)

     /**
      *
      * @brief  default constructor
      *
      */
      LocationEntry () :
       u16Node_(0),
       i32LatitudeMARCS_(0),
       i32LongitudeMARCS_(0),
       i32AltitudeMeters_(0),
       i32YawMARCS_(0),
       i32PitchMARCS_(0),
       i32RollMARCS_(0),
       i32VelocityAzimuthMARCS_(0),
       i32VelocityElevationMARCS_(0),
       u32VelocityCMPS_(0)
     { }
    } __attribute__((packed));

  /**
   *
   * @brief  parameter constructor
   *
   * @param pEntries pointer of LocationEntry 
   * @param  u16NumberOfEntries number of entries
   *
   */
  LocationEvent(LocationEntry * pEntries, ACE_UINT16 u16NumberOfEntries);
  
   /**
    *
    * @brief  parameter constructor
    *
    * @param state event object state (the data)
    * @exception  throws EventObjectStateException
    *
    */
  LocationEvent(const EMANE::EventObjectState state)
    throw(EMANE::EventObjectStateException);
  
  ~LocationEvent();
  
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
  const LocationEntry * getEntries() const;
 
  /**
   *
   * @return returns the number of stored entries
   *
   */
  ACE_UINT16 getNumberOfEntries() const;

private:
  LocationEntry * pEntries_;

  ACE_UINT16 u16NumberOfEntries_;
};

#include "emaneevents/locationevent.inl"

#endif // LOCATIONEVENT_HEADER_
