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

#ifndef EMANEEVENTS_HEADER_
#define EMANEEVENTS_HEADER_

#include "emane/emaneevent.h"

namespace EMANE
{
  /* 
   * Resevered Event Id Range (0,32767]
   *.
   * Local Event Id Range     [32768,65535)
   */
  
  /*
   * Pathloss Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.09.11
   */
  const EventId REGISTERED_EMANE_EVENT_PATHLOSS = 203;

  /*
   * Location Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.09.11
   */
  const EventId REGISTERED_EMANE_EVENT_LOCATION = 204;

  /*
   * Statistic Request Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.09.11
   */
  const EventId REGISTERED_EMANE_EVENT_STATISTIC_REQUEST = 205;

  /*
   * Statistic Response Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.09.11
   */
  const EventId REGISTERED_EMANE_EVENT_STATISTIC_RESPONSE = 206;

  /*
   * Statistic Response Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2009.08.10
   */
  const EventId REGISTERED_EMANE_EVENT_COMMEFFECT = 207;
 
  /*
   * IEEE80211ABG One Hop Nbr List Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2011.02.10
   */
  const EventId REGISTERED_EMANE_EVENT_IEEE80211ABG_ONE_HOP_NBR_LIST = 208;
 
  /*
   * Antenna Direction Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2011.02.24
   *
   * depricated, by antenna profile event
   *
   * const EventId REGISTERED_EMANE_EVENT_ANTENNADIRECTION = 209;
   */

  /*
   * Antenna Profile Event.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2012.11.28
   */
  const EventId REGISTERED_EMANE_EVENT_ANTENNAPROFILE = 210;

}


#endif //EMANEEVENTS_HEADER_
