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

#ifndef EMANE_CONSTANTS_HEADER_
#define EMANE_CONSTANTS_HEADER_

// Need for Win32 and OS X
#ifndef M_PIl
#define M_PIl 3.1415926535897932384626433832795029L
#endif

namespace EMANE
{
  // defines full rf path loss
  const float FULL_PATH_LOSS_DB = 512.0;

  // arc seconds per degree of angle
  const float ARC_SECONDS_PER_DEGREE = 3600.0;

  // milli arc seconds per degree of angle (3600 arc seconds per deg)
  const float MILLI_ARC_SECONDS_PER_DEGREE = 3600.0e3;

  const double SEMI_MAJOR   = 6378137.0;
  const double SEMI_MINOR   = 6356752.3142;
  const double SEMI_MAJOR_2 = SEMI_MAJOR * SEMI_MAJOR;
  const double SEMI_MINOR_2 = SEMI_MINOR * SEMI_MINOR;
  const double ECC2         = (SEMI_MAJOR_2 - SEMI_MINOR_2) / SEMI_MAJOR_2; 
}


#endif // EMANE_CONSTANTS_HEADER_
