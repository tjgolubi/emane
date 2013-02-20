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

#ifndef TUNTAP_HEADER_
#define TUNTAP_HEADER_

#include <ace/Handle_Set.h>
#include <ace/INET_Addr.h>
#include <string>

#include "emane/emaneplatformserviceprovider.h"
#include "emaneutils/netutils.h"


// linux
#ifdef linux

#include <linux/if_tun.h>

#undef ETH_ALEN 
#undef ETH_P_ARP
#undef ETH_P_IPV6

#define NETWORK_DEVICE_PATH  "/dev/net/tun"

// local sockaddr
struct sockaddr_in_t {
    short            sin_family;   // e.g. AF_INET, AF_INET6
    unsigned short   sin_port;     // e.g. htons(3490)
    struct in_addr   sin_addr;     // see struct in_addr, below
    char             sin_zero[8];  // zero this if you want to
} __attribute__((__may_alias__));



// APPLE
#elif defined(__APPLE__)

#define NETWORK_DEVICE_PATH  "/dev/tap0"

// local sockaddr
struct sockaddr_in_t {
   __uint8_t      sin_len;
   sa_family_t    sin_family;
   in_port_t      sin_port;
   struct in_addr sin_addr;
   char           sin_zero[8];
} __attribute__((__may_alias__));

// WIN32
#elif defined(WIN32)

#include <winioctl.h>

#define NETWORK_DEVICE_PATH "SYSTEM\\CurrentControlSet\\Control\\Network\\{4D36E972-E325-11CE-BFC1-08002BE10318}"

#define NETWORK_CONTROL_PATH "SYSTEM\\CurrentControlSet\\Control\\Class\\{4D36E972-E325-11CE-BFC1-08002BE10318}"


// win32 tap ioctls
#define TAP_GET_MAC               CTL_CODE (FILE_DEVICE_UNKNOWN, 1, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_GET_VERSION           CTL_CODE (FILE_DEVICE_UNKNOWN, 2, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_GET_MTU               CTL_CODE (FILE_DEVICE_UNKNOWN, 3, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_GET_INFO              CTL_CODE (FILE_DEVICE_UNKNOWN, 4, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_CONFIG_POINT_TO_POINT CTL_CODE (FILE_DEVICE_UNKNOWN, 5, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_SET_MEDIA_STATUS      CTL_CODE (FILE_DEVICE_UNKNOWN, 6, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_CONFIG_DHCP_MASQ      CTL_CODE (FILE_DEVICE_UNKNOWN, 7, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_GET_LOG_LINE          CTL_CODE (FILE_DEVICE_UNKNOWN, 8, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_CONFIG_DHCP_SET_OPT   CTL_CODE (FILE_DEVICE_UNKNOWN, 9, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define TAP_CONFIG_TUN            CTL_CODE (FILE_DEVICE_UNKNOWN,10, METHOD_BUFFERED, FILE_ANY_ACCESS)

#endif


  class TunTap
  {
   private:
  
    EMANE::PlatformServiceProvider * pPlatformService_;
    ACE_HANDLE            tunHandle_;
    std::string           tunName_;
    std::string           tunPath_;
    std::string           tunGuid_;
    int                   tunIndex_;
    ACE_INET_Addr         tunAddr_;
    ACE_INET_Addr         tunMask_;

#if defined(linux) || defined(__APPLE__)

    int set_flags (int, int);
    int get_flags ();

#elif defined(WIN32)

const static size_t IO_BUFFLEN = 2048;

struct IoOverlapped {
  char          buffer[IO_BUFFLEN];
  DWORD         byteCount;
  OVERLAPPED    overlapped;
};

 struct IoOverlapped ioRead_;
 struct IoOverlapped ioWrite_;

#endif

  public:
    TunTap (EMANE::PlatformServiceProvider * pPlatformService);

   ~TunTap ();

    int open (const char *, const char *);

    int close ();

    int activate (bool);
    
    int deactivate ();
    
    ACE_HANDLE get_handle ();

    int readv (struct iovec*, size_t);

    int writev (const struct iovec*, size_t);

    int set_addr (const ACE_INET_Addr &, const ACE_INET_Addr &);

    int set_ethaddr (const EMANEUtils::EtherAddr &);

    int set_ethaddr (ACE_UINT16);
    
    ACE_INET_Addr & get_addr ();

    ACE_INET_Addr & get_mask ();
  };

#endif // TUNTAP_HEADER_
