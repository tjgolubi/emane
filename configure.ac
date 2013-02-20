AC_COPYRIGHT([
Copyright (c) 2008-2013 - DRS CenGen, LLC, Columbia, Maryland
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
 
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.
* Neither the name of DRS CenGen, LLC nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

The copyright holder hereby grants to the U.S. Government a copyright
license to use this computer software/computer software documentation
that is of the same scope as the rights set forth in the definition of
"unlimited rights" found in DFARS 252.227-7014(a)(15)(June 1995).
])

AC_INIT([emane],0.8.1,[labs at cengen dot com])

AC_CONFIG_SRCDIR([src/emane/emane.cc])

AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([1.9.6])

AC_PROG_CXX

AC_PROG_CXXCPP

AC_PROG_INSTALL

AC_LIBTOOL_WIN32_DLL
AC_PROG_LIBTOOL

AC_LANG([C++])

AC_ARG_WITH(profile,
           [  --with-profile          add profiling support (off)])

AC_ARG_WITH(debug,
           [  --with-debug            add debug support (off)])

AC_ARG_WITH(dot,
           [  --with-dot              Use dot with doxygen (off)])

AC_ARG_WITH(verbose-logging,
           [  --with-verbose-logging  add verbose logging support (off)])

AC_ARG_VAR([ACE_CFLAGS],[C compiler flags for ACE])
AC_ARG_VAR([ACE_LIBS], [linker flags for ACE])
AC_ARG_VAR([PCAP_CFLAGS],[C compiler flags for libpcap])
AC_ARG_VAR([PCAP_LIBS], [linker flags for libpcap])

PKG_CHECK_MODULES([LIBXML2], libxml-2.0)

AC_SUBST(LIBXML2_CFLAGS)
AC_SUBST(LIBXML2_LIBS)

_cppflags=${CPPFLAGS}
_ldflags=${LDFLAGS}

# Checks for libraries.
if test "${ACE_CFLAGS}" != ""; then
   CPPFLAGS="${ACE_CFLAGS} ${CPPFLAGS}"
fi

if test "${ACE_LIBS}" != ""; then
   LDFLAGS="${ACE_LIBS} ${LDFLAGS}"
fi

AC_CHECK_HEADER(ace/OS_main.h,
 [AC_CHECK_LIB([ACE],main,[ACE_LIBS+=' -lACE'],[AC_MSG_ERROR("ACE not installed")])],
 [AC_MSG_ERROR("Missing ACE development headers")])

# check rand_r signature
AC_MSG_CHECKING(check rand_r uses seed value pass by ptr)
AC_TRY_COMPILE([
#include <ace/OS_NS_stdlib.h>
],[
unsigned int seed = 0;
ACE_OS::rand_r(&seed);
],[
AC_MSG_RESULT(yes)
AC_DEFINE(HAVE_RAND_R_SEED_PASS_BY_PTR,,rand_r uses seed value pass by ptr)],
AC_MSG_RESULT(no))]

CPPFLAGS=${_cppflags}
LDFLAGS=${_ldflags}

AC_SUBST(ACE_CFLAGS)
AC_SUBST(ACE_LIBS)

PCAP_LIB_NAME=pcap

WIN32_EXTRA_LIBADD=
WIN32_EXTRA_LDFLAGS=

case "$host" in
 *-mingw*)
 CPPFLAGS="$CPPFLAGS -DWIN32"
 WIN32_EXTRA_LIBADD="-lws2_32 -liphlpapi"
 WIN32_EXTRA_LDFLAGS=-no-undefined
 PCAP_LIB_NAME=wpcap
 ;;
*)
 LDFLAGS="$LDFLAGS -lpthread -ldl"
 ;;
esac

AC_SUBST(WIN32_EXTRA_LIBADD)
AC_SUBST(WIN32_EXTRA_LDFLAGS)

_cppflags=${CPPFLAGS}
_ldflags=${LDFLAGS}

# Checks for libraries.
if test "${PCAP_CFLAGS}" != ""; then
   CPPFLAGS="${PCAP_CFLAGS} ${CPPFLAGS}"
fi

if test "${PCAP_LIBS}" != ""; then
   LDFLAGS="${PCAP_LIBS} ${LDFLAGS}"
fi

AC_CHECK_HEADER(pcap.h,
 [AC_CHECK_LIB([${PCAP_LIB_NAME}],pcap_open_live,
 [AM_CONDITIONAL(EMANE_HAS_PCAP,true) PCAP_LIBS+=" -l${PCAP_LIB_NAME}"],
 [AM_CONDITIONAL(EMANE_HAS_PCAP,false)])],
 [AM_CONDITIONAL(EMANE_HAS_PCAP,false)])

CPPFLAGS=${_cppflags}
LDFLAGS=${_ldflags}

AC_SUBST(PCAP_CFLAGS)
AC_SUBST(PCAP_LIBS)

LDFLAGS="${LDFLAGS} -avoid-version"

AC_C_BIGENDIAN(AC_DEFINE([__BYTE_ORDER],__BIG_ENDIAN,[Big Endian]),AC_DEFINE([__BYTE_ORDER],__LITTLE_ENDIAN,[Little Endian]))
AC_DEFINE([__LITTLE_ENDIAN],1234,[for the places where it is not defined])
AC_DEFINE([__BIG_ENDIAN],4321,[for the places where it is not defined])

# compiler options
CXXFLAGS="-W -Wall -O2"

# options for use with gprof
if test "$with_profile" = "yes"
then
CPPFLAGS="$CPPFLAGS -g -pg"
LDFLAGS="$LDFLAGS -g -pg"
fi

# options for use with debug
if test "$with_debug" = "yes"
then
CPPFLAGS="$CPPFLAGS -g -O0"
LDFLAGS="$LDFLAGS -g -O0"
fi

# options for use with verbose logging
if test "$with_verbose_logging" = "yes"
then
CPPFLAGS="$CPPFLAGS -DVERBOSE_LOGGING"
fi

LANG=C
AC_SUBST(LANG)

RELDATE=`date +'%a %b %e %Y'`
AC_SUBST(RELDATE)

AC_CHECK_HEADER([pty.h],
[AM_CONDITIONAL(EMANE_HAS_PTY,true)],
[AC_CHECK_HEADER([util.h],
 [AM_CONDITIONAL(EMANE_HAS_PTY,true)],
 [AM_CONDITIONAL(EMANE_HAS_PTY,false)])])

# check for doxygen
AC_CHECK_PROG(HAVE_DOXYGEN, doxygen, true, false)
AM_CONDITIONAL(HAVE_DOXYGEN,$HAVE_DOXYGEN)

if test "$with_dot" = "yes"
then
# check for dot
AC_CHECK_PROG(HAVE_DOT, dot, true, false)
AM_CONDITIONAL(HAVE_DOT,$HAVE_DOT)
else
AM_CONDITIONAL(HAVE_DOT,false)
fi

# check for rpmbuild
AC_CHECK_PROG(HAVE_RPMBUILD, rpmbuild, true, false)
AM_CONDITIONAL(HAVE_RPMBUILD,$HAVE_RPMBUILD)

# check for deb
AC_CHECK_PROG(HAVE_DEB, dh_clean, true, false)
AM_CONDITIONAL(HAVE_DEB,$HAVE_DEB)

AC_OUTPUT(
 emane.spec:emane.spec.in
 Makefile 
 src/libemanexmlparser/Makefile 
 src/libemane/Makefile
 src/emane/Makefile
 src/emanetransportd/Makefile
 src/emanetransportd/xml/Makefile
 src/emaneeventd/Makefile
 src/emaneeventd/xml/Makefile
 src/emaneeventservice/Makefile
 src/emaneeventservice/xml/Makefile
 generators/antennaprofile/Makefile
 generators/antennaprofile/xml/Makefile
 generators/mitremobility/Makefile
 generators/mitremobility/xml/Makefile
 generators/emulationscript/Makefile
 generators/emulationscript/xml/Makefile
 transports/common/Makefile
 transports/virtual/Makefile
 transports/virtual/xml/Makefile
 transports/raw/Makefile
 transports/raw/xml/Makefile
 models/bypass/maclayer/Makefile
 models/bypass/phylayer/Makefile
 models/bypass/xml/Makefile
 models/ieee80211abg/maclayer/Makefile
 models/ieee80211abg/xml/Makefile
 models/rfpipe/maclayer/Makefile
 models/rfpipe/xml/Makefile
 models/universal/phylayer/Makefile
 models/universal/xml/Makefile
 shims/timinganalysis/src/Makefile
 shims/timinganalysis/xml/Makefile
 shims/phyapitest/src/Makefile
 shims/phyapitest/xml/Makefile
 include/emane/Makefile
 include/emaneutils/Makefile
 include/emaneevents/Makefile
 include/emanecontrolmessages/Makefile
 include/libemane/Makefile
 agents/gpsdlocation/Makefile
 agents/gpsdlocation/xml/Makefile
 agents/sampleagent/Makefile
 scripts/Makefile
 scripts/doc/Makefile
 dtd/Makefile
 doc/Makefile
 doc/emane.doxygen:doc/emane.doxygen.in
)
