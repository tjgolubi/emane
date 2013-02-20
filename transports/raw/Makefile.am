lib_LTLIBRARIES = libtransraw.la

INCLUDES = \
 -I@top_srcdir@/include  \
 -I../common

libtransraw_la_CPPFLAGS= \
 $(ACE_CFLAGS)           \
 $(PCAP_CFLAGS)          \
 $(AM_CFLAGS)

libtransraw_la_SOURCES= \
 rawtransport.cc        \
 ../common/ethernettransport.cc

EXTRA_DIST=             \
 rawtransport.h         \
 ../common/ethernettransport.h


libtransraw_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libtransraw_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS) \
 $(PCAP_LIBS)           \
 $(ACE_LIBS)            


