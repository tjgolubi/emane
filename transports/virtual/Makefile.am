lib_LTLIBRARIES = libtransvirtual.la

INCLUDES = \
 -I@top_srcdir@/include     \
 -I../common

libtransvirtual_la_CPPFLAGS= \
 $(ACE_CFLAGS)               \
 $(AM_CFLAGS)

libtransvirtual_la_SOURCES= \
 virtualtransport.cc        \
 tuntap.cc                  \
 ../common/ethernettransport.cc

EXTRA_DIST=                 \
 virtualtransport.h         \
 tuntap.h                   \
 ../common/ethernettransport.h

libtransvirtual_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libtransvirtual_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)     \
 $(ACE_LIBS)                

