lib_LTLIBRARIES = libbypassmaclayer.la

INCLUDES = -I@top_srcdir@/include

libbypassmaclayer_la_CPPFLAGS= \
 $(ACE_CFLAGS)              \
 $(AM_CFLAGS)

libbypassmaclayer_la_SOURCES = \
 bypassmaclayer.cc

EXTRA_DIST =       \
 bypassmaclayer.h

libbypassmaclayer_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libbypassmaclayer_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)       \
 $(ACE_LIBS)

