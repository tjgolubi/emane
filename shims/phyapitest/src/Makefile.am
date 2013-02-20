lib_LTLIBRARIES = libphyapitestshim.la

INCLUDES = -I@top_srcdir@/include

libphyapitestshim_la_CPPFLAGS= \
 $(ACE_CFLAGS)                 \
 $(AM_CFLAGS)

libphyapitestshim_la_SOURCES = \
 phyapitestshim.cc

EXTRA_DIST =      \
 phyapitestshim.h

libphyapitestshim_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libphyapitestshim_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)       \
 $(ACE_LIBS)

