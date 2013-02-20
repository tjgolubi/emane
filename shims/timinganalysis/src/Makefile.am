noinst_LTLIBRARIES = libtiminganalysisshim.la

INCLUDES = -I@top_srcdir@/include

libtiminganalysisshim_la_CPPFLAGS= \
 $(ACE_CFLAGS)              \
 $(AM_CFLAGS)

libtiminganalysisshim_la_SOURCES = \
 timinganalysisshim.cc

EXTRA_DIST =       \
 timinganalysisshim.h \
 shimheader.h

libtiminganalysisshim_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libtiminganalysisshim_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)       \
 $(ACE_LIBS)

