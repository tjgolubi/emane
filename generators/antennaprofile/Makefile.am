lib_LTLIBRARIES = libantennaprofilegenerator.la

INCLUDES =              \
 -I@top_srcdir@/include 

libantennaprofilegenerator_la_CPPFLAGS= \
 $(ACE_CFLAGS)                          \
 $(AM_CFLAGS)

libantennaprofilegenerator_la_SOURCES= \
 antennaprofilegenerator.cc            \
 antennaprofileloader.cc

EXTRA_DIST=                           \
 antennaprofilegenerator.h            \
 antennaprofileloader.h                

libantennaprofilegenerator_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libantennaprofilegenerator_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)                \
 $(ACE_LIBS)
