lib_LTLIBRARIES = libgpsdlocationagent.la

INCLUDES =              \
 -I@top_srcdir@/include 

libgpsdlocationagent_la_CPPFLAGS= \
 $(ACE_CFLAGS)                 \
 $(AM_CFLAGS)

libgpsdlocationagent_la_SOURCES= \
 gpsdlocationagent.cc

EXTRA_DIST=                      \
 gpsdlocationagent.h             

libgpsdlocationagent_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libgpsdlocationagent_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)          \
 $(ACE_LIBS)                  \
 -lutil
