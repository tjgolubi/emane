noinst_LTLIBRARIES = libsampleagent.la

INCLUDES =              \
 -I@top_srcdir@/include 

libsampleagent_la_CPPFLAGS= \
 $(ACE_CFLAGS)                 \
 $(AM_CFLAGS)

libsampleagent_la_SOURCES= \
 sampleagent.cc

EXTRA_DIST=                      \
 sampleagent.h            \
 sampleagentevent.h       \
 sampleagentevent.inl

libsampleagent_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libsampleagent_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)          \
 $(ACE_LIBS)                  
 
