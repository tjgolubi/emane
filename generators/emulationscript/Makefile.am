lib_LTLIBRARIES = libemulationscriptgenerator.la

INCLUDES =              \
 -I@top_srcdir@/include 

libemulationscriptgenerator_la_CPPFLAGS= \
 $(LIBXML2_CFLAGS)                       \
 $(ACE_CFLAGS)                           \
 $(AM_CFLAGS)

libemulationscriptgenerator_la_SOURCES= \
 node.cc                                \
 event.cc                               \
 emulationscripteventgenerator.cc       \
 emulationscriptloader.cc

EXTRA_DIST=                           \
 node.h                               \
 event.h                              \
 emulationscripteventgenerator.h      \
 emulationscriptloader.h              

libemulationscriptgenerator_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libemulationscriptgenerator_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)                 \
 $(LIBXML2_LIBS)                        \
 $(ACE_LIBS)
