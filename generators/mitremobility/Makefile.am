lib_LTLIBRARIES = libmitremobilitygenerator.la

INCLUDES =              \
 -I@top_srcdir@/include 

libmitremobilitygenerator_la_CPPFLAGS= \
 $(ACE_CFLAGS)                      \
 $(AM_CFLAGS)

libmitremobilitygenerator_la_SOURCES= \
 mobilitymodeleventgenerator.cc       \
 mobilitymodelloader.cc

EXTRA_DIST=                           \
 mobilitymodeleventgenerator.h        \
 mobilitymodelloader.h                

libmitremobilitygenerator_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libmitremobilitygenerator_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)               \
 $(ACE_LIBS)
