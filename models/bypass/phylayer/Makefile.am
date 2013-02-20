lib_LTLIBRARIES = libbypassphylayer.la

INCLUDES = -I@top_srcdir@/include

libbypassphylayer_la_CPPFLAGS= \
 $(ACE_CFLAGS)              \
 $(AM_CFLAGS)

libbypassphylayer_la_SOURCES = \
 bypassphylayer.cc 

EXTRA_DIST =         \
 bypassphylayer.h    

libbypassphylayer_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libbypassphylayer_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)       \
 $(ACE_LIBS)


