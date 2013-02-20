lib_LTLIBRARIES = librfpipemaclayer.la

INCLUDES =                       \
 -I@top_srcdir@/include          \
 -I../common

librfpipemaclayer_la_CPPFLAGS=   \
 $(ACE_CFLAGS)                   \
 $(AM_CFLAGS)                    \
 $(LIBXML2_CFLAGS)

librfpipemaclayer_la_SOURCES =   \
 rfpipemaclayer.cc               \
 rfpipemacdownstreamqueue.cc     \
 pcrmanager.cc

EXTRA_DIST=                      \
 rfpipemaclayer.h                \
 rfpipemacdownstreamqueue.h      \
 pcrmanager.h

librfpipemaclayer_la_LIBADD=     \
 $(WIN32_EXTRA_LIBADD)        

librfpipemaclayer_la_LDFLAGS=    \
 $(WIN32_EXTRA_LDFLAGS)          \
 $(ACE_LIBS)                     \
 $(LIBXML2_LIBS)
