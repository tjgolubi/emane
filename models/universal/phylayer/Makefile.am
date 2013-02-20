lib_LTLIBRARIES = libuniversalphylayer.la

INCLUDES = -I@top_srcdir@/include

libuniversalphylayer_la_CPPFLAGS=  \
 $(ACE_CFLAGS)                     \
 $(AM_CFLAGS)                      \
 $(LIBXML2_CFLAGS)

libuniversalphylayer_la_SOURCES =  \
 universalphylayer.cc              \
 pathlossmanager.cc                \
 pathlossentry.cc                  \
 noisemanager.cc                   \
 antennaprofilemanager.cc          \
 antennaprofileparser.cc           \
 antennaprofile.cc                 \
 antennaprofilepattern.cc          \
 noiseentry.cc                     \
 frequencyentry.cc

EXTRA_DIST =                       \
 universalphylayer.h               \
 pathlossmanager.h                 \
 pathlossentry.h                   \
 noisemanager.h                    \
 antennaprofilemanager.h           \
 antennaprofileparser.h            \
 antennaprofile.h                  \
 antennaprofilepattern.h           \
 noiseentry.h                      \
 frequencyentry.h                  \
 universalphyheader.h

libuniversalphylayer_la_LIBADD=    \
 $(WIN32_EXTRA_LIBADD)        

libuniversalphylayer_la_LDFLAGS=   \
 $(WIN32_EXTRA_LDFLAGS)            \
 $(ACE_LIBS)                       \
 $(LIBXML2_LIBS)
