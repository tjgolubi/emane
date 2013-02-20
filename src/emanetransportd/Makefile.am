bin_PROGRAMS = emanetransportd

INCLUDES =                             \
 -I@top_srcdir@/include                \
 -I@top_srcdir@/src/libemanexmlparser  \
 -I@top_srcdir@/src/libemane

emanetransportd_CXXFLAGS =  \
 $(LIBXML2_CFLAGS)          \
 $(ACE_CFLAGS)              \
 $(AM_CFLAGS)


emanetransportd_SOURCES =                  \
 emanetransportd.cc                        \
 transportdaemonconfiguration.cc           \
 transportinstanceconfiguration.cc         \
 transportdirector.cc

EXTRA_DIST  =                        \
 transportdaemonconfiguration.h      \
 transportinstanceconfiguration.h    \
 transportdirector.h

emanetransportd_LDADD =                      \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -L@top_srcdir@/src/libemane/.libs           \
 -lemanexmlparser                            \
 -lemane                                     \
 $(LIBXML2_LIBS)                             \
 $(ACE_LIBS)
