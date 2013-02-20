bin_PROGRAMS = emaneeventservice

INCLUDES =                                    \
 -I@top_srcdir@/include                       \
 -I@top_srcdir@/src/libemanexmlparser         \
 -I@top_srcdir@/src/libemane

emaneeventservice_CXXFLAGS =  \
 $(LIBXML2_CFLAGS)            \
 $(ACE_CFLAGS)                \
 $(AM_CFLAGS)

emaneeventservice_SOURCES =                \
 deploymentmanager.cc                      \
 emaneeventserver.cc                       \
 erroreventreceiver.cc                     \
 eventdirector.cc                          \
 eventserviceconfiguration.cc              \
 eventgeneratorconfiguration.cc

EXTRA_DIST=                      \
 deploymentmanager.h             \
 erroreventreceiver.h            \
 eventdirector.h                 \
 eventserviceconfiguration.h     \
 eventgeneratorconfiguration.h

emaneeventservice_LDADD =                           \
 -L@top_srcdir@/src/libemanexmlparser/.libs         \
 -L@top_srcdir@/src/libemane/.libs                  \
 -lemanexmlparser                                   \
 -lemane                                            \
 $(LIBXML2_LIBS)                                    \
 $(ACE_LIBS)
