bin_PROGRAMS = emaneeventd

INCLUDES =                             \
 -I@top_srcdir@/include                \
 -I@top_srcdir@/src/libemanexmlparser  \
 -I@top_srcdir@/src/libemane

emaneeventd_CXXFLAGS =  \
 $(LIBXML2_CFLAGS)      \
 $(ACE_CFLAGS)          \
 $(AM_CFLAGS)

emaneeventd_SOURCES =                      \
 emaneeventd.cc                            \
 eventdaemonconfiguration.cc               \
 eventagentconfiguration.cc                \
 eventagentdirector.cc

EXTRA_DIST  =                        \
 eventdaemonconfiguration.h          \
 eventagentconfiguration.h           \
 eventagentdirector.h

emaneeventd_LDADD =                          \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -L@top_srcdir@/src/libemane/.libs           \
 -lemanexmlparser                            \
 -lemane                                     \
 $(LIBXML2_LIBS)                             \
 $(ACE_LIBS)
