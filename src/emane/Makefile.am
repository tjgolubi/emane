bin_PROGRAMS = emane

INCLUDES =                            \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemanexmlparser

emane_CXXFLAGS =    \
 $(LIBXML2_CFLAGS)  \
 $(ACE_CFLAGS)      \
 $(AM_CFLAGS)

emane_SOURCES =  \
 emane.cc        \
 nemdirector.cc 

EXTRA_DIST=   \
 nemdirector.h

emane_LDADD =                                \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -L@top_srcdir@/src/libemane/.libs           \
 -lemanexmlparser                            \
 -lemane                                     \
 $(LIBXML2_LIBS)                             \
 $(ACE_LIBS)
